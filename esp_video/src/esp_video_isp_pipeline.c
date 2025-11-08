/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

// #undef LOG_LOCAL_LEVEL
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

#include "linux/videodev2.h"
#include "esp_video_pipeline_isp.h"
#include "esp_video_ioctl.h"
#include "esp_video_isp_ioctl.h"
#include "esp_video_device_internal.h"
#include "esp_ipa.h"
#include "esp_cam_sensor.h"
#include "esp_video_device.h"

#define ISP_METADATA_BUFFER_COUNT   2
#define ISP_TASK_PRIORITY           8  // Lowered from 11 to reduce interference with camera/encoder pipeline
#define ISP_TASK_STACK_SIZE         4096
#define ISP_TASK_NAME               "isp_task"

#define UNUSED(x)                   (void)(x)

typedef struct esp_video_isp {
    int isp_fd;
    esp_video_isp_stats_t *isp_stats[ISP_METADATA_BUFFER_COUNT];

    esp_ipa_stats_t ipa_stats;
    esp_ipa_metadata_t metadata;

    int cam_fd;

    esp_ipa_pipeline_handle_t ipa_pipeline;

    esp_ipa_sensor_t sensor;
#if CONFIG_ESP_IPA_AF_ALGORITHM
    /* Focus information for IPA */
    esp_ipa_sensor_focus_t focus_info;
#endif

    int32_t prev_gain_index;
    uint32_t sensor_stats_seq;
    struct {
        uint8_t gain        : 1;
        uint8_t exposure    : 1;
        uint8_t stats       : 1;
        uint8_t awb         : 1;
        uint8_t group       : 1;
        uint8_t ae_level    : 1;
        uint8_t af_stime    : 1;
    } sensor_attr;

    TaskHandle_t task_handler;
#if CONFIG_ISP_PIPELINE_CONTROLLER_TASK_STACK_USE_PSRAM
    StaticTask_t *task_ptr;
    StackType_t *task_stack_ptr;
#endif
    
    // Frame boundary synchronization
    SemaphoreHandle_t frame_boundary_sem;
    
    // Async camera parameter update task
    TaskHandle_t camera_update_task;
    QueueHandle_t camera_update_queue;
} esp_video_isp_t;

// Camera update request structure for async task
typedef struct {
    uint32_t exposure_us;
    uint32_t gain_index;
    float gain_value;
} camera_update_request_t;

static const char *TAG = "ISP";
static esp_video_isp_t *s_esp_video_isp;

// Forward declaration for camera pipeline callback registration
extern void camera_pipeline_register_frame_callback(void (*cb)(void));

// Callback from camera pipeline when frame boundary occurs
static void isp_frame_boundary_notify(void)
{
    if (s_esp_video_isp && s_esp_video_isp->frame_boundary_sem) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_esp_video_isp->frame_boundary_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Print ISP statistics data
 *
 * @param stats ISP statistics pointer.
 *
 * @return None
 */
static void print_stats_info(const esp_ipa_stats_t *stats)
{
#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
    ESP_LOGD(TAG, "");
    ESP_LOGD(TAG, "Sequence: %llu", stats->seq);

    if (stats->flags & IPA_STATS_FLAGS_AWB) {
        ESP_LOGD(TAG, "Auto white balance:");
        for (int i = 0; i < ISP_AWB_REGIONS; i++) {
            const esp_ipa_stats_awb_t *awb_stats = &stats->awb_stats[i];

            ESP_LOGD(TAG, "  region:      %d", i);
            ESP_LOGD(TAG, "    counted:   %"PRIu32, awb_stats->counted);
            ESP_LOGD(TAG, "    sum_r:     %"PRIu32, awb_stats->sum_r);
            ESP_LOGD(TAG, "    sum_g:     %"PRIu32, awb_stats->sum_g);
            ESP_LOGD(TAG, "    sum_b:     %"PRIu32, awb_stats->sum_b);
        }
    }

    if (stats->flags & IPA_STATS_FLAGS_AE) {
        const esp_ipa_stats_ae_t *ae_stats = stats->ae_stats;

        ESP_LOGD(TAG, "Auto exposure:");
        for (int i = 0; i < ISP_AE_BLOCK_X_NUM; i++) {
            char print_buf[ISP_AE_BLOCK_X_NUM * 6];
            uint32_t offset = 0;

            for (int j = 0; j < ISP_AE_BLOCK_Y_NUM; j++) {
                int ret;

                ret = snprintf(print_buf + offset, sizeof(print_buf) - offset, " %3"PRIu32,
                               ae_stats[i * ISP_AE_BLOCK_Y_NUM + j].luminance);
                assert(ret > 0);
                offset += ret;
            }
            ESP_LOGD(TAG, "  [%s ]", print_buf);
        }
    }

    if (stats->flags & IPA_STATS_FLAGS_HIST) {
        const esp_ipa_stats_hist_t *hist_stats = stats->hist_stats;

        ESP_LOGD(TAG, "Histogram:");
        for (int i = 0; i < ISP_HIST_SEGMENT_NUMS; i++) {
            ESP_LOGD(TAG, "  %2d: %6"PRIu32, i, hist_stats[i].value);
        }
    }

    if (stats->flags & IPA_STATS_FLAGS_SHARPEN) {
        ESP_LOGD(TAG, "Sharpen high frequency pixel maximum value: %d", stats->sharpen_stats.value);
    }

    if (stats->flags & IPA_STATS_FLAGS_AF) {
        const esp_ipa_stats_af_t *af_stats = stats->af_stats;

        ESP_LOGD(TAG, "AF:");
        for (int i = 0; i < ISP_AF_WINDOW_NUM; i++) {
            ESP_LOGD(TAG, "  definition[%2d]: %"PRIu32, i, af_stats[i].definition);
            ESP_LOGD(TAG, "  luminance[%2d]:  %"PRIu32, i, af_stats[i].luminance);
        }
    }

    ESP_LOGD(TAG, "");
#endif
}

/**
 * @brief Print video device information.
 *
 * @param fd video device file description
 *
 * @return None
 */
static void print_dev_info(int fd)
{
#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
    struct v4l2_capability capability;

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "failed to get capability");
        return;
    }

    ESP_LOGD(TAG, "version: %d.%d.%d", (uint16_t)(capability.version >> 16),
             (uint8_t)(capability.version >> 8),
             (uint8_t)capability.version);
    ESP_LOGD(TAG, "driver:  %s", capability.driver);
    ESP_LOGD(TAG, "card:    %s", capability.card);
    ESP_LOGD(TAG, "bus:     %s", capability.bus_info);
    ESP_LOGD(TAG, "capabilities:");
    if (capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        ESP_LOGD(TAG, "\tVIDEO_CAPTURE");
    }
    if (capability.capabilities & V4L2_CAP_READWRITE) {
        ESP_LOGD(TAG, "\tREADWRITE");
    }
    if (capability.capabilities & V4L2_CAP_ASYNCIO) {
        ESP_LOGD(TAG, "\tASYNCIO");
    }
    if (capability.capabilities & V4L2_CAP_STREAMING) {
        ESP_LOGD(TAG, "\tSTREAMING");
    }
    if (capability.capabilities & V4L2_CAP_META_OUTPUT) {
        ESP_LOGD(TAG, "\tMETA_OUTPUT");
    }
    if (capability.capabilities & V4L2_CAP_DEVICE_CAPS) {
        ESP_LOGD(TAG, "device capabilities:");
        if (capability.device_caps & V4L2_CAP_VIDEO_CAPTURE) {
            ESP_LOGD(TAG, "\tVIDEO_CAPTURE");
        }
        if (capability.device_caps & V4L2_CAP_READWRITE) {
            ESP_LOGD(TAG, "\tREADWRITE");
        }
        if (capability.device_caps & V4L2_CAP_ASYNCIO) {
            ESP_LOGD(TAG, "\tASYNCIO");
        }
        if (capability.device_caps & V4L2_CAP_STREAMING) {
            ESP_LOGD(TAG, "\tSTREAMING");
        }
        if (capability.device_caps & V4L2_CAP_META_OUTPUT) {
            ESP_LOGD(TAG, "\tMETA_OUTPUT");
        }
    }
#endif
}

static void config_white_balance(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    bool rc = metadata->flags & IPA_METADATA_FLAGS_RG;
    bool bg = metadata->flags & IPA_METADATA_FLAGS_BG;

    if (rc && bg) {
        esp_video_isp_wb_t wb = {
            .enable = true,
            .red_gain = metadata->red_gain,
            .blue_gain = metadata->blue_gain
        };

        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_WB;
        control[0].p_u8     = (uint8_t *)&wb;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set white balance");
        }
    } else if (rc) {
        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_RED_BALANCE;
        control[0].value    = metadata->red_gain * V4L2_CID_RED_BALANCE_DEN;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set red balance");
        }
    } else if (bg) {
        controls.ctrl_class = V4L2_CTRL_CLASS_USER;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_BLUE_BALANCE;
        control[0].value    = metadata->blue_gain * V4L2_CID_BLUE_BALANCE_DEN;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set blue balance");
        }
    }
}

static void config_bayer_filter(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_video_isp_bf_t bf;

    if (metadata->flags & IPA_METADATA_FLAGS_BF) {
        bf.enable = true;
        bf.level = metadata->bf.level;
        for (int i = 0; i < ISP_BF_TEMPLATE_X_NUMS; i++) {
            for (int j = 0; j < ISP_BF_TEMPLATE_Y_NUMS; j++) {
                bf.matrix[i][j] = metadata->bf.matrix[i][j];
            }
        }

        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_BF;
        control[0].p_u8     = (uint8_t *)&bf;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set bayer filter");
        }
    }
}

static void config_demosaic(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_video_isp_demosaic_t demosaic;

    if (metadata->flags & IPA_METADATA_FLAGS_DM) {
        demosaic.enable = true;
        demosaic.gradient_ratio = metadata->demosaic.gradient_ratio;

        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_DEMOSAIC;
        control[0].p_u8     = (uint8_t *)&demosaic;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set demosaic");
        }
    }
}

static void config_sharpen(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_video_isp_sharpen_t sharpen;

    if (metadata->flags & IPA_METADATA_FLAGS_SH) {
        sharpen.enable = true;
        sharpen.h_thresh = metadata->sharpen.h_thresh;
        sharpen.l_thresh = metadata->sharpen.l_thresh;
        sharpen.h_coeff = metadata->sharpen.h_coeff;
        sharpen.m_coeff = metadata->sharpen.m_coeff;
        for (int i = 0; i < ISP_SHARPEN_TEMPLATE_X_NUMS; i++) {
            for (int j = 0; j < ISP_SHARPEN_TEMPLATE_Y_NUMS; j++) {
                sharpen.matrix[i][j] = metadata->sharpen.matrix[i][j];
            }
        }

        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_SHARPEN;
        control[0].p_u8     = (uint8_t *)&sharpen;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set sharpen");
        }
    }
}

static void config_gamma(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_video_isp_gamma_t gamma;

    if (metadata->flags & IPA_METADATA_FLAGS_GAMMA) {
        gamma.enable = true;
        for (int i = 0; i < ISP_GAMMA_CURVE_POINTS_NUM; i++) {
            gamma.points[i].x = metadata->gamma.x[i];
            gamma.points[i].y = metadata->gamma.y[i];
        }

        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_GAMMA;
        control[0].p_u8     = (uint8_t *)&gamma;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set GAMMA");
        }
    }
}

static void config_ccm(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_video_isp_ccm_t ccm;

    if (metadata->flags & IPA_METADATA_FLAGS_CCM) {
        ccm.enable = true;
        for (int i = 0; i < ISP_CCM_DIMENSION; i++) {
            for (int j = 0; j < ISP_CCM_DIMENSION; j++) {
                ccm.matrix[i][j] = metadata->ccm.matrix[i][j];
            }
        }

        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_CCM;
        control[0].p_u8     = (uint8_t *)&ccm;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set CCM");
        }
    }
}

static void config_color(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    if (metadata->flags & IPA_METADATA_FLAGS_BR) {
        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_BRIGHTNESS;
        control[0].value    = metadata->brightness;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set brightness");
        }
    }

    if (metadata->flags & IPA_METADATA_FLAGS_CN) {
        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_CONTRAST;
        control[0].value    = metadata->contrast;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set contrast");
        }
    }

    if (metadata->flags & IPA_METADATA_FLAGS_ST) {
        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_SATURATION;
        control[0].value    = metadata->saturation;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set saturation");
        }
    }

    if (metadata->flags & IPA_METADATA_FLAGS_HUE) {
        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_HUE;
        control[0].value    = metadata->hue;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set hue");
        }
    }
}

static void config_exposure_and_gain(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    float target_gain = 0.0;
    int32_t gain_index = -1;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];


    if (metadata->flags & IPA_METADATA_FLAGS_GN) {
        // Prefer INTEGER control if available; fallback to MENU search
        struct v4l2_query_ext_ctrl qctrl = {0};
        qctrl.id = V4L2_CID_GAIN;
        if (ioctl(isp->cam_fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0 && qctrl.type == V4L2_CTRL_TYPE_INTEGER) {
            // Direct integer scaling: value ~= target_gain * minimum
            int64_t minv = qctrl.minimum;
            int64_t maxv = qctrl.maximum;
            int64_t step = qctrl.step > 0 ? qctrl.step : 1;
            float tg = metadata->gain;
            // clamp tg to [1.0, max/min]
            float tg_max = (float)maxv / (float)minv;
            if (tg < 1.0f) tg = 1.0f;
            if (tg > tg_max) tg = tg_max;
            int64_t raw = (int64_t)llroundf(tg * (float)minv);
            // round to step
            int64_t off = raw - minv;
            off = ((off + step / 2) / step) * step;
            raw = minv + off;
            if (raw < minv) raw = minv;
            if (raw > maxv) raw = maxv;
            gain_index = (int32_t)raw;
            target_gain = (float)gain_index / (float)minv;
        } else {
            // Fallback: integer menu binary search (previous behavior)
            int ret;
            int32_t base_gain;
            int32_t gain_value;
            uint32_t cur_index;
            uint32_t left_index;
            uint32_t right_index;
            struct v4l2_querymenu qmenu;

            qmenu.id = V4L2_CID_GAIN;
            // Query base (min) menu value
            struct v4l2_query_ext_ctrl qc2 = { .id = V4L2_CID_GAIN };
            if (ioctl(isp->cam_fd, VIDIOC_QUERY_EXT_CTRL, &qc2) != 0) {
                ESP_LOGE(TAG, "failed to query gain ctrl");
                return;
            }
            qmenu.index = qc2.minimum;
            ret = ioctl(isp->cam_fd, VIDIOC_QUERYMENU, &qmenu);
            if (ret) {
                ESP_LOGE(TAG, "failed to query gain min menu");
                return;
            }

            gain_value = qmenu.value * metadata->gain;
            base_gain = qmenu.value;
            left_index = qc2.minimum;
            right_index = qc2.maximum;
            cur_index = (left_index + right_index) / 2;

            int max_inter = qc2.maximum - qc2.minimum;
            do {
                if (max_inter-- <= 0) {
                    ESP_LOGE(TAG, "failed to search target gain");
                    break;
                }

                qmenu.index = cur_index;
                if (ioctl(isp->cam_fd, VIDIOC_QUERYMENU, &qmenu)) {
                    ESP_LOGE(TAG, "failed to query gain menu");
                    return;
                }

                if (gain_value > qmenu.value) {
                    left_index = cur_index;
                    cur_index = (cur_index + right_index) / 2;
                } else if (gain_value < qmenu.value) {
                    right_index = cur_index;
                    cur_index = (cur_index + left_index) / 2;
                } else {
                    gain_index = cur_index;
                    target_gain = (float)qmenu.value / base_gain;
                    break;
                }

                int index_diff = right_index - left_index;
                if (index_diff <= 1) {
                    // choose closer bound
                    uint32_t left_gain, right_gain;
                    qmenu.index = left_index;
                    if (ioctl(isp->cam_fd, VIDIOC_QUERYMENU, &qmenu)) return;
                    left_gain = qmenu.value;
                    qmenu.index = right_index;
                    if (ioctl(isp->cam_fd, VIDIOC_QUERYMENU, &qmenu)) return;
                    right_gain = qmenu.value;
                    if ((gain_value - (int32_t)left_gain) > ((int32_t)right_gain - gain_value)) {
                        gain_index = right_index;
                        target_gain = (float)right_gain / base_gain;
                    } else {
                        gain_index = left_index;
                        target_gain = (float)left_gain / base_gain;
                    }
                    break;
                }
            } while (1);

            if (gain_index < 0) {
                ESP_LOGE(TAG, "failed to find gain=%0.4f", metadata->gain);
                return;
            }
        }

        // Don't remove GN flag if ET is also set - we need both for GROUP_EXP_GAIN
        if (isp->prev_gain_index == gain_index && !(metadata->flags & IPA_METADATA_FLAGS_ET)) {
            metadata->flags &= ~IPA_METADATA_FLAGS_GN;
        }
    }

    // If GROUP_EXP_GAIN is available and either ET or GN is set, force both flags
    // to ensure atomic updates even when only one value changes
    if (isp->sensor_attr.group) {
        if ((metadata->flags & IPA_METADATA_FLAGS_ET) && !(metadata->flags & IPA_METADATA_FLAGS_GN)) {
            // ET set but not GN - force GN with current value
            metadata->flags |= IPA_METADATA_FLAGS_GN;
            if (gain_index < 0) {
                // No gain computed yet - use previous value
                gain_index = isp->prev_gain_index;
                target_gain = isp->sensor.cur_gain;
            }
        } else if ((metadata->flags & IPA_METADATA_FLAGS_GN) && !(metadata->flags & IPA_METADATA_FLAGS_ET)) {
            // GN set but not ET - force ET with current value
            metadata->flags |= IPA_METADATA_FLAGS_ET;
            if (!metadata->exposure) {
                metadata->exposure = isp->sensor.cur_exposure;
            }
        }
    }

    if ((metadata->flags & IPA_METADATA_FLAGS_ET) &&
            (metadata->flags & IPA_METADATA_FLAGS_GN) &&
            isp->sensor_attr.group) {
        esp_cam_sensor_gh_exp_gain_t group;
        // Sanitize exposure against device limits (V4L2_CID_EXPOSURE_ABSOLUTE is in 100us units)
        uint32_t apply_us = metadata->exposure;
        {
#if CONFIG_CAMERA_IMX708
            struct v4l2_query_ext_ctrl qexp = {0};
            qexp.id = V4L2_CID_EXPOSURE_ABSOLUTE;
            if (ioctl(isp->cam_fd, VIDIOC_QUERY_EXT_CTRL, &qexp) == 0 && qexp.step > 0) {
                int64_t min_us = (int64_t)qexp.minimum * 100LL;
                int64_t max_us = (int64_t)qexp.maximum * 100LL;
                int64_t step_us = (int64_t)qexp.step * 100LL;
                int64_t au = (int64_t)apply_us;
                // round to nearest step
                au = (au + step_us / 2) / step_us;
                au *= step_us;
                if (au < min_us) au = min_us;
                if (au > max_us) au = max_us;
                apply_us = (uint32_t)au;
            }
#endif
        }
        group.exposure_us = apply_us;
        group.gain_index = gain_index;

        // Send camera update request to async task (non-blocking!)
        // This eliminates I2C stalls by moving 14-15ms ioctl to low-priority background task
        // Check if queue is initialized (it may not be during early init)
        if (isp->camera_update_queue != NULL) {
            camera_update_request_t req = {
                .exposure_us = apply_us,
                .gain_index = gain_index,
                .gain_value = target_gain
            };
            
            if (xQueueSend(isp->camera_update_queue, &req, 0) == pdTRUE) {
                // Update tracking variables optimistically (actual I2C happens in background)
                isp->prev_gain_index = gain_index;
                ESP_LOGD(TAG, "ISP: Queued camera update (exp=%lu gain=%lu)", apply_us, gain_index);
            } else {
                ESP_LOGW(TAG, "ISP: Camera update queue FULL! Dropping update (exp=%lu gain=%lu)", 
                         apply_us, gain_index);
            }
        } else {
            // Queue not initialized yet - apply update synchronously for now
            ESP_LOGD(TAG, "Camera update queue not ready, skipping update");
        }
    } else {
        if ((metadata->flags & IPA_METADATA_FLAGS_ET) &&
                isp->sensor_attr.exposure) {
            uint32_t apply_us = metadata->exposure;
            {
#if CONFIG_CAMERA_IMX708
                struct v4l2_query_ext_ctrl qexp = {0};
                qexp.id = V4L2_CID_EXPOSURE_ABSOLUTE;
                if (ioctl(isp->cam_fd, VIDIOC_QUERY_EXT_CTRL, &qexp) == 0 && qexp.step > 0) {
                    int64_t min_us = (int64_t)qexp.minimum * 100LL;
                    int64_t max_us = (int64_t)qexp.maximum * 100LL;
                    int64_t step_us = (int64_t)qexp.step * 100LL;
                    int64_t au = (int64_t)apply_us;
                    au = (au + step_us / 2) / step_us;
                    au *= step_us;
                    if (au < min_us) au = min_us;
                    if (au > max_us) au = max_us;
                    apply_us = (uint32_t)au;
                }
#endif
            }
            controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
            controls.count      = 1;
            controls.controls   = control;
            control[0].id       = V4L2_CID_EXPOSURE_ABSOLUTE;
            control[0].value    = (int32_t)apply_us / 100;
            if (ioctl(isp->cam_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
                ESP_LOGE(TAG, "failed to set exposure time");
            } else {
                isp->sensor.cur_exposure = apply_us;
                        }
        }

        if ((metadata->flags & IPA_METADATA_FLAGS_GN) &&
                isp->sensor_attr.gain) {
            controls.ctrl_class = V4L2_CID_USER_CLASS;
            controls.count      = 1;
            controls.controls   = control;
            control[0].id       = V4L2_CID_GAIN;
            control[0].value    = gain_index;
            if (ioctl(isp->cam_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
                ESP_LOGE(TAG, "failed to set pixel gain");
            } else {
                isp->sensor.cur_gain = target_gain;
                isp->prev_gain_index = gain_index;
            }
        }
    }
}

#if ESP_VIDEO_ISP_DEVICE_LSC
static void config_lsc(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_video_isp_lsc_t lsc;

    if (metadata->flags & IPA_METADATA_FLAGS_LSC) {
        lsc.enable = true;
        lsc.gain_r = metadata->lsc.gain_r;
        lsc.gain_gr = metadata->lsc.gain_gr;
        lsc.gain_gb = metadata->lsc.gain_gb;
        lsc.gain_b = metadata->lsc.gain_b;
        lsc.lsc_gain_size = metadata->lsc.lsc_gain_array_size;

        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_LSC;
        control[0].p_u8     = (uint8_t *)&lsc;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set LSC");
        }
    }
}
#endif

static void config_sensor_ae_target_level(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    if ((metadata->flags & IPA_METADATA_FLAGS_AETL) &&
            isp->sensor_attr.ae_level) {
        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_CAMERA_AE_LEVEL;
        control[0].value    = metadata->ae_target_level;
        if (ioctl(isp->cam_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set sensor AE target level");
        } else {
            isp->sensor.cur_ae_target_level = metadata->ae_target_level;
        }
    }
}

static void config_awb(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_video_isp_awb_t awb;

    if (metadata->flags & IPA_METADATA_FLAGS_AWB) {
        esp_ipa_awb_range_t *range = &metadata->awb;

        awb.enable = true;
        awb.green_max = range->green_max;
        awb.green_min = range->green_min;
        awb.rg_max = range->rg_max;
        awb.rg_min = range->rg_min;
        awb.bg_max = range->bg_max;
        awb.bg_min = range->bg_min;

        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_AWB;
        control[0].p_u8     = (uint8_t *)&awb;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set AWB");
        }
    }
}

static void config_statistics_region(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    if (metadata->flags & IPA_METADATA_FLAGS_SR) {
        esp_ipa_region_t *sr = &metadata->stats_region;
        struct v4l2_selection selection;

        memset(&selection, 0, sizeof(selection));
        selection.type = V4L2_BUF_TYPE_META_CAPTURE;
        selection.r.left = sr->left;
        selection.r.width = sr->width;
        selection.r.top = sr->top;
        selection.r.height = sr->height;
        if (ioctl(isp->isp_fd, VIDIOC_S_SELECTION, &selection) != 0) {
            ESP_LOGE(TAG, "failed to set selection");
        }
    }
}

static void config_af(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    esp_video_isp_af_t af;

    if (metadata->flags & IPA_METADATA_FLAGS_AF) {
        esp_ipa_af_t *ipa_af = &metadata->af;

        af.enable = true;
        af.edge_thresh = ipa_af->edge_thresh;
        memcpy(af.windows, ipa_af->windows, sizeof(isp_window_t) * ISP_AF_WINDOW_NUM);

        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_USER_ESP_ISP_AF;
        control[0].p_u8     = (uint8_t *)&af;
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set AF");
        }
    }
}

#if CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR
static void config_motor_position(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    // AF runs freely (no gating by AE stability)

    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    if (metadata->flags & IPA_METADATA_FLAGS_FP) {
        controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_FOCUS_ABSOLUTE;
        control[0].value    = metadata->focus_pos;
        if (ioctl(isp->cam_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            ESP_LOGE(TAG, "failed to set motor position");
            isp->focus_info.start_time = 0;
        } else {
            int64_t strat_time;

            controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
            controls.count      = 1;
            controls.controls   = control;
            control[0].id       = V4L2_CID_MOTOR_START_TIME;
            control[0].p_u8     = (uint8_t *)&strat_time;
            control[0].size     = sizeof(strat_time);
            if (ioctl(isp->cam_fd, VIDIOC_G_EXT_CTRLS, &controls) != 0) {
                ESP_LOGE(TAG, "failed to get motor start time");
                isp->focus_info.start_time = 0;
            } else {
                isp->focus_info.start_time = strat_time;
                isp->focus_info.cur_pos = metadata->focus_pos;
            }
        }
    }
}
#endif

static void config_isp_and_camera(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)
{
    // Configure ISP-only parameters (no camera I2C access, always safe)
    config_statistics_region(isp, metadata);
    config_bayer_filter(isp, metadata);
    config_demosaic(isp, metadata);
    config_sharpen(isp, metadata);
    config_gamma(isp, metadata);
    config_ccm(isp, metadata);
    config_color(isp, metadata);
#if ESP_VIDEO_ISP_DEVICE_LSC
    config_lsc(isp, metadata);
#endif
    config_awb(isp, metadata);
    config_af(isp, metadata);

    // Camera parameter updates now use Group Hold for atomic application at frame boundaries
    // to prevent I2C bus contention and DQBUF stalls during active streaming.
    
    if (!isp->sensor_attr.awb) {
        config_white_balance(isp, metadata);
    }
    config_sensor_ae_target_level(isp, metadata);
    config_exposure_and_gain(isp, metadata);
#if CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR
    config_motor_position(isp, metadata);
#endif
}

static void isp_stats_to_ipa_stats(esp_video_isp_stats_t *isp_stat, esp_ipa_stats_t *ipa_stats)
{
    ipa_stats->flags = 0;
    ipa_stats->seq = isp_stat->seq;

    if (isp_stat->flags & ESP_VIDEO_ISP_STATS_FLAG_AE) {
        esp_ipa_stats_ae_t *ipa_ae = &ipa_stats->ae_stats[0];
        isp_ae_result_t *isp_ae = &isp_stat->ae.ae_result;

        for (int i = 0; i < ISP_AE_BLOCK_X_NUM; i++) {
            for (int j = 0; j < ISP_AE_BLOCK_Y_NUM; j++) {
                ipa_ae[i * ISP_AE_BLOCK_Y_NUM + j].luminance = isp_ae->luminance[i][j];
            }
        }
        ipa_stats->flags |= IPA_STATS_FLAGS_AE;
    }

    if (isp_stat->flags & ESP_VIDEO_ISP_STATS_FLAG_AWB) {
        esp_ipa_stats_awb_t *ipa_awb = &ipa_stats->awb_stats[0];
        isp_awb_stat_result_t *isp_awb = &isp_stat->awb.awb_result;

        ipa_awb->counted = isp_awb->white_patch_num;
        ipa_awb->sum_r = isp_awb->sum_r;
        ipa_awb->sum_g = isp_awb->sum_g;
        ipa_awb->sum_b = isp_awb->sum_b;
        ipa_stats->flags |= IPA_STATS_FLAGS_AWB;
    }

    if (isp_stat->flags & ESP_VIDEO_ISP_STATS_FLAG_HIST) {
        esp_ipa_stats_hist_t *ipa_hist = &ipa_stats->hist_stats[0];
        isp_hist_result_t *isp_hist = &isp_stat->hist.hist_result;

        for (int i = 0; i < ISP_HIST_SEGMENT_NUMS; i++) {
            ipa_hist[i].value = isp_hist->hist_value[i];
        }
        ipa_stats->flags |= IPA_STATS_FLAGS_HIST;
    }

    if (isp_stat->flags & ESP_VIDEO_ISP_STATS_FLAG_SHARPEN) {
        esp_ipa_stats_sharpen_t *ipa_sharpen = &ipa_stats->sharpen_stats;
        esp_isp_sharpen_evt_data_t *isp_sharpen = &isp_stat->sharpen;

        ipa_sharpen->value = isp_sharpen->high_freq_pixel_max;
        ipa_stats->flags |= IPA_STATS_FLAGS_SHARPEN;
    }

    if (isp_stat->flags & ESP_VIDEO_ISP_STATS_FLAG_AF) {
        esp_ipa_stats_af_t *ipa_af = ipa_stats->af_stats;
        isp_af_result_t *isp_af = &isp_stat->af.af_result;

        for (int i = 0; i < ISP_AF_WINDOW_NUM; i++) {
            ipa_af[i].definition = isp_af->definition[i];
            ipa_af[i].luminance = isp_af->luminance[i];
        }
        ipa_stats->flags |= IPA_STATS_FLAGS_AF;
    }
}

static void get_sensor_state(esp_video_isp_t *isp, int index)
{
    int ret;
    struct v4l2_format format;

    if (isp->sensor_attr.awb) {
        isp->isp_stats[index]->flags &= ~ESP_VIDEO_ISP_STATS_FLAG_AWB;
    }

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(isp->cam_fd, VIDIOC_G_FMT, &format);
    if (ret == 0) {
        isp->sensor.width = format.fmt.pix.width;
        isp->sensor.height = format.fmt.pix.height;
    }

    if (isp->sensor_attr.stats) {
        struct v4l2_ext_controls controls;
        struct v4l2_ext_control control[1];
        esp_cam_sensor_stats_t sensor_stats;

        controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_CAMERA_STATS;
        control[0].p_u8     = (uint8_t *)&sensor_stats;
        control[0].size     = sizeof(sensor_stats);
        ret = ioctl(isp->cam_fd, VIDIOC_G_EXT_CTRLS, &controls);
        if (ret == 0) {
            if (isp->sensor_stats_seq != sensor_stats.seq) {
                if (sensor_stats.flags & ESP_CAM_SENSOR_STATS_FLAG_AGC_GAIN) {
                    isp->sensor.cur_gain = sensor_stats.agc_gain;
                }

                if (sensor_stats.flags & ESP_CAM_SENSOR_STATS_FLAG_WB_GAIN) {
                    isp_awb_stat_result_t *awb = &isp->isp_stats[index]->awb.awb_result;

                    isp->isp_stats[index]->flags |= ESP_VIDEO_ISP_STATS_FLAG_AWB;
                    awb->white_patch_num = 1;
                    awb->sum_r = sensor_stats.wb_avg.red_avg;
                    awb->sum_g = sensor_stats.wb_avg.green_avg;
                    awb->sum_b = sensor_stats.wb_avg.blue_avg;
                }

                isp->sensor_stats_seq = sensor_stats.seq;
            }
        }
    }
}

// Async camera parameter update task - runs at low priority to avoid blocking main pipeline
static void camera_update_task(void *p)
{
    esp_video_isp_t *isp = (esp_video_isp_t *)p;
    camera_update_request_t req;
    
    ESP_LOGI(TAG, "Camera update task started (low priority, non-blocking)");
    
    while (1) {
        // Wait for update request from ISP task (blocking, no CPU usage when idle)
        if (xQueueReceive(isp->camera_update_queue, &req, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "CamUpdateTask: Received request (exp=%lu gain=%lu)", 
                     req.exposure_us, req.gain_index);
            
            // Apply camera parameters via GROUP_EXP_GAIN
            // This I2C transaction (14-15ms) happens in background at low priority
            if (isp->sensor_attr.group) {
                esp_cam_sensor_gh_exp_gain_t group;
                group.exposure_us = req.exposure_us;
                group.gain_index = req.gain_index;
                
                struct v4l2_ext_controls controls;
                struct v4l2_ext_control control[1];
                
                controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
                controls.count = 1;
                controls.controls = control;
                control[0].id = V4L2_CID_CAMERA_GROUP;
                control[0].p_u8 = (uint8_t *)&group;
                control[0].size = sizeof(group);
                
                int64_t t0 = esp_timer_get_time();
                if (ioctl(isp->cam_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
                    ESP_LOGE(TAG, "CamUpdateTask: GROUP_EXP_GAIN failed");
                } else {
                    int64_t t1 = esp_timer_get_time();
                    int64_t dt = t1 - t0;
                    ESP_LOGD(TAG, "CamUpdateTask: GROUP_EXP_GAIN completed in %lld us", dt);
                    if (dt > 20000) { // Log if > 20ms (slower than expected)
                        ESP_LOGW(TAG, "CamUpdateTask: Slow I2C (%lld us) exp=%lu gain=%lu", 
                                 dt, req.exposure_us, req.gain_index);
                    }
                    // Update tracking variables
                    isp->sensor.cur_exposure = req.exposure_us;
                    isp->sensor.cur_gain = req.gain_value;
                }
            }
        }
    }
}

static void isp_task(void *p)
{
    esp_err_t ret;
    struct v4l2_buffer buf;
    esp_video_isp_t *isp = (esp_video_isp_t *)p;
    
    // Frame skipping to reduce ISP interference with camera pipeline
    // Process only every Nth frame to lower ISP task CPU/memory pressure
    // Higher value = less interference but slower AE/AWB/AG convergence
    // Optimal: 3 = 5fps processing (good balance of performance vs convergence speed)
    static const uint32_t ISP_PROCESS_INTERVAL = 3; // Process 1 out of every 3 frames (5fps)
    uint32_t frame_counter = 0;

    while (1) {
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_META_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(isp->isp_fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "failed to receive video frame");
            continue;
        }

        // DISABLED: get_sensor_state() causes I2C contention with camera_update_task
        // Both tasks accessing I2C simultaneously → mutex lock → deadlock → watchdog restart
        // We already have sensor state from IPA calculations, don't need to read it again
        // get_sensor_state(isp, buf.index);

        // Only process IPA on selected frames to reduce interference
        bool should_process = ((frame_counter++ % ISP_PROCESS_INTERVAL) == 0);
        
        if (should_process) {
            isp_stats_to_ipa_stats(isp->isp_stats[buf.index], &isp->ipa_stats);
        }
        
        // Always QBUF immediately to avoid blocking V4L2 subsystem
        if (ioctl(isp->isp_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "failed to queue video frame");
        }
        
        if (!should_process) {
            // Skip IPA processing for this frame
            // Sleep briefly to avoid hammering DQBUF and competing with pipeline
            // This yields CPU to camera_update_task and other low-priority tasks
            // At 5fps (interval=3), we skip 2 frames, sleep ~10ms each = 20ms total breathing room
            vTaskDelay(pdMS_TO_TICKS(10)); // ~10ms = ~0.15 frame period, minimal but effective
            continue;
        }
        
        print_stats_info(&isp->ipa_stats);

        isp->metadata.flags = 0;
        ret = esp_ipa_pipeline_process(isp->ipa_pipeline, &isp->ipa_stats, &isp->sensor, &isp->metadata);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to process image algorithm");
            continue;
        }

        #if CONFIG_ESP_VIDEO_ENABLE_CAMERA_TELEMETRY
        // Compact telemetry: log key metadata at a low rate for tuning
        {
            static uint32_t s_log_counter = 0;
            if ((s_log_counter++ % 5) == 0) { // ~1 line per ~30 stats frames
                char buf[192];
                int n = 0;
                n += snprintf(buf + n, sizeof(buf) - n, "[META]");
                // Report applied exposure from the camera driver when possible
                {
#if CONFIG_CAMERA_IMX708
                    struct v4l2_ext_controls c = {0};
                    struct v4l2_ext_control  ctrl[1] = {0};
                    c.ctrl_class = V4L2_CID_CAMERA_CLASS;
                    c.count = 1;
                    c.controls = ctrl;
                    ctrl[0].id = V4L2_CID_EXPOSURE_ABSOLUTE; // units: 100us
                    if (ioctl(isp->cam_fd, VIDIOC_G_EXT_CTRLS, &c) == 0) {
                        unsigned applied_us = (unsigned)ctrl[0].value * 100U;
                        n += snprintf(buf + n, sizeof(buf) - n, " exp=%uus", applied_us);
                    } else
#endif
                    if (isp->sensor.cur_exposure) {
                        n += snprintf(buf + n, sizeof(buf) - n, " exp=%uus", (unsigned)isp->sensor.cur_exposure);
                    } else if (isp->metadata.flags & IPA_METADATA_FLAGS_ET) {
                        n += snprintf(buf + n, sizeof(buf) - n, " exp=%uus", (unsigned)isp->metadata.exposure);
                    }
                }
                if (isp->metadata.flags & IPA_METADATA_FLAGS_GN) {
                    n += snprintf(buf + n, sizeof(buf) - n, " gain=%.2f", (double)isp->metadata.gain);
                } else if (isp->sensor.cur_gain > 0.0f) {
                    n += snprintf(buf + n, sizeof(buf) - n, " gain=%.2f", (double)isp->sensor.cur_gain);
                }
                // Try to fetch current analogue/digital gains for detailed logging
                {
#if CONFIG_CAMERA_IMX708
                    struct v4l2_ext_controls c = {0};
                    struct v4l2_ext_control  ctrl[1] = {0};
                    // Analogue gain (raw code)
                    c.ctrl_class = V4L2_CID_CAMERA_CLASS;
                    c.count = 1;
                    c.controls = ctrl;
                    ctrl[0].id = V4L2_CID_ANALOGUE_GAIN;
                    if (ioctl(isp->cam_fd, VIDIOC_G_EXT_CTRLS, &c) == 0) {
                        n += snprintf(buf + n, sizeof(buf) - n, " ag=%ld", (long)ctrl[0].value);
                    }
                    // Digital gain (code/256)
                    memset(&c, 0, sizeof(c));
                    memset(ctrl, 0, sizeof(ctrl));
                    c.ctrl_class = V4L2_CID_CAMERA_CLASS;
                    c.count = 1;
                    c.controls = ctrl;
                    ctrl[0].id = V4L2_CID_DIGITAL_GAIN;
                    if (ioctl(isp->cam_fd, VIDIOC_G_EXT_CTRLS, &c) == 0) {
                        float dg = (ctrl[0].value > 0) ? (ctrl[0].value / 256.0f) : 0.0f;
                        n += snprintf(buf + n, sizeof(buf) - n, " dg=%.2f", (double)dg);
                    }
#endif
                }

                if (isp->metadata.flags & IPA_METADATA_FLAGS_RG) {
                    n += snprintf(buf + n, sizeof(buf) - n, " rG=%.3f", (double)isp->metadata.red_gain);
                }
                if (isp->metadata.flags & IPA_METADATA_FLAGS_BG) {
                    n += snprintf(buf + n, sizeof(buf) - n, " bG=%.3f", (double)isp->metadata.blue_gain);
                }
                if (isp->metadata.flags & IPA_METADATA_FLAGS_AETL) {
                    n += snprintf(buf + n, sizeof(buf) - n, " ae_tl=%u", (unsigned)isp->metadata.ae_target_level);
                } else if (isp->sensor.cur_ae_target_level) {
                    n += snprintf(buf + n, sizeof(buf) - n, " ae_tl=%u", (unsigned)isp->sensor.cur_ae_target_level);
                }
                if (isp->metadata.flags & IPA_METADATA_FLAGS_CN) {
                    n += snprintf(buf + n, sizeof(buf) - n, " con=%u", (unsigned)isp->metadata.contrast);
                }
                if (isp->metadata.flags & IPA_METADATA_FLAGS_ST) {
                    n += snprintf(buf + n, sizeof(buf) - n, " sat=%u", (unsigned)isp->metadata.saturation);
                }
                if (isp->metadata.flags & IPA_METADATA_FLAGS_BR) {
                    n += snprintf(buf + n, sizeof(buf) - n, " br=%u", (unsigned)isp->metadata.brightness);
                }
                if (isp->metadata.flags & IPA_METADATA_FLAGS_FP) {
                    n += snprintf(buf + n, sizeof(buf) - n, " focus=%u", (unsigned)isp->metadata.focus_pos);
                }
                ESP_LOGI(TAG, "%s", buf);
            }
        }
        #endif // CONFIG_ESP_VIDEO_ENABLE_CAMERA_TELEMETRY

        // Async camera update task handles all I2C operations in background.
        // No rate limiting needed - queue naturally handles backpressure.
        // ISP task applies every update immediately without blocking.
        {
            // For camera parameter updates (slow I2C ~14-15ms), don't wait for semaphore
            // since the Group Hold mechanism already ensures atomic application at frame boundaries.
            // Just apply immediately - the camera will latch the values at the next frame start.
            config_isp_and_camera(isp, &isp->metadata);
            
            // Consume the semaphore signal if it arrived (prevents accumulation)
            xSemaphoreTake(isp->frame_boundary_sem, 0);
        }
    }

    vTaskDelete(NULL);
}

static esp_err_t init_cam_dev(const esp_video_isp_config_t *config, esp_video_isp_t *isp)
{
    int fd;
    int owner = 0;
    esp_err_t ret;
    struct v4l2_format format;
    struct v4l2_query_ext_ctrl qctrl;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
#if CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR
    esp_cam_motor_format_t motor_format;
#endif

    fd = open(config->cam_dev, O_RDWR);
    ESP_RETURN_ON_FALSE(fd > 0, ESP_ERR_INVALID_ARG, TAG, "failed to open %s", config->cam_dev);
    print_dev_info(fd);

    ret = ioctl(fd, VIDIOC_SET_OWNER, &owner);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to set owner");

    qctrl.id = V4L2_CID_GAIN;
    ret = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (ret == 0) {
        controls.ctrl_class = V4L2_CID_USER_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_GAIN;
        control[0].value    = qctrl.default_value;
        ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
        ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to set gain");

        isp->sensor.min_gain = 1.0;
        if (qctrl.type == V4L2_CTRL_TYPE_INTEGER) {
            isp->sensor.max_gain  = (float)qctrl.maximum / qctrl.minimum;
            isp->sensor.cur_gain  = (float)control[0].value / qctrl.minimum;
            isp->sensor.step_gain = (float)qctrl.step / qctrl.minimum;
        } else if (qctrl.type == V4L2_CTRL_TYPE_INTEGER_MENU) {
            int64_t min;
            struct v4l2_querymenu qmenu;

            qmenu.id = V4L2_CID_GAIN;
            qmenu.index = qctrl.minimum;
            ret = ioctl(fd, VIDIOC_QUERYMENU, &qmenu);
            ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to query gain min menu");
            min = qmenu.value;

            qmenu.index = qctrl.maximum;
            ret = ioctl(fd, VIDIOC_QUERYMENU, &qmenu);
            ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to query gain max menu");
            isp->sensor.max_gain = (float)qmenu.value / min;

            qmenu.index = control[0].value;
            ret = ioctl(fd, VIDIOC_QUERYMENU, &qmenu);
            ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to query gain current menu");
            isp->sensor.cur_gain = (float)qmenu.value / min;

            isp->sensor.step_gain = 0.0;
        }

        isp->sensor_attr.gain = 1;

        ESP_LOGD(TAG, "Sensor gain:");
        ESP_LOGD(TAG, "  min:     %0.4f", isp->sensor.min_gain);
        ESP_LOGD(TAG, "  max:     %0.4f", isp->sensor.max_gain);
        ESP_LOGD(TAG, "  step:    %0.4f", isp->sensor.step_gain);
        ESP_LOGD(TAG, "  current: %0.4f", isp->sensor.cur_gain);
    } else {
        ESP_LOGD(TAG, "V4L2_CID_GAIN is not supported");
    }

    qctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
#if CONFIG_CAMERA_IMX708
    ret = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (ret == 0) {
        controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_EXPOSURE_ABSOLUTE;
        // Prefer a safer initial exposure (target ~4ms) to avoid underexposed start
        int32_t init_val = qctrl.default_value; // units: 100us
        int32_t step = (qctrl.step > 0) ? (int32_t)qctrl.step : 1;
        int32_t minv = (int32_t)qctrl.minimum;
        int32_t maxv = (int32_t)qctrl.maximum;
        // Aim ~4.0ms (40 units) if default is too low
        const int32_t prefer = 40; // 4.0ms
        if (init_val < prefer) {
            init_val = prefer;
        }
        // Round to step and clamp
        if (step > 1) {
            int32_t k = (init_val + step/2) / step;
            init_val = k * step;
        }
        if (init_val < minv) init_val = ((minv + step - 1) / step) * step;
        if (init_val > maxv) init_val = (maxv / step) * step;
        control[0].value    = init_val;
        ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls);
        ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to set exposure time");

        isp->sensor.min_exposure = qctrl.minimum * 100;
        isp->sensor.max_exposure = qctrl.maximum * 100;
        isp->sensor.step_exposure = qctrl.step * 100;
        isp->sensor.cur_exposure = control[0].value * 100;

        isp->sensor_attr.exposure = 1;

        ESP_LOGD(TAG, "Exposure time:");
        ESP_LOGD(TAG, "  min:     %"PRIi64, qctrl.minimum);
        ESP_LOGD(TAG, "  max:     %"PRIi64, qctrl.maximum);
        ESP_LOGD(TAG, "  step:    %"PRIu64, qctrl.step);
        ESP_LOGD(TAG, "  current: %"PRIi32, control[0].value);
    } else {
        ESP_LOGD(TAG, "V4L2_CID_EXPOSURE_ABSOLUTE is not supported");
    }
#else
    (void)ret;
#endif

    qctrl.id = V4L2_CID_CAMERA_STATS;
    ret = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (ret == 0) {
        esp_cam_sensor_stats_t sensor_stats;

        controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_CAMERA_STATS;
        control[0].p_u8     = (uint8_t *)&sensor_stats;
        control[0].size     = sizeof(sensor_stats);
        ret = ioctl(fd, VIDIOC_G_EXT_CTRLS, &controls);
        ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to get sensor statistics");

        if (sensor_stats.flags & ESP_CAM_SENSOR_STATS_FLAG_WB_GAIN) {
            isp->sensor_attr.awb = 1;
        }

        isp->sensor_attr.stats = 1;
    } else {
        ESP_LOGD(TAG, "V4L2_CID_CAMERA_STATS is not supported");
    }

    qctrl.id = V4L2_CID_CAMERA_GROUP;
    ESP_LOGD(TAG, "Querying V4L2_CID_CAMERA_GROUP (0x%08x)...", V4L2_CID_CAMERA_GROUP);
    ret = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (ret == 0) {
        isp->sensor_attr.group = 1;
        ESP_LOGI(TAG, "V4L2_CID_CAMERA_GROUP supported - using batched exposure+gain updates");
    } else {
        ESP_LOGW(TAG, "V4L2_CID_CAMERA_GROUP not supported (ret=%d, errno=%d) - using separate exposure/gain updates", ret, errno);
    }

    qctrl.id = V4L2_CID_CAMERA_AE_LEVEL;
    ret = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (ret == 0) {
        isp->sensor_attr.ae_level = 1;

        controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_CAMERA_AE_LEVEL;
        control[0].value    = 0;
        ret = ioctl(fd, VIDIOC_G_EXT_CTRLS, &controls);
        ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to get AE target level");

        isp->sensor.min_ae_target_level = qctrl.minimum;
        isp->sensor.max_ae_target_level = qctrl.maximum;
        isp->sensor.step_ae_target_level = qctrl.step;
        isp->sensor.cur_ae_target_level = control[0].value;

        ESP_LOGD(TAG, "AE target level:");
        ESP_LOGD(TAG, "  min:     %"PRIi64, qctrl.minimum);
        ESP_LOGD(TAG, "  max:     %"PRIi64, qctrl.maximum);
        ESP_LOGD(TAG, "  step:    %"PRIu64, qctrl.step);
        ESP_LOGD(TAG, "  current: %"PRIi32, control[0].value);
    } else {
        ESP_LOGD(TAG, "V4L2_CID_CAMERA_AE_LEVEL is not supported");
    }

#if CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR
    qctrl.id = V4L2_CID_MOTOR_START_TIME;
    ret = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (ret == 0) {
        isp->sensor_attr.af_stime = 1;
    } else {
        ESP_LOGD(TAG, "V4L2_CID_MOTOR_START_TIME is not supported");
    }

    qctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
    ret = ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qctrl);
    if (ret == 0) {
        controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
        controls.count      = 1;
        controls.controls   = control;
        control[0].id       = V4L2_CID_FOCUS_ABSOLUTE;
        control[0].value    = 0;
        ret = ioctl(fd, VIDIOC_G_EXT_CTRLS, &controls);
        ESP_GOTO_ON_FALSE(ret == 0, ESP_ERR_NOT_SUPPORTED, fail_0, TAG, "failed to get AF absolute position code");

        isp->sensor.focus_info = &isp->focus_info;

        isp->focus_info.min_pos = qctrl.minimum;
        isp->focus_info.max_pos = qctrl.maximum;
        isp->focus_info.step_pos = qctrl.step;
        isp->focus_info.cur_pos = control[0].value;

        ESP_LOGD(TAG, "AF absolute position code:");
        ESP_LOGD(TAG, "  min:     %"PRIi64, qctrl.minimum);
        ESP_LOGD(TAG, "  max:     %"PRIi64, qctrl.maximum);
        ESP_LOGD(TAG, "  step:    %"PRIu64, qctrl.step);
        ESP_LOGD(TAG, "  current: %"PRIi32, control[0].value);
    } else {
        ESP_LOGD(TAG, "V4L2_CID_FOCUS_ABSOLUTE is not supported");
    }

    ret = ioctl(fd, VIDIOC_G_MOTOR_FMT, &motor_format);
    if (ret == 0) {
        isp->focus_info.period_in_us = motor_format.step_period.period_in_us;
        isp->focus_info.codes_per_step = motor_format.step_period.codes_per_step;
    } else {
        ESP_LOGE(TAG, "VIDIOC_G_MOTOR_FMT is not supported");
    }
#elif CONFIG_ESP_IPA_AF_ALGORITHM
    isp->sensor.focus_info = &isp->focus_info;

    isp->focus_info.min_pos = 0;
    isp->focus_info.max_pos = 1;
    isp->focus_info.step_pos = 1;
    isp->focus_info.cur_pos = 0;
#endif

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_G_FMT, &format);
    if (ret == 0) {
        isp->sensor.width = format.fmt.pix.width;
        isp->sensor.height = format.fmt.pix.height;
    }

    isp->cam_fd = fd;

    return ESP_OK;

fail_0:
    close(fd);
    return ret;
}

static esp_err_t init_isp_dev(const esp_video_isp_config_t *config, esp_video_isp_t *isp)
{
    int fd;
    esp_err_t ret;
    struct v4l2_requestbuffers req;
    int type = V4L2_BUF_TYPE_META_CAPTURE;

    fd = open(config->isp_dev, O_RDWR);
    ESP_RETURN_ON_FALSE(fd > 0, ESP_ERR_INVALID_ARG, TAG, "failed to open %s", config->isp_dev);
    print_dev_info(fd);

    memset(&req, 0, sizeof(req));
    req.count  = ISP_METADATA_BUFFER_COUNT;
    req.type   = type;
    req.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(fd, VIDIOC_REQBUFS, &req);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to require buffer");

    for (int i = 0; i < ISP_METADATA_BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type        = type;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ret = ioctl(fd, VIDIOC_QUERYBUF, &buf);
        ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to query buffer");

        isp->isp_stats[i] = (esp_video_isp_stats_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, buf.m.offset);
        ESP_GOTO_ON_FALSE(isp->isp_stats[i] != NULL, ESP_FAIL, fail_0, TAG, "failed to map buffer");

        ret = ioctl(fd, VIDIOC_QBUF, &buf);
        ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to queue buffer");
    }

    ret = ioctl(fd, VIDIOC_STREAMON, &type);
    ESP_GOTO_ON_FALSE(ret == 0, ESP_FAIL, fail_0, TAG, "failed to start stream");

    isp->isp_fd = fd;

    return ESP_OK;

fail_0:
    close(fd);
    return ret;
}

/**
 * @brief Initialize and start ISP system module.
 *
 * @param config ISP configuration
 *
 * @return
 *      - ESP_OK on success
 *      - Others if failed
 */
esp_err_t esp_video_isp_pipeline_init(const esp_video_isp_config_t *config)
{
    esp_err_t ret;
    esp_video_isp_t *isp;
    esp_ipa_metadata_t metadata;

#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif

    if (!config || !config->isp_dev || !config->cam_dev ||
            !config->ipa_config) {
        ESP_LOGE(TAG, "failed to check ISP configuration");
        return ESP_ERR_INVALID_ARG;
    }

    isp = calloc(1, sizeof(esp_video_isp_t));
    ESP_RETURN_ON_FALSE(isp, ESP_ERR_NO_MEM, TAG, "failed to malloc isp");

    ESP_GOTO_ON_ERROR(esp_ipa_pipeline_create(config->ipa_config, &isp->ipa_pipeline),
                      fail_0, TAG, "failed to create IPA pipeline");

    ESP_GOTO_ON_ERROR(init_cam_dev(config, isp), fail_1, TAG, "failed to initialize camera device");
    ESP_GOTO_ON_ERROR(init_isp_dev(config, isp), fail_2, TAG, "failed to initialize ISP device");

    metadata.flags = 0;
    ESP_GOTO_ON_ERROR(esp_ipa_pipeline_init(isp->ipa_pipeline, &isp->sensor, &metadata),
                      fail_3, TAG, "failed to initialize IPA pipeline");
    config_isp_and_camera(isp, &metadata);
    
    // Create frame boundary semaphore for synchronization with camera pipeline
    isp->frame_boundary_sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(isp->frame_boundary_sem != NULL, ESP_ERR_NO_MEM,
                      fail_3, TAG, "failed to create frame boundary semaphore");

    // Create camera update queue and async task for non-blocking I2C updates
    // Queue depth=2: allows 1 pending + 1 active update without blocking ISP task
    isp->camera_update_queue = xQueueCreate(2, sizeof(camera_update_request_t));
    ESP_GOTO_ON_FALSE(isp->camera_update_queue != NULL, ESP_ERR_NO_MEM,
                      fail_3, TAG, "failed to create camera update queue");
    ESP_LOGI(TAG, "Camera update queue created successfully (depth=2)");
    
    // Camera update task runs at much lower priority to avoid blocking pipeline
    // Priority 5 ensures it only runs when camera/encoder tasks are idle
    // Pin to CORE 1 to avoid contention with camera/encoder tasks (typically on CORE 0)
    ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(camera_update_task, "cam_update", 3072, isp, 
                                               5, &isp->camera_update_task, 1) == pdPASS,
                      ESP_ERR_NO_MEM, fail_3, TAG, "failed to create camera update task");
    ESP_LOGI(TAG, "Camera update task created successfully (priority=5, core=1)");

    /**
     * If CONFIG_ISP_PIPELINE_CONTROLLER_TASK_STACK_USE_PSRAM is enabled, the ISP controller task stack
     * will be allocated in PSRAM instead of DRAM. This reduces DRAM usage but may introduce slight
     * performance overhead due to slower PSRAM access.
     */
#if CONFIG_ISP_PIPELINE_CONTROLLER_TASK_STACK_USE_PSRAM
    StaticTask_t *task_ptr = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL);
    ESP_GOTO_ON_FALSE(task_ptr, ESP_ERR_NO_MEM, fail_3, TAG, "failed to malloc task");

    StackType_t *task_stack_ptr = heap_caps_malloc(ISP_TASK_STACK_SIZE * sizeof(StackType_t), MALLOC_CAP_SPIRAM);
    ESP_GOTO_ON_FALSE(task_stack_ptr, ESP_ERR_NO_MEM, fail_4, TAG, "failed to malloc task stack");

    // Pin ISP task to CORE 1 to avoid interference with video pipeline on CORE 0
    isp->task_handler = xTaskCreateStaticPinnedToCore(isp_task, ISP_TASK_NAME, ISP_TASK_STACK_SIZE,
                                          isp, ISP_TASK_PRIORITY, task_stack_ptr, task_ptr, 1);
    ESP_GOTO_ON_FALSE(isp->task_handler != NULL, ESP_ERR_NO_MEM,
                      fail_5, TAG, "failed to create ISP static task");

    isp->task_ptr = task_ptr;
    isp->task_stack_ptr = task_stack_ptr;
    ESP_LOGI(TAG, "ISP task created (priority=%d, core=1, stack=PSRAM)", ISP_TASK_PRIORITY);
#else
    // Pin ISP task to CORE 1 to avoid interference with video pipeline on CORE 0
    ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(isp_task, ISP_TASK_NAME, ISP_TASK_STACK_SIZE, isp, 
                                               ISP_TASK_PRIORITY, &isp->task_handler, 1) == pdPASS,
                      ESP_ERR_NO_MEM, fail_3, TAG, "failed to create ISP task");
    ESP_LOGI(TAG, "ISP task created (priority=%d, core=1, stack=DRAM)", ISP_TASK_PRIORITY);
#endif

    s_esp_video_isp = isp;
    
    // Register callback with camera pipeline for frame boundary notifications
    camera_pipeline_register_frame_callback(isp_frame_boundary_notify);
    
    ESP_LOGI(TAG, "ISP Pipeline initialized");
    
    return ESP_OK;

#if CONFIG_ISP_PIPELINE_CONTROLLER_TASK_STACK_USE_PSRAM
fail_5:
    heap_caps_free(task_stack_ptr);
fail_4:
    heap_caps_free(task_ptr);
#endif
fail_3:
    close(isp->isp_fd);
fail_2:
    close(isp->cam_fd);
fail_1:
    esp_ipa_pipeline_destroy(isp->ipa_pipeline);
fail_0:
    free(isp);
    return ret;
}

/**
 * @brief Deinitialize ISP system module.
 *
 * @param None
 *
 * @return
 *      - ESP_OK on success
 *      - Others if failed
 */
esp_err_t esp_video_isp_pipeline_deinit(void)
{
    int ret;
    esp_video_isp_t *isp = s_esp_video_isp;
    int type = V4L2_BUF_TYPE_META_CAPTURE;

    ESP_RETURN_ON_FALSE(s_esp_video_isp, ESP_FAIL, TAG, "ISP controller is not initialized");

    ret = ioctl(isp->isp_fd, VIDIOC_STREAMOFF, &type);
    ESP_RETURN_ON_FALSE(ret == 0, ESP_FAIL, TAG, "failed to stop stream");
    vTaskDelay(ISP_METADATA_BUFFER_COUNT * 50 / portTICK_PERIOD_MS);

    vTaskDelete(isp->task_handler);
    vTaskDelay(1);
#if CONFIG_ISP_PIPELINE_CONTROLLER_TASK_STACK_USE_PSRAM
    heap_caps_free(isp->task_ptr);
    heap_caps_free(isp->task_stack_ptr);
#endif

    // Cleanup frame boundary semaphore
    if (isp->frame_boundary_sem) {
        vSemaphoreDelete(isp->frame_boundary_sem);
        isp->frame_boundary_sem = NULL;
    }
    
    // Unregister callback
    camera_pipeline_register_frame_callback(NULL);

    ESP_RETURN_ON_FALSE(close(isp->isp_fd) == 0, ESP_FAIL, TAG, "failed to close ISP");
    ESP_RETURN_ON_FALSE(close(isp->cam_fd) == 0, ESP_FAIL, TAG, "failed to close camera sensor");
    ESP_RETURN_ON_ERROR(esp_ipa_pipeline_destroy(isp->ipa_pipeline), TAG, "failed to destroy pipeline");
    free(isp);
    s_esp_video_isp = NULL;

    return ESP_OK;
}

/**
 * @brief Check if ISP pipeline is initialized.
 *
 * @return
 *      - true if ISP pipeline is initialized
 *      - false if ISP pipeline is not initialized
 */
bool esp_video_isp_pipeline_is_initialized(void)
{
    return s_esp_video_isp != NULL;
}
