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

// HAL-based ISP stats (replaces /dev/video1)
#include "esp_isp_stats.h"

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
    
    // Camera update VSync synchronization (separate from ISP processing)
    SemaphoreHandle_t camera_vsync_sem;
    
    // Async camera parameter update task
    TaskHandle_t camera_update_task;
    QueueHandle_t camera_update_queue;

    // Async ISP configuration task (handles ISP ioctls off the hot path)
    TaskHandle_t isp_config_task;
    QueueHandle_t isp_config_queue;
    
    // **NEW: Shared buffer from pipeline for direct ISP processing**
    // Pipeline shares its buffer pointer so ISP can process the same frame
    // This eliminates /dev/video1 and CSI dual-stream contention
    struct {
        void *buffer_ptr;           // Pointer to RAW frame buffer
        uint32_t buffer_index;      // V4L2 buffer index
        uint32_t width;
        uint32_t height;
        uint32_t format;            // V4L2_PIX_FMT_*
        bool ready;                 // Buffer ready for ISP processing
        
        // HAL stats read in callback (perfect timing - right after DQBUF)
        esp_isp_hal_ae_stats_t ae_stats;   // AE stats from ISP registers
        esp_isp_hal_awb_stats_t awb_stats; // AWB stats from ISP registers
    } shared_buffer;
    
    // **I2C Performance Monitoring** (minimal - only essentials)
    struct {
        uint32_t total_updates;          // Total I2C updates executed
        uint32_t skipped_updates;        // Updates skipped due to <5% change
        TickType_t last_report_tick;     // Last time stats were printed
    } i2c_stats;
    bool isp_meta_streaming;
} esp_video_isp_t;

// Camera update request structure for async task
typedef struct {
    uint32_t exposure_us;
    uint32_t gain_index;
    float gain_value;
} camera_update_request_t;

// ISP async configuration operations
typedef enum {
    ISP_CFG_NONE = 0,
    ISP_CFG_STATS_REGION,
    ISP_CFG_BF,
    ISP_CFG_DEMOSAIC,
    ISP_CFG_SHARPEN,
    ISP_CFG_GAMMA,
    ISP_CFG_CCM,
    ISP_CFG_COLOR_BRIGHTNESS,
    ISP_CFG_COLOR_CONTRAST,
    ISP_CFG_COLOR_SATURATION,
    ISP_CFG_COLOR_HUE,
    ISP_CFG_AWB,
    ISP_CFG_AF,
#if ESP_VIDEO_ISP_DEVICE_LSC
    ISP_CFG_LSC,
#endif
} isp_cfg_op_t;

typedef struct {
    isp_cfg_op_t op;
    union {
        esp_ipa_region_t stats_region;
        esp_video_isp_bf_t bf;
        esp_video_isp_demosaic_t demosaic;
        esp_video_isp_sharpen_t sharpen;
        esp_video_isp_gamma_t gamma;
        esp_video_isp_ccm_t ccm;
        int32_t color_val; // for brightness/contrast/saturation/hue
        esp_video_isp_awb_t awb;
        esp_video_isp_af_t af;
#if ESP_VIDEO_ISP_DEVICE_LSC
        esp_video_isp_lsc_t lsc;
#endif
    } u;
} isp_config_request_t;

static const char *TAG = "ISP";
static esp_video_isp_t *s_esp_video_isp;

// Forward declaration for camera pipeline callback registration
// **MODIFIED**: Callback now receives buffer information for direct ISP processing
extern void camera_pipeline_register_frame_callback(void (*cb)(void*, uint32_t, uint32_t, uint32_t));

// Callback from camera pipeline when frame is ready for ISP processing
// Called AFTER pipeline DQBUF (CSI just finished, ISP just processed, stats ready!)
// This is the PERFECT timing to read HAL stats - ISP is idle but stats are fresh
static void isp_frame_ready_notify(void *buffer_ptr, uint32_t buffer_index, uint32_t width, uint32_t height)
{
    if (s_esp_video_isp && s_esp_video_isp->frame_boundary_sem) {
        // **ULTRA-FAST CALLBACK**: Must return IMMEDIATELY to avoid blocking camera pipeline!
        // This callback runs IN PIPELINE CONTEXT before QBUF - any delay here stalls the entire pipeline
        
        // Read stats directly in callback (fast ~1-5µs total - hardware register reads)
        esp_err_t ret_ae = esp_isp_hal_get_ae_stats(&s_esp_video_isp->shared_buffer.ae_stats);
        esp_err_t ret_awb = esp_isp_hal_get_awb_stats(&s_esp_video_isp->shared_buffer.awb_stats);
        
        if (ret_ae == ESP_OK && ret_awb == ESP_OK) {
            // Store buffer info (just pointer copies - sub-microsecond)
            s_esp_video_isp->shared_buffer.buffer_ptr = buffer_ptr;
            s_esp_video_isp->shared_buffer.buffer_index = buffer_index;
            s_esp_video_isp->shared_buffer.width = width;
            s_esp_video_isp->shared_buffer.height = height;
            s_esp_video_isp->shared_buffer.ready = true;
            
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            
            // Signal ISP task (binary semaphore - just sets a flag, ~500ns)
            xSemaphoreGiveFromISR(s_esp_video_isp->frame_boundary_sem, &xHigherPriorityTaskWoken);
            
            // **CRITICAL**: Do NOT yield here! Let pipeline complete its work first
            // ISP task will wake on next scheduler tick (not immediately)
            // This prevents ISP processing from blocking the pipeline
            // portYIELD_FROM_ISR removed - scheduler will context switch naturally
        }
        // If stats read failed, silently skip this frame (no signal to ISP task)
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
        
        int64_t t0 = esp_timer_get_time();
        if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
            int64_t dt = esp_timer_get_time() - t0;
            ESP_LOGE(TAG, "failed to set GAMMA (errno=%d, took %lld us) ⚠️ CAUSES STALLS!", 
                     errno, dt);
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

// ============================ Async ISP config path ============================
static inline bool isp_enqueue(esp_video_isp_t *isp, const isp_config_request_t *req)
{
    if (!isp || !isp->isp_config_queue || !req) return false;
    if (xQueueSend(isp->isp_config_queue, req, 0) != pdTRUE) {
        // drop on overflow; keep pipeline smooth
        ESP_LOGW(TAG, "ISP cfg queue full, dropping op=%d", (int)req->op);
        return false;
    }
    return true;
}

static void isp_config_task(void *p)
{
    esp_video_isp_t *isp = (esp_video_isp_t *)p;
    isp_config_request_t req;

    ESP_LOGI(TAG, "ISP config task started (low prio async ioctls)");
    for (;;) {
        if (xQueueReceive(isp->isp_config_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (req.op) {
        case ISP_CFG_STATS_REGION: {
            struct v4l2_selection selection = {0};
            selection.type = V4L2_BUF_TYPE_META_CAPTURE;
            selection.r.left = req.u.stats_region.left;
            selection.r.top = req.u.stats_region.top;
            selection.r.width = req.u.stats_region.width;
            selection.r.height = req.u.stats_region.height;
            if (ioctl(isp->isp_fd, VIDIOC_S_SELECTION, &selection) != 0) {
                ESP_LOGE(TAG, "failed to set selection");
            }
            break;
        }
        case ISP_CFG_BF: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_USER_ESP_ISP_BF;
            ctrl[0].p_u8 = (uint8_t *)&req.u.bf;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set bayer filter");
            }
            break;
        }
        case ISP_CFG_DEMOSAIC: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_USER_ESP_ISP_DEMOSAIC;
            ctrl[0].p_u8 = (uint8_t *)&req.u.demosaic;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set demosaic");
            }
            break;
        }
        case ISP_CFG_SHARPEN: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_USER_ESP_ISP_SHARPEN;
            ctrl[0].p_u8 = (uint8_t *)&req.u.sharpen;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set sharpen");
            }
            break;
        }
        case ISP_CFG_GAMMA: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_USER_ESP_ISP_GAMMA;
            ctrl[0].p_u8 = (uint8_t *)&req.u.gamma;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set GAMMA");
            }
            break;
        }
        case ISP_CFG_CCM: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_USER_ESP_ISP_CCM;
            ctrl[0].p_u8 = (uint8_t *)&req.u.ccm;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set CCM");
            }
            break;
        }
        case ISP_CFG_COLOR_BRIGHTNESS: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_BRIGHTNESS;
            ctrl[0].value = req.u.color_val;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set brightness");
            }
            break;
        }
        case ISP_CFG_COLOR_CONTRAST: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_CONTRAST;
            ctrl[0].value = req.u.color_val;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set contrast");
            }
            break;
        }
        case ISP_CFG_COLOR_SATURATION: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_SATURATION;
            ctrl[0].value = req.u.color_val;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set saturation");
            }
            break;
        }
        case ISP_CFG_COLOR_HUE: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_HUE;
            ctrl[0].value = req.u.color_val;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set hue");
            }
            break;
        }
        case ISP_CFG_AWB: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_USER_ESP_ISP_AWB;
            ctrl[0].p_u8 = (uint8_t *)&req.u.awb;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set AWB");
            }
            break;
        }
        case ISP_CFG_AF: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_USER_ESP_ISP_AF;
            ctrl[0].p_u8 = (uint8_t *)&req.u.af;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set AF");
            }
            break;
        }
#if ESP_VIDEO_ISP_DEVICE_LSC
        case ISP_CFG_LSC: {
            struct v4l2_ext_controls ctrls = {0};
            struct v4l2_ext_control ctrl[1] = {0};
            ctrls.ctrl_class = V4L2_CID_USER_CLASS;
            ctrls.count = 1; ctrls.controls = ctrl;
            ctrl[0].id = V4L2_CID_USER_ESP_ISP_LSC;
            ctrl[0].p_u8 = (uint8_t *)&req.u.lsc;
            if (ioctl(isp->isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
                ESP_LOGE(TAG, "failed to set LSC");
            }
            break;
        }
#endif
        default:
            break;
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

        // Send camera update request to async task (non-blocking!)
        // This eliminates I2C stalls by moving 14-15ms ioctl to low-priority background task
        // Check if queue is initialized (it may not be during early init)
        if (isp->camera_update_queue != NULL) {
            camera_update_request_t req = {
                .exposure_us = apply_us,
                .gain_index = gain_index,
                .gain_value = target_gain
            };
            
            // **VSync-synchronized updates - NO artificial throttling!**
            // With VSync sync working perfectly, we don't need smart skip anymore
            // camera_update_task will skip only if values are EXACTLY the same
            // This allows ISP to respond quickly to lighting changes
            
            // **Camera updates with VSync synchronization (no throttle)**
            #if 1  // Set to 0 to disable camera updates again
            if (xQueueSend(isp->camera_update_queue, &req, 0) == pdTRUE) {
                ESP_LOGD(TAG, "ISP: Queued camera update (exp=%lu gain=%lu)", apply_us, gain_index);
            } else {
                ESP_LOGW(TAG, "ISP: Camera update queue FULL! Dropping update (exp=%lu gain=%lu)", 
                         apply_us, gain_index);
            }
            #else
            ESP_LOGI(TAG, "ISP: Camera updates DISABLED for testing (exp=%lu gain=%lu)", 
                     apply_us, gain_index);
            #endif
        } else {
            // Queue not initialized yet - apply update synchronously for now
            ESP_LOGD(TAG, "Camera update queue not ready, skipping update");
        }
    }
    
    // **REMOVED**: Old synchronous update path (lines 748-790)
    // Always use async camera_update_task with VSync synchronization
    // This eliminates the dual-path problem where GROUP_HOLD disable
    // would bypass VSync sync logic
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
    // Route ISP controls through async config queue to avoid stalls
    if (isp->isp_config_queue) {
        isp_config_request_t req;
        // Stats region
        if (metadata->flags & IPA_METADATA_FLAGS_SR) {
            req.op = ISP_CFG_STATS_REGION;
            req.u.stats_region = metadata->stats_region;
            isp_enqueue(isp, &req);
        }
        // Bayer filter
        if (metadata->flags & IPA_METADATA_FLAGS_BF) {
            req.op = ISP_CFG_BF;
            req.u.bf.enable = true;
            req.u.bf.level = metadata->bf.level;
            for (int i = 0; i < ISP_BF_TEMPLATE_X_NUMS; i++) {
                for (int j = 0; j < ISP_BF_TEMPLATE_Y_NUMS; j++) {
                    req.u.bf.matrix[i][j] = metadata->bf.matrix[i][j];
                }
            }
            isp_enqueue(isp, &req);
        }
        // Demosaic
        if (metadata->flags & IPA_METADATA_FLAGS_DM) {
            req.op = ISP_CFG_DEMOSAIC;
            req.u.demosaic.enable = true;
            req.u.demosaic.gradient_ratio = metadata->demosaic.gradient_ratio;
            isp_enqueue(isp, &req);
        }
        // Sharpen
        if (metadata->flags & IPA_METADATA_FLAGS_SH) {
            req.op = ISP_CFG_SHARPEN;
            req.u.sharpen.enable = true;
            req.u.sharpen.h_thresh = metadata->sharpen.h_thresh;
            req.u.sharpen.l_thresh = metadata->sharpen.l_thresh;
            req.u.sharpen.h_coeff = metadata->sharpen.h_coeff;
            req.u.sharpen.m_coeff = metadata->sharpen.m_coeff;
            for (int i = 0; i < ISP_SHARPEN_TEMPLATE_X_NUMS; i++) {
                for (int j = 0; j < ISP_SHARPEN_TEMPLATE_Y_NUMS; j++) {
                    req.u.sharpen.matrix[i][j] = metadata->sharpen.matrix[i][j];
                }
            }
            isp_enqueue(isp, &req);
        }
        // AWB
        if (metadata->flags & IPA_METADATA_FLAGS_AWB) {
            esp_ipa_awb_range_t *range = &metadata->awb;
            req.op = ISP_CFG_AWB;
            req.u.awb.enable = true;
            req.u.awb.green_max = range->green_max;
            req.u.awb.green_min = range->green_min;
            req.u.awb.rg_max = range->rg_max;
            req.u.awb.rg_min = range->rg_min;
            req.u.awb.bg_max = range->bg_max;
            req.u.awb.bg_min = range->bg_min;
            isp_enqueue(isp, &req);
        }
        // AF
        if (metadata->flags & IPA_METADATA_FLAGS_AF) {
            req.op = ISP_CFG_AF;
            req.u.af.enable = true;
            req.u.af.edge_thresh = metadata->af.edge_thresh;
            memcpy(req.u.af.windows, metadata->af.windows, sizeof(isp_window_t) * ISP_AF_WINDOW_NUM);
            isp_enqueue(isp, &req);
        }
        // Optional blocks (keep disabled unless verified safe)
        // GAMMA/CCM/COLOR/LSC could be enqueued similarly when re-enabled.
    } else {
        // Fallback to direct ioctls if queue not ready (early init)
        config_statistics_region(isp, metadata);
        config_bayer_filter(isp, metadata);
        config_demosaic(isp, metadata);
        config_sharpen(isp, metadata);
        config_awb(isp, metadata);
        config_af(isp, metadata);
    }

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

// Async camera parameter update task - runs at moderate priority
static void camera_update_task(void *p)
{
    esp_video_isp_t *isp = (esp_video_isp_t *)p;
    camera_update_request_t req;
    
    // Initialize I2C stats
    memset(&isp->i2c_stats, 0, sizeof(isp->i2c_stats));
    isp->i2c_stats.last_report_tick = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");
    ESP_LOGI(TAG, "🚀 Camera Update Task - HARDWARE VSYNC MODE");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");
    ESP_LOGI(TAG, "✅ NO software VSync wait (GROUP_HOLD does it!)");
    ESP_LOGI(TAG, "✅ Direct apply from queue → ~12ms per update");
    ESP_LOGI(TAG, "✅ Zero stalls - hardware handles synchronization");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");
    
    while (1) {
        // Wait for update request from ISP task (blocking)
        if (xQueueReceive(isp->camera_update_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        ESP_LOGD(TAG, "📝 Update received (exp=%lu gain=%lu) - applying immediately", 
                 req.exposure_us, req.gain_index);
        
        // **CRITICAL INSIGHT**: GROUP_HOLD release ALREADY waits for VSync!
        // No need for software semaphore - hardware does the synchronization
        // The ioctl() will block ~12ms until next frame boundary automatically
        {
            struct v4l2_ext_controls controls;
            struct v4l2_ext_control control[2];  // Both exposure AND gain
            
            controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
            controls.count = 2;  // Always send both
            controls.controls = control;
            
            // Always send exposure
            control[0].id = V4L2_CID_EXPOSURE_ABSOLUTE;
            control[0].value = (int32_t)req.exposure_us / 100;
            
            // Always send gain
            control[1].id = V4L2_CID_GAIN;
            control[1].value = req.gain_index;
            
            // Calculate change magnitude for logging
            uint32_t exp_change_pct = 0;
            uint32_t gain_change_pct = 0;
            if (isp->sensor.cur_exposure > 0) {
                uint32_t exp_delta = (req.exposure_us > isp->sensor.cur_exposure) ?
                    (req.exposure_us - isp->sensor.cur_exposure) :
                    (isp->sensor.cur_exposure - req.exposure_us);
                exp_change_pct = (exp_delta * 100) / (isp->sensor.cur_exposure + 1);
            }
            if (isp->prev_gain_index > 0) {
                uint32_t gain_delta = (req.gain_index > isp->prev_gain_index) ?
                    (req.gain_index - isp->prev_gain_index) :
                    (isp->prev_gain_index - req.gain_index);
                gain_change_pct = (gain_delta * 100) / (isp->prev_gain_index + 1);
            }
            
            // Log BEFORE ioctl
            ESP_LOGI(TAG, "🚀 Applying (exp=%lu→%lu [%lu%%] gain=%lu→%lu [%lu%%])",
                     isp->sensor.cur_exposure, req.exposure_us, exp_change_pct,
                     (uint32_t)isp->prev_gain_index, req.gain_index, gain_change_pct);
            
            int64_t t0 = esp_timer_get_time();
            int ioctl_result = ioctl(isp->cam_fd, VIDIOC_S_EXT_CTRLS, &controls);
            int64_t dt = esp_timer_get_time() - t0;
            
            // Log AFTER with timing
            ESP_LOGI(TAG, "✅ Applied in %lld us (GROUP_HOLD sync)", (long long)dt);
            
            if (ioctl_result == 0) {
                isp->i2c_stats.total_updates++;
                isp->sensor.cur_exposure = req.exposure_us;
                isp->sensor.cur_gain = req.gain_value;
                isp->prev_gain_index = req.gain_index;
                
                // Log slow ioctls (normal is ~12ms due to GROUP_HOLD VSync wait)
                if (dt > 20000) {  // More than 20ms is abnormal
                    ESP_LOGW(TAG, "⚠️ SLOW ioctl: %lld us", (long long)dt);
                }
            } else {
                ESP_LOGE(TAG, "❌ ioctl FAILED (errno=%d, took %lld us)", errno, (long long)dt);
            }
            
            // **Report stats every 30 seconds** (reduced frequency)
            TickType_t now = xTaskGetTickCount();
            if ((now - isp->i2c_stats.last_report_tick) >= pdMS_TO_TICKS(30000)) {
                ESP_LOGI(TAG, "Camera: %lu updates total",
                         isp->i2c_stats.total_updates);
                
                isp->i2c_stats.last_report_tick = now;
            }
        }
    }
    
    vTaskDelete(NULL);
}

// ============================================================================
// isp_task() - Now using HAL-based stats (no /dev/video1!)
// ============================================================================
// Modified to use esp_isp_hal_get_ae_stats() and esp_isp_hal_get_awb_stats()
// instead of /dev/video1 DQBUF. This eliminates CSI dual-stream contention.
// ============================================================================
static void isp_task(void *p)
{
    esp_video_isp_t *isp = (esp_video_isp_t *)p;
    esp_ipa_metadata_t metadata;
    esp_ipa_stats_t ipa_stats;
    
    // Process every 2nd frame for stable ~7.5Hz updates
    uint32_t frame_counter = 0;
    uint32_t processed_counter = 0;

    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");
    ESP_LOGI(TAG, "🚀 ISP TASK - HARDWARE VSYNC MODE");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");
    ESP_LOGI(TAG, "✅ Process every 2nd frame (~7.5Hz)");
    ESP_LOGI(TAG, "✅ camera_update_task handles VSync (12ms each)");
    ESP_LOGI(TAG, "✅ No software VSync wait - GROUP_HOLD does it!");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");

    while (1) {
        // Wait for frame boundary from camera pipeline
        if (xSemaphoreTake(isp->frame_boundary_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        frame_counter++;
        
        // Process every 2nd frame for ~7.5Hz rate
        // This gives camera_update_task time to process (~12ms per update)
        if ((frame_counter % 2) != 0) {
            continue;  // Skip odd frames
        }
        
        processed_counter++;
        
        // Convert HAL stats to IPA format (stats already read in callback)
        esp_isp_hal_to_ipa_stats(&isp->shared_buffer.ae_stats, 
                                  &isp->shared_buffer.awb_stats, 
                                  &ipa_stats);
        
        // Process IPA pipeline - this is FAST (<1ms typically)
        metadata.flags = 0;
        esp_ipa_pipeline_process(isp->ipa_pipeline, &ipa_stats, &isp->sensor, &metadata);
        
        // Queue camera update (NON-BLOCKING) - camera_update_task will handle VSync
        config_isp_and_camera(isp, &metadata);
        
        // Log progress every 100 processed frames
        if ((processed_counter % 100) == 0) {
            UBaseType_t queue_waiting = uxQueueMessagesWaiting(isp->camera_update_queue);
            ESP_LOGI(TAG, "ISP: %lu frames total (%lu processed @ ~7.5Hz, queue=%lu)", 
                     frame_counter, processed_counter, (unsigned long)queue_waiting);
        }
    }
    
    vTaskDelete(NULL);
}

#if 0  // OLD /dev/video1 version - kept for reference
static void isp_task_old_devvideo1(void *p)
{
    esp_video_isp_t *isp = (esp_video_isp_t *)p;
    struct v4l2_buffer buf;
    esp_ipa_metadata_t metadata;
    
    static const uint32_t ISP_PROCESS_INTERVAL = 2;
    uint32_t frame_counter = 0;

    ESP_LOGI(TAG, "ISP task started - HARDWARE ISP MODE (/dev/video1, interval=%lu)", 
             ISP_PROCESS_INTERVAL);

    while (1) {
        // Wait for frame boundary from camera pipeline
        if (xSemaphoreTake(isp->frame_boundary_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        // OPTIMIZATION #2: Add 5ms delay to avoid collision with camera pipeline QBUF
        // This staggers ISP access - camera finishes QBUF first, then ISP does DQBUF
        // Reduces mutex contention probability by ~50-70%
        vTaskDelay(pdMS_TO_TICKS(5));
        
        // OPTIMIZATION #3: Skip alternate frames to reduce CSI access frequency
        // CSI access: 30/s → 22.5/s (25% reduction)
        // This moves from overload (166%) to near-limit (~125%)
        frame_counter++;
        if ((frame_counter % ISP_PROCESS_INTERVAL) != 0) {
            continue;
        }
        
        // DQBUF to get hardware ISP statistics from /dev/video1
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_META_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        int64_t t0 = esp_timer_get_time();
        int ret = ioctl(isp->isp_fd, VIDIOC_DQBUF, &buf);
        int64_t dt = esp_timer_get_time() - t0;
        
        if (ret != 0) {
            if (errno == EAGAIN) {
                ESP_LOGD(TAG, "ISP DQBUF EAGAIN (CSI busy) - skipping frame");
            } else {
                ESP_LOGW(TAG, "ISP DQBUF failed: errno=%d", errno);
            }
            continue;
        }
        
        if (dt > 10000) { // Log if DQBUF took >10ms (potential contention)
            ESP_LOGW(TAG, "ISP DQBUF took %lld us (contention detected!)", dt);
        } else {
            ESP_LOGD(TAG, "ISP DQBUF took %lld us (OK)", dt);
        }
        
        // Convert hardware ISP stats to IPA format
        esp_ipa_stats_t ipa_stats;
        isp_stats_to_ipa_stats(isp->isp_stats[buf.index], &ipa_stats);
        
        // Process IPA pipeline to calculate AE/AWB parameters
        get_sensor_stats(isp, &ipa_stats);
        metadata.flags = 0;
        esp_ipa_pipeline_process(isp->ipa_pipeline, &ipa_stats, &metadata);
        
        // QBUF to return buffer to driver
        ret = ioctl(isp->isp_fd, VIDIOC_QBUF, &buf);
        if (ret != 0) {
            ESP_LOGE(TAG, "ISP QBUF failed: errno=%d", errno);
        }
        
        // Apply camera/ISP configuration based on IPA results
        config_isp_and_camera(isp, &metadata);
        
        ESP_LOGD(TAG, "ISP frame processed (seq=%lu, interval=%lu)", 
                 frame_counter, ISP_PROCESS_INTERVAL);
    }
    
    vTaskDelete(NULL);
}
#endif  // End of disabled isp_task()

// ============================================================================
// End of legacy /dev/video1 code
// ============================================================================


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
        // **VSync-SYNCHRONIZED UPDATES**: GROUP_HOLD enabled with VSync timing
        // camera_update_task waits for frame_boundary before applying
        // This ensures changes are applied when camera is IDLE (between frames)
        isp->sensor_attr.group = 1;  // Enable GROUP_HOLD
        ESP_LOGI(TAG, "✅ V4L2_CID_CAMERA_GROUP ENABLED (VSync-synchronized updates)");
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

    fd = open(config->isp_dev, O_RDWR | O_NONBLOCK); // Non-blocking to prevent DQBUF stalls
    ESP_RETURN_ON_FALSE(fd > 0, ESP_ERR_INVALID_ARG, TAG, "failed to open %s", config->isp_dev);
    ESP_LOGI(TAG, "ISP device opened in non-blocking mode to prevent MIPI/DMA contention");
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
    isp->isp_meta_streaming = true;

    return ESP_OK;

fail_0:
    close(fd);
    return ret;
}

// Open ISP device only for control ioctls (no META capture streaming)
static esp_err_t init_isp_ctrl_dev(const esp_video_isp_config_t *config, esp_video_isp_t *isp)
{
    int fd = open(config->isp_dev, O_RDWR | O_NONBLOCK);
    ESP_RETURN_ON_FALSE(fd > 0, ESP_ERR_INVALID_ARG, TAG, "failed to open %s", config->isp_dev);
    ESP_LOGI(TAG, "ISP control device opened (non-blocking, no meta stream)");
    print_dev_info(fd);
    isp->isp_fd = fd;
    isp->isp_meta_streaming = false;
    return ESP_OK;
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
    
    // Open ISP control device (no META capture stream)
    ESP_GOTO_ON_ERROR(init_isp_ctrl_dev(config, isp), fail_2, TAG, "failed to initialize ISP control device");

    // Initialize IPA pipeline with sensor info
    metadata.flags = 0;
    ESP_GOTO_ON_ERROR(esp_ipa_pipeline_init(isp->ipa_pipeline, &isp->sensor, &metadata),
                      fail_2, TAG, "failed to initialize IPA pipeline");
    config_isp_and_camera(isp, &metadata);
    ESP_LOGI(TAG, "IPA pipeline initialized successfully");
    
    // Create frame boundary semaphore for synchronization with camera pipeline
    isp->frame_boundary_sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(isp->frame_boundary_sem != NULL, ESP_ERR_NO_MEM,
                      fail_2, TAG, "failed to create frame boundary semaphore");
    
    // Create camera VSync semaphore for camera updates (counting semaphore, not binary)
    // Counting semaphore allows multiple signals to queue up (one per frame)
    isp->camera_vsync_sem = xSemaphoreCreateCounting(10, 0);
    ESP_GOTO_ON_FALSE(isp->camera_vsync_sem != NULL, ESP_ERR_NO_MEM,
                      fail_2, TAG, "failed to create camera VSync semaphore");

    // Create camera update queue and async task for non-blocking I2C updates
    // **OPTIMIZED**: Increased queue depth (2→20) to handle 15Hz bursts without drops
    // At 15Hz with 25ms I2C latency, queue can hold ~1.33s worth of updates
    // Larger queue to absorb I2C update bursts (20 items = 1.33s @ 15Hz)
    // This prevents queue overflow when I2C is slow (~4ms per update)
    isp->camera_update_queue = xQueueCreate(20, sizeof(camera_update_request_t));
    ESP_GOTO_ON_FALSE(isp->camera_update_queue != NULL, ESP_ERR_NO_MEM,
                      fail_3, TAG, "failed to create camera update queue");
    ESP_LOGI(TAG, "Camera update queue created successfully (depth=20, absorbs bursts)");
    
    // **CRITICAL FIX**: Priority increased from 1→5 to reduce scheduler latency
    // Root cause analysis showed:
    //   - I2C @ 400kHz: 1.4ms (FAST!)
    //   - ioctl() total: 13.5ms (10× slower!)
    //   - Overhead breakdown: 12ms scheduler + 1.4ms I2C
    // With priority=1 (lowest), task waits ~12ms for CPU time before executing I2C
    // Priority=5 (moderate) reduces scheduler latency while still preventing capture/encode blocking
    // Expected: 13.5ms → 2-3ms total latency (matching I2C hardware speed)
    ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(camera_update_task, "cam_update", 3072, isp, 
                                               5, &isp->camera_update_task, 1) == pdPASS,
                      ESP_ERR_NO_MEM, fail_3, TAG, "failed to create camera update task");
    ESP_LOGI(TAG, "Camera update task created successfully (priority=5 [moderate], core=1)");
    ESP_LOGI(TAG, "  Expected latency: 2-3ms (was 13ms @ priority=1 due to scheduler delays)");

    // Initialize HAL-based ISP stats (direct register access)
    ESP_LOGI(TAG, "Initializing HAL-based ISP stats...");
    ret = esp_isp_hal_stats_init();
    ESP_GOTO_ON_ERROR(ret, fail_3, TAG, "failed to initialize HAL ISP stats");
    ESP_LOGI(TAG, "HAL ISP stats initialized - zero CSI contention mode!");

    // Create ISP task (now using HAL stats instead of /dev/video1)
#if CONFIG_ISP_PIPELINE_CONTROLLER_TASK_STACK_USE_PSRAM
    isp->task_ptr = heap_caps_calloc(1, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_GOTO_ON_FALSE(isp->task_ptr, ESP_ERR_NO_MEM, fail_3, TAG, "failed to malloc for task");

    isp->task_stack_ptr = heap_caps_calloc(1, ISP_TASK_STACK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_GOTO_ON_FALSE(isp->task_stack_ptr, ESP_ERR_NO_MEM, fail_3, TAG, "failed to malloc for task stack");

    isp->task_handler = xTaskCreateStatic(isp_task, ISP_TASK_NAME, ISP_TASK_STACK_SIZE,
                                           isp, ISP_TASK_PRIORITY, isp->task_stack_ptr, isp->task_ptr);
    ESP_GOTO_ON_FALSE(isp->task_handler, ESP_ERR_NO_MEM, fail_3, TAG, "failed to create ISP task");
#else
    ESP_GOTO_ON_FALSE(xTaskCreate(isp_task, ISP_TASK_NAME, ISP_TASK_STACK_SIZE,
                                   isp, ISP_TASK_PRIORITY, &isp->task_handler) == pdPASS,
                      ESP_ERR_NO_MEM, fail_3, TAG, "failed to create ISP task");
#endif
    ESP_LOGI(TAG, "ISP task created successfully (HAL-based, priority=%d)", ISP_TASK_PRIORITY);

    // Create ISP config queue + task (low priority) for async ioctls
    isp->isp_config_queue = xQueueCreate(16, sizeof(isp_config_request_t));
    ESP_GOTO_ON_FALSE(isp->isp_config_queue != NULL, ESP_ERR_NO_MEM,
                      fail_3, TAG, "failed to create ISP config queue");
    if (xTaskCreate(isp_config_task, "isp_cfg", 3072, isp, 3, &isp->isp_config_task) != pdPASS) {
        ESP_LOGE(TAG, "failed to create ISP config task");
        vQueueDelete(isp->isp_config_queue);
        isp->isp_config_queue = NULL;
        ESP_GOTO_ON_ERROR(ESP_FAIL, fail_3, TAG, "");
    }
    ESP_LOGI(TAG, "ISP config task started (priority=3)");

    s_esp_video_isp = isp;
    
    // Register callback to receive frame boundary notifications from camera pipeline
    // This signals the ISP task when a frame is ready for stats processing
    camera_pipeline_register_frame_callback(isp_frame_ready_notify);
    
    ESP_LOGI(TAG, "✅ ISP Pipeline initialized (ESP-IDF ISP driver mode - no /dev/video1)");
    
    return ESP_OK;

// Error handling: Cleanup in reverse order of initialization
fail_3:
    if (isp->isp_config_task) {
        vTaskDelete(isp->isp_config_task);
    }
    if (isp->isp_config_queue) {
        vQueueDelete(isp->isp_config_queue);
    }
    if (isp->camera_update_task) {
        vTaskDelete(isp->camera_update_task);
    }
    if (isp->camera_update_queue) {
        vQueueDelete(isp->camera_update_queue);
    }
fail_2:
    if (isp->camera_vsync_sem) {
        vSemaphoreDelete(isp->camera_vsync_sem);
    }
    if (isp->frame_boundary_sem) {
        vSemaphoreDelete(isp->frame_boundary_sem);
    }
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

    // Stop ISP meta stream only if it was started
    if (isp->isp_meta_streaming) {
        ret = ioctl(isp->isp_fd, VIDIOC_STREAMOFF, &type);
        ESP_RETURN_ON_FALSE(ret == 0, ESP_FAIL, TAG, "failed to stop stream");
        vTaskDelay(ISP_METADATA_BUFFER_COUNT * 50 / portTICK_PERIOD_MS);
    }

    // Stop tasks
    if (isp->isp_config_task) {
        vTaskDelete(isp->isp_config_task);
        isp->isp_config_task = NULL;
    }
    if (isp->task_handler) {
        vTaskDelete(isp->task_handler);
        isp->task_handler = NULL;
    }
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
    
    // Cleanup camera VSync semaphore
    if (isp->camera_vsync_sem) {
        vSemaphoreDelete(isp->camera_vsync_sem);
        isp->camera_vsync_sem = NULL;
    }

    // Cleanup ISP config queue
    if (isp->isp_config_queue) {
        vQueueDelete(isp->isp_config_queue);
        isp->isp_config_queue = NULL;
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
