/**
 * @file esp_isp_stats.c
 * @brief ISP HAL Statistics Implementation
 * 
 * Direct hardware register access for ISP statistics.
 * This bypasses esp_driver_isp API limitations by reading registers directly.
 */

#include "esp_isp_stats.h"
#include "esp_log.h"
#include "esp_check.h"
#include "hal/isp_ll.h"
#include "hal/isp_types.h"
#include "soc/soc_caps.h"
#include <string.h>
#include <limits.h>

// IPA types (we use void* in header to avoid exposing IPA dependency)
#include "esp_ipa_types.h"

static const char *TAG = "esp_isp_hal_stats";

// Hardware ISP pointer (accesses existing ISP, doesn't create new one)
static isp_dev_t *s_isp_hw = NULL;
static bool s_initialized = false;

esp_err_t esp_isp_hal_stats_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Get pointer to ISP hardware (doesn't create, just gets existing)
    s_isp_hw = ISP_LL_GET_HW(0);  // ESP32-P4 has only ISP index 0
    
    if (s_isp_hw == NULL) {
        ESP_LOGE(TAG, "Failed to get ISP hardware pointer");
        return ESP_ERR_NOT_FOUND;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "ISP HAL stats initialized (direct register access)");
    ESP_LOGI(TAG, "Reading from existing ISP hardware @ %p", s_isp_hw);
    
    return ESP_OK;
}

esp_err_t esp_isp_hal_stats_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    s_isp_hw = NULL;
    s_initialized = false;
    
    ESP_LOGI(TAG, "ISP HAL stats deinitialized");
    return ESP_OK;
}

esp_err_t esp_isp_hal_get_ae_stats(esp_isp_hal_ae_stats_t *stats)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    ESP_RETURN_ON_FALSE(stats != NULL, ESP_ERR_INVALID_ARG, TAG, "Stats pointer is NULL");
    ESP_RETURN_ON_FALSE(s_isp_hw != NULL, ESP_ERR_INVALID_STATE, TAG, "ISP hardware pointer is NULL");

    // Read luminance values from ISP hardware registers
    // Each block represents a region in the 5x5 AE grid
    int block_id = 0;
    uint32_t sum = 0;
    int min_val = INT32_MAX;
    int max_val = 0;
    
    for (int i = 0; i < SOC_ISP_AE_BLOCK_X_NUMS; i++) {
        for (int j = 0; j < SOC_ISP_AE_BLOCK_Y_NUMS; j++) {
            // Direct HAL register read - same as esp_driver_isp uses internally
            int lum = isp_ll_ae_get_block_mean_lum(s_isp_hw, block_id);
            stats->luminance[i][j] = lum;
            
            sum += lum;
            if (lum < min_val) min_val = lum;
            if (lum > max_val) max_val = lum;
            
            block_id++;
        }
    }
    
    stats->avg_luminance = sum / (SOC_ISP_AE_BLOCK_X_NUMS * SOC_ISP_AE_BLOCK_Y_NUMS);
    stats->min_luminance = min_val;
    stats->max_luminance = max_val;
    
    return ESP_OK;
}

esp_err_t esp_isp_hal_get_awb_stats(esp_isp_hal_awb_stats_t *stats)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "Not initialized");
    ESP_RETURN_ON_FALSE(stats != NULL, ESP_ERR_INVALID_ARG, TAG, "Stats pointer is NULL");
    ESP_RETURN_ON_FALSE(s_isp_hw != NULL, ESP_ERR_INVALID_STATE, TAG, "ISP hardware pointer is NULL");

    // Read AWB statistics from ISP hardware registers
    stats->white_patch_num = isp_ll_awb_get_white_patch_cnt(s_isp_hw);
    
    if (stats->white_patch_num > 0) {
        // Read accumulated R/G/B values from white patches
        stats->sum_r = isp_ll_awb_get_accumulated_r_value(s_isp_hw);
        stats->sum_g = isp_ll_awb_get_accumulated_g_value(s_isp_hw);
        stats->sum_b = isp_ll_awb_get_accumulated_b_value(s_isp_hw);
        
        // Calculate averages
        stats->avg_r = stats->sum_r / stats->white_patch_num;
        stats->avg_g = stats->sum_g / stats->white_patch_num;
        stats->avg_b = stats->sum_b / stats->white_patch_num;
    } else {
        stats->sum_r = 0;
        stats->sum_g = 0;
        stats->sum_b = 0;
        stats->avg_r = 0;
        stats->avg_g = 0;
        stats->avg_b = 0;
    }
    
    return ESP_OK;
}

void esp_isp_hal_to_ipa_stats(const esp_isp_hal_ae_stats_t *ae_stats,
                               const esp_isp_hal_awb_stats_t *awb_stats,
                               void *ipa_stats_ptr)
{
    if (!ae_stats || !awb_stats || !ipa_stats_ptr) {
        return;
    }

    esp_ipa_stats_t *ipa_stats = (esp_ipa_stats_t *)ipa_stats_ptr;
    
    // Clear flags and increment sequence
    ipa_stats->flags = 0;
    ipa_stats->seq++;
    
    // Convert AE stats: HAL uses 5x5 grid, IPA expects flat array
    for (int i = 0; i < SOC_ISP_AE_BLOCK_X_NUMS; i++) {
        for (int j = 0; j < SOC_ISP_AE_BLOCK_Y_NUMS; j++) {
            int idx = i * SOC_ISP_AE_BLOCK_Y_NUMS + j;
            ipa_stats->ae_stats[idx].luminance = ae_stats->luminance[i][j];
        }
    }
    ipa_stats->flags |= IPA_STATS_FLAGS_AE;
    
    // Convert AWB stats: HAL provides sums and count, IPA expects same format
    ipa_stats->awb_stats[0].counted = awb_stats->white_patch_num;
    ipa_stats->awb_stats[0].sum_r = awb_stats->sum_r;
    ipa_stats->awb_stats[0].sum_g = awb_stats->sum_g;
    ipa_stats->awb_stats[0].sum_b = awb_stats->sum_b;
    ipa_stats->flags |= IPA_STATS_FLAGS_AWB;
    
    // Note: HIST, SHARPEN, AF stats not available via HAL reads
    // IPA will work with just AE+AWB (which are the critical ones)
}
