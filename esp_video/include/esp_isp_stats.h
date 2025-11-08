/**
 * @file esp_isp_stats.h
 * @brief ISP Hardware Statistics via Direct HAL Access
 * 
 * This module provides direct access to ISP statistics by reading hardware
 * registers via HAL (Hardware Abstraction Layer) functions. Unlike the
 * esp_driver_isp API which requires creating a new processor (impossible when
 * one already exists), this approach reads from the EXISTING ISP hardware
 * that's already being used by the CSI video driver.
 * 
 * Key advantages:
 * - Zero CSI contention (no /dev/video1 needed)
 * - Direct MMIO register reads (~1-5µs)
 * - Uses existing ISP hardware (no resource conflicts)
 * - Hardware-quality stats (same as /dev/video1)
 * 
 * Architecture:
 *   Camera → CSI → ISP Hardware (managed by V4L2 driver)
 *                      ↓
 *              This module reads stats registers directly
 *                      ↓
 *              AE/AWB stats available for IPA algorithms
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Auto Exposure (AE) statistics from ISP hardware
 */
typedef struct {
    uint32_t avg_luminance;     ///< Average luminance across all blocks (0-1023)
    uint32_t min_luminance;     ///< Minimum luminance block value
    uint32_t max_luminance;     ///< Maximum luminance block value
    int luminance[5][5];        ///< 5x5 grid of luminance values (matches SOC_ISP_AE_BLOCK)
} esp_isp_hal_ae_stats_t;

/**
 * @brief Auto White Balance (AWB) statistics from ISP hardware
 */
typedef struct {
    uint32_t white_patch_num;   ///< Number of white patches detected
    uint32_t sum_r;             ///< Sum of R channel in white patches
    uint32_t sum_g;             ///< Sum of G channel in white patches
    uint32_t sum_b;             ///< Sum of B channel in white patches
    uint32_t avg_r;             ///< Average R (calculated if white_patch_num > 0)
    uint32_t avg_g;             ///< Average G
    uint32_t avg_b;             ///< Average B
} esp_isp_hal_awb_stats_t;

/**
 * @brief Initialize ISP HAL stats access
 * 
 * This function prepares for direct HAL register access. It doesn't create
 * a new ISP processor - it simply sets up pointers to the existing hardware.
 * 
 * MUST be called AFTER video stream starts (when ISP hardware is active).
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_isp_hal_stats_init(void);

/**
 * @brief Deinitialize ISP HAL stats access
 * 
 * Cleanup function (currently minimal since we don't own the hardware).
 * 
 * @return ESP_OK on success
 */
esp_err_t esp_isp_hal_stats_deinit(void);

/**
 * @brief Get Auto Exposure statistics via direct HAL register reads
 * 
 * Reads luminance statistics from ISP hardware registers. This is a fast
 * MMIO operation (~1-5µs) with zero impact on CSI bandwidth.
 * 
 * @param[out] stats Pointer to structure to receive AE statistics
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_STATE if not initialized
 *         ESP_ERR_INVALID_ARG if stats is NULL
 */
esp_err_t esp_isp_hal_get_ae_stats(esp_isp_hal_ae_stats_t *stats);

/**
 * @brief Get Auto White Balance statistics via direct HAL register reads
 * 
 * Reads AWB statistics from ISP hardware registers.
 * 
 * @param[out] stats Pointer to structure to receive AWB statistics
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_STATE if not initialized
 *         ESP_ERR_INVALID_ARG if stats is NULL
 */
esp_err_t esp_isp_hal_get_awb_stats(esp_isp_hal_awb_stats_t *stats);

/**
 * @brief Convert HAL statistics to IPA format
 * 
 * Maps our HAL-based AE/AWB stats into the esp_ipa_stats_t format
 * expected by the IPA pipeline algorithms.
 * 
 * @param[in] ae_stats HAL AE statistics
 * @param[in] awb_stats HAL AWB statistics
 * @param[out] ipa_stats IPA statistics structure to populate
 */
void esp_isp_hal_to_ipa_stats(const esp_isp_hal_ae_stats_t *ae_stats,
                               const esp_isp_hal_awb_stats_t *awb_stats,
                               void *ipa_stats);

#ifdef __cplusplus
}
#endif
