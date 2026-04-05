/*
 * tof.h
 *
 * Time-of-Flight presence detection module using DFRobot TMF8701.
 *
 * Pins:
 *   SDA: GPIO_NUM_37
 *   SCL: GPIO_NUM_36
 *   EN:  not used (GPIO_NUM_NC)
 *   INT: not used (GPIO_NUM_NC)
 *
 * Calibration data (from your tested hardware):
 *   0x0F, 0x3F, 0x8A, 0x01, 0x00, 0x00, 0x00, 0x00,
 *   0x41, 0x57, 0x01, 0x00, 0x00, 0x00
 *
 * Call tof_init() once at boot.
 * Call tof_wait_for_presence() in the feed workflow — it blocks until
 * a reading below the threshold is detected or the timeout expires.
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise I2C bus and TMF8701 sensor.
 *        Must be called once before tof_wait_for_presence().
 *
 * @return ESP_OK on success, ESP_FAIL if sensor init fails.
 */
esp_err_t tof_init(void);

/**
 * @brief Block until a reading below threshold_mm is detected,
 *        or until timeout_ms milliseconds have elapsed.
 *
 * Takes CONFIG_TOF_SAMPLES_TO_CONFIRM consecutive readings below
 * threshold_mm before returning true, to avoid false triggers.
 *
 * @param threshold_mm  Distance in mm below which pet is considered present.
 * @param timeout_ms    Maximum time to wait in milliseconds.
 * @return true  if pet presence was confirmed within timeout.
 * @return false if timeout elapsed without detection.
 */
bool tof_wait_for_presence(uint16_t threshold_mm, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
