/*
 * dispenser.h
 *
 * Stepper motor dispenser driver using DRV8825.
 *
 * Pins:
 *   STEP: GPIO_NUM_1
 *   DIR:  GPIO_NUM_39
 *   EN:   GPIO_NUM_48  (active low)
 *   SLP:  GPIO_NUM_2
 *   RST:  GPIO_NUM_42
 *   M0:   GPIO_NUM_14
 *   M1:   GPIO_NUM_38
 *   M2:   GPIO_NUM_41
 *
 * Call dispenser_init() once at boot, then dispenser_run(grams) to dispense.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise all stepper GPIO pins and put the driver in idle state.
 *        Must be called once before dispenser_run().
 */
void dispenser_init(void);

/**
 * @brief Dispense the requested amount of food.
 *
 * Converts grams to motor revolutions using CONFIG_DISPENSER_GRAMS_PER_REV,
 * then runs the motor at a slow controlled speed with a trapezoidal ramp.
 * Blocks until dispensing is complete.
 *
 * @param grams  Amount of food to dispense in grams (must be > 0).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if grams is 0.
 */
esp_err_t dispenser_run(int grams);

#ifdef __cplusplus
}
#endif
