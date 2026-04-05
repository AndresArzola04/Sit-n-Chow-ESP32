/*
 * ultrasonic.h
 *
 * HC-SR04 ultrasonic distance sensor driver.
 *
 * TRIG_PIN: GPIO_NUM_45
 * ECHO_PIN: GPIO_NUM_35
 *
 * Call ultrasonic_init() once at boot, then ultrasonic_read_cm()
 * any time you need a distance reading.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise TRIG and ECHO GPIO pins.
 *        Must be called once before ultrasonic_read_cm().
 */
void ultrasonic_init(void);

/**
 * @brief Trigger a measurement and return the distance in centimetres.
 *
 * @return Distance in cm (positive value), or -1.0f on timeout/error.
 */
float ultrasonic_read_cm(void);

#ifdef __cplusplus
}
#endif
