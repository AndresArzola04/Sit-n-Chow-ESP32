#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

#define TMF8701_CALIB_DATA_SIZE 14

typedef enum {
    TMF8701_CALIB_MODE_NO_CALIB          = 0,
    TMF8701_CALIB_MODE_CALIB             = 1,
    TMF8701_CALIB_MODE_CALIB_AND_ALGO    = 3
} tmf8701_calib_mode_t;

typedef enum {
    TMF8701_DISTANCE_MODE_PROXIMITY = 0,
    TMF8701_DISTANCE_MODE_DISTANCE  = 1,
    TMF8701_DISTANCE_MODE_COMBINE   = 2
} tmf8701_distance_mode_t;

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle type (C code only sees a pointer to this)
typedef struct DFRobot_TMF8701_C DFRobot_TMF8701_C;

/**
 * Create a TMF8701 instance.
 * enPin/intPin can be GPIO_NUM_NC if unused.
 */
DFRobot_TMF8701_C *tmf8701_create(gpio_num_t enPin,
                                  gpio_num_t intPin,
                                  i2c_port_t i2cPort,
                                  uint8_t i2cAddr);

/** Destroy (free) the instance */
void tmf8701_destroy(DFRobot_TMF8701_C *handle);

/** Initialize the sensor. Returns 0 on success, -1 on failure. */
int tmf8701_begin(DFRobot_TMF8701_C *handle);

/** Start measurement */
bool tmf8701_start_measurement(DFRobot_TMF8701_C *handle);

/** Check if new data is ready */
bool tmf8701_is_data_ready(DFRobot_TMF8701_C *handle);

/** Get the distance in mm (only call if data ready) */
uint16_t tmf8701_get_distance_mm(DFRobot_TMF8701_C *handle);

/** Get calibration data (14 bytes). len must be TMF8701_CALIB_DATA_SIZE. */
bool tmf8701_get_calibration_data(DFRobot_TMF8701_C *handle,
                                  uint8_t *data,
                                  uint8_t len);

/** Set calibration data (14 bytes). len must be TMF8701_CALIB_DATA_SIZE. */
bool tmf8701_set_calibration_data(DFRobot_TMF8701_C *handle,
                                  const uint8_t *data,
                                  uint8_t len);

/** Set calibration mode (no-calib / calib / calib+algo). */
bool tmf8701_set_calibration_mode(DFRobot_TMF8701_C *handle,
                                  tmf8701_calib_mode_t mode);

/**
 * Start measurement with explicit calibration & distance mode.
 * (More flexible than tmf8701_start_measurement, which uses CALIB+COMBINE.)
 */
bool tmf8701_start_measurement_ex(DFRobot_TMF8701_C *handle,
                                  tmf8701_calib_mode_t calib_mode,
                                  tmf8701_distance_mode_t dist_mode);


/** Get model string (e.g., "TMF8701") */
void tmf8701_get_model(DFRobot_TMF8701_C *handle,
                       char *out, uint32_t out_len);

/** Get SW version string */
void tmf8701_get_version(DFRobot_TMF8701_C *handle,
                         char *out, uint32_t out_len);

#ifdef __cplusplus
}
#endif
