/*
 * schedule_runner.h
 *
 * Fetches user schedules from Firebase and fires the feed workflow
 * at the correct local time.
 *
 * Call schedule_runner_start() once after WiFi + Firebase are ready.
 * It creates an internal FreeRTOS task that wakes every minute,
 * checks whether any schedule matches the current HH:mm, and if so
 * calls the registered feed_callback.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Prototype for the function called when a schedule fires.
 *
 * @param grams   Amount of food to dispense (from the schedule).
 * @param source  Human-readable trigger string, e.g. "schedule".
 */
typedef void (*feed_callback_t)(int grams, const char *source);

/**
 * @brief Start the schedule runner background task.
 *
 * @param device_id   Firebase device ID (used to build RTDB paths).
 * @param uid         Owner UID (used to read userSchedules/<uid>).
 * @param on_feed     Callback invoked when a schedule triggers.
 */
void schedule_runner_start(const char *device_id,
                           const char *uid,
                           feed_callback_t on_feed);

/**
 * @brief Force an immediate refresh of schedules from Firebase.
 *        Safe to call from any task.
 */
void schedule_runner_refresh(void);

#ifdef __cplusplus
}
#endif
