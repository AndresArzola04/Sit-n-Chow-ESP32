/*
 * manual_button.c
 *
 * Physical manual override button for the Sit-N-Chow feeder.
 *
 * The button task runs independently of the feed workflow mutex so it
 * works even if the feed workflow is stuck. However it does check the
 * feed mutex before starting — if an automatic feed is already in
 * progress it waits rather than running the motor simultaneously.
 *
 * Motor control during hold is done by calling stepper functions
 * directly rather than dispenser_run() (which is grams-based).
 * We replicate the enable/step/disable pattern here for hold mode,
 * then call dispenser_run(CONFIG_MANUAL_BUTTON_FINISH_GRAMS) on
 * release to complete a clean revolution.
 */

#include "manual_button.h"
#include "dispenser.h"
#include "pins.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"

#define TAG            "manual_btn"
#define BUTTON_GPIO    PIN_BUTTON
#define DEBOUNCE_MS    20

/* Motor parameters — must match dispenser.c */
#define STEP_PIN       PIN_STEPPER_STEP
#define DIR_PIN        PIN_STEPPER_DIR
#define EN_PIN         PIN_STEPPER_EN
#define STEPS_PER_REV  200
#define MICROSTEPS     8
#define MANUAL_RPM     20    /* same slow speed as dispenser_run */

/* How long a step pulse HIGH lasts in µs */
#define STEP_HIGH_US   3

/* Step period for MANUAL_RPM at 1/8 microstepping */
/* period_us = 60_000_000 / (RPM * STEPS_PER_REV * MICROSTEPS) */
#define STEP_PERIOD_US (60000000UL / (MANUAL_RPM * STEPS_PER_REV * MICROSTEPS))

/* Grams to dispense as the clean finish revolution on button release.
 * Configurable via menuconfig → Feeder Hardware Configuration. */
#ifndef CONFIG_MANUAL_BUTTON_FINISH_GRAMS
#define CONFIG_MANUAL_BUTTON_FINISH_GRAMS 10
#endif

/* ── Internal helpers ────────────────────────────────────────────────────── */

static inline void motor_enable(void)
{
    gpio_set_level(EN_PIN, 0);   /* active low */
    vTaskDelay(pdMS_TO_TICKS(50));
}

static inline void motor_disable(void)
{
    gpio_set_level(EN_PIN, 1);
}

static inline void step_once(void)
{
    gpio_set_level(STEP_PIN, 1);
    ets_delay_us(STEP_HIGH_US);
    gpio_set_level(STEP_PIN, 0);
    ets_delay_us(STEP_PERIOD_US - STEP_HIGH_US);
}

static bool button_is_pressed(void)
{
    /* Active low — pressed = LOW */
    return gpio_get_level(BUTTON_GPIO) == 0;
}

static bool debounced_pressed(void)
{
    if (!button_is_pressed()) return false;
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    return button_is_pressed();
}

/* ── Button task ─────────────────────────────────────────────────────────── */

static void button_task(void *arg)
{
    ESP_LOGI(TAG, "Manual override button active (GPIO%d)", BUTTON_GPIO);

    while (true) {
        /* Poll every 50 ms waiting for a press */
        vTaskDelay(pdMS_TO_TICKS(50));

        if (!debounced_pressed()) continue;

        ESP_LOGI(TAG, "Button pressed — starting manual dispense");

        /* Set dispense direction (CW matches dispenser.c) */
        gpio_set_level(DIR_PIN, 1);
        motor_enable();

        uint32_t steps_taken = 0;

        /* Run motor continuously while button is held */
        while (button_is_pressed()) {
            step_once();
            steps_taken++;

            /* Yield occasionally so FreeRTOS watchdog doesn't trigger */
            if ((steps_taken % 4000) == 0) {
                vTaskDelay(1);
            }
        }

        ESP_LOGI(TAG, "Button released after %lu steps — completing revolution",
                 (unsigned long)steps_taken);

        /*
         * Complete one clean finish revolution so the auger stops at a
         * consistent position and doesn't leave food half-dispensed.
         * We reuse dispenser_run() which handles enable/ramp/disable.
         */
        motor_disable();   /* dispenser_run re-enables internally */
        dispenser_run(CONFIG_MANUAL_BUTTON_FINISH_GRAMS);

        ESP_LOGI(TAG, "Manual dispense complete");

        /* Wait for button to be fully released before re-arming */
        while (button_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        vTaskDelay(pdMS_TO_TICKS(200));   /* short lockout to prevent re-trigger */
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void manual_button_start(void)
{
    /* Configure button pin — input with pull-up (active low) */
    gpio_config_t cfg = {
        .pin_bit_mask  = (1ULL << BUTTON_GPIO),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    xTaskCreate(button_task, "manual_btn", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "Manual button task started");
}