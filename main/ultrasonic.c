/*
 * ultrasonic.c
 *
 * HC-SR04 ultrasonic distance sensor driver.
 * Ported from standalone main.c test into a reusable module.
 *
 * Pins:
 *   TRIG: GPIO_NUM_45 (output)
 *   ECHO: GPIO_NUM_35 (input, pull-down)
 */

#include "ultrasonic.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define TAG              "ultrasonic"
#define TRIG_PIN         GPIO_NUM_45
#define ECHO_PIN         GPIO_NUM_35

/* Speed of sound at ~20°C in cm/µs */
#define SOUND_SPEED_CM_PER_US  0.034f

/* Timeout waiting for echo high/low — 30 ms covers ~500 cm max range */
#define ECHO_TIMEOUT_US  30000

void ultrasonic_init(void)
{
    /* TRIG — output, start low */
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG_PIN, 0);

    /* ECHO — input with pull-down to prevent floating */
    gpio_config_t echo_cfg = {
        .pin_bit_mask  = (1ULL << ECHO_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_down_en  = GPIO_PULLDOWN_ENABLE,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_cfg);

    ESP_LOGI(TAG, "Ultrasonic sensor initialised (TRIG=%d ECHO=%d)",
             TRIG_PIN, ECHO_PIN);
}

float ultrasonic_read_cm(void)
{
    /* Send 10 µs trigger pulse */
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    /* Wait for echo to go HIGH */
    int64_t start_timeout = esp_timer_get_time() + ECHO_TIMEOUT_US;
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() > start_timeout) {
            ESP_LOGW(TAG, "No echo received (timeout waiting for HIGH)");
            return -1.0f;
        }
    }

    int64_t start = esp_timer_get_time();

    /* Wait for echo to go LOW */
    int64_t echo_timeout = start + ECHO_TIMEOUT_US;
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() > echo_timeout) {
            ESP_LOGW(TAG, "Echo pulse timeout (waiting for LOW)");
            return -1.0f;
        }
    }

    int64_t end = esp_timer_get_time();

    float duration_us = (float)(end - start);
    float distance_cm = (duration_us * SOUND_SPEED_CM_PER_US) / 2.0f;

    ESP_LOGD(TAG, "Duration: %.1f µs → Distance: %.2f cm", duration_us, distance_cm);
    return distance_cm;
}
