/*
 * dispenser.c
 *
 * Stepper motor dispenser driver using DRV8825.
 * Ported from standalone hello_world_main.c test into a reusable module.
 *
 * Grams → revolutions conversion uses CONFIG_DISPENSER_GRAMS_PER_REV which
 * can be tuned in idf.py menuconfig → Feeder Hardware Configuration without
 * rewriting code.
 *
 * Speed is intentionally kept slow and steady (CONFIG_DISPENSER_RPM, default
 * 20 RPM) to avoid jamming and keep food flow controlled.
 */

#include "dispenser.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

/* ── Kconfig symbols ─────────────────────────────────────────────────────── *
 * CONFIG_DISPENSER_GRAMS_PER_REV  — grams dispensed per full revolution
 * CONFIG_DISPENSER_RPM            — motor speed in RPM (keep slow, ~20)
 * ─────────────────────────────────────────────────────────────────────────── */

#define TAG "dispenser"

/* ── Pin definitions (matching tested hardware) ──────────────────────────── */
#define STEP_PIN    GPIO_NUM_1
#define DIR_PIN     GPIO_NUM_39
#define EN_PIN      GPIO_NUM_48   /* active low */
#define SLP_PIN     GPIO_NUM_2
#define RST_PIN     GPIO_NUM_42
#define M0_PIN      GPIO_NUM_14
#define M1_PIN      GPIO_NUM_38
#define M2_PIN      GPIO_NUM_41

/* ── Motor constants ─────────────────────────────────────────────────────── */
#define STEPS_PER_REV      200    /* NEMA 17: 1.8° per step */
#define MICROSTEPS         8      /* 1/8 microstepping: M0=H M1=H M2=L */
#define MIN_STEP_PERIOD_US 800    /* ~1250 steps/s hard floor */

/* Dispense direction — clockwise. Flip to false if your auger runs backwards */
#define DISPENSE_CW        true

/* ── Low-level step pulse ────────────────────────────────────────────────── */

static inline void step_pulse(uint32_t period_us)
{
    const uint32_t high_us = 3;
    if (period_us < high_us + 2) period_us = high_us + 2;
    gpio_set_level(STEP_PIN, 1);
    ets_delay_us(high_us);
    gpio_set_level(STEP_PIN, 0);
    ets_delay_us(period_us - high_us);
}

/* ── Trapezoidal speed ramp ──────────────────────────────────────────────── */

static void stepper_move_ramp(uint32_t steps,
                               uint32_t start_period_us,
                               uint32_t target_period_us,
                               uint32_t ramp_steps)
{
    if (steps == 0) return;

    if (target_period_us < 10) target_period_us = 10;
    if (start_period_us < target_period_us) start_period_us = target_period_us;
    if (ramp_steps * 2 > steps) ramp_steps = steps / 2;

    /* Ramp up */
    for (uint32_t i = 0; i < ramp_steps; i++) {
        uint32_t period = start_period_us
            - (uint32_t)(((uint64_t)(start_period_us - target_period_us) * i) / ramp_steps);
        step_pulse(period);
        if ((i % 2000) == 0) vTaskDelay(1);
    }

    /* Cruise */
    uint32_t cruise = steps - 2 * ramp_steps;
    for (uint32_t i = 0; i < cruise; i++) {
        step_pulse(target_period_us);
        if ((i % 4000) == 0) vTaskDelay(1);
    }

    /* Ramp down */
    for (uint32_t i = 0; i < ramp_steps; i++) {
        uint32_t period = target_period_us
            + (uint32_t)(((uint64_t)(start_period_us - target_period_us) * i) / ramp_steps);
        step_pulse(period);
        if ((i % 2000) == 0) vTaskDelay(1);
    }
}

/* ── Internal rotate helper ──────────────────────────────────────────────── */

static void stepper_rotate(float revolutions, uint32_t rpm, bool clockwise)
{
    if (rpm == 0 || revolutions <= 0.0f) return;

    uint32_t total_steps = (uint32_t)(revolutions * STEPS_PER_REV * MICROSTEPS);

    uint64_t denom = (uint64_t)rpm * STEPS_PER_REV * MICROSTEPS;
    uint32_t target_period_us = (uint32_t)(60000000ULL / denom);
    if (target_period_us < MIN_STEP_PERIOD_US) target_period_us = MIN_STEP_PERIOD_US;

    uint32_t start_period_us = target_period_us * 4;  /* start at 1/4 speed */

    uint32_t ramp_steps = 1600;
    if (total_steps < 6000) ramp_steps = total_steps / 3;

    ESP_LOGI(TAG, "Rotating %.2f rev @ %lu RPM (%s) — %lu steps",
             revolutions, (unsigned long)rpm,
             clockwise ? "CW" : "CCW",
             (unsigned long)total_steps);

    gpio_set_level(DIR_PIN, clockwise ? 1 : 0);
    stepper_move_ramp(total_steps, start_period_us, target_period_us, ramp_steps);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void dispenser_init(void)
{
    /* STEP */
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << STEP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    /* DIR */
    cfg.pin_bit_mask = (1ULL << DIR_PIN);
    gpio_config(&cfg);

    /* EN */
    cfg.pin_bit_mask = (1ULL << EN_PIN);
    gpio_config(&cfg);

    /* SLP */
    cfg.pin_bit_mask = (1ULL << SLP_PIN);
    gpio_config(&cfg);

    /* RST */
    cfg.pin_bit_mask = (1ULL << RST_PIN);
    gpio_config(&cfg);

    /* M0, M1, M2 */
    cfg.pin_bit_mask = (1ULL << M0_PIN) | (1ULL << M1_PIN) | (1ULL << M2_PIN);
    gpio_config(&cfg);

    /* Initial pin states */
    gpio_set_level(STEP_PIN, 0);
    gpio_set_level(DIR_PIN,  0);
    gpio_set_level(EN_PIN,   1);   /* disabled (active low) */
    gpio_set_level(SLP_PIN,  1);   /* not sleeping */
    gpio_set_level(RST_PIN,  1);   /* not in reset */

    /* 1/8 microstepping: M0=H M1=H M2=L */
    gpio_set_level(M0_PIN, 1);
    gpio_set_level(M1_PIN, 1);
    gpio_set_level(M2_PIN, 0);

    ESP_LOGI(TAG, "Dispenser initialised (%.1f g/rev, %d RPM)",
             (float)CONFIG_DISPENSER_GRAMS_PER_REV,
             CONFIG_DISPENSER_RPM);
}

esp_err_t dispenser_run(int grams)
{
    if (grams <= 0) {
        ESP_LOGW(TAG, "dispenser_run called with invalid grams: %d", grams);
        return ESP_ERR_INVALID_ARG;
    }

    float revolutions = (float)grams / (float)CONFIG_DISPENSER_GRAMS_PER_REV;

    ESP_LOGI(TAG, "Dispensing %d g → %.2f revolutions @ %d RPM",
             grams, revolutions, CONFIG_DISPENSER_RPM);

    /* Enable driver */
    gpio_set_level(EN_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));  /* let driver wake up */

    stepper_rotate(revolutions, CONFIG_DISPENSER_RPM, DISPENSE_CW);

    /* Disable driver to cut idle current and prevent motor heating */
    gpio_set_level(EN_PIN, 1);

    ESP_LOGI(TAG, "Dispense complete");
    return ESP_OK;
}
