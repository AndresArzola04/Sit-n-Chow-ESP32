/*
 * tof.c
 *
 * Time-of-Flight presence detection using DFRobot TMF8701.
 *
 * The sensor is initialised once at boot and left running continuously
 * so it's immediately ready when the feed workflow needs it. This avoids
 * the ~500ms startup delay per feed cycle.
 *
 * Presence detection uses a sliding window of consecutive readings
 * below the threshold (CONFIG_TOF_SAMPLES_TO_CONFIRM) to filter out
 * single noisy spikes.
 */

#include "tof.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmf8701_c_api.h"

#define TAG "tof"

/* ── I2C config ──────────────────────────────────────────────────────────── */
#define I2C_PORT          I2C_NUM_0
#define I2C_SDA_PIN       GPIO_NUM_37
#define I2C_SCL_PIN       GPIO_NUM_36
#define I2C_FREQ_HZ       400000      /* 400 kHz fast mode */
#define TMF8701_ADDR      0x41

/* ── Calibration data from hardware testing ──────────────────────────────── */
static const uint8_t CALIB_DATA[TMF8701_CALIB_DATA_SIZE] = {
    0x0F, 0x3F, 0x8A, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x41, 0x57, 0x01, 0x00, 0x00, 0x00
};

/* ── How many consecutive sub-threshold readings confirm presence ─────────── */
/* Configurable via menuconfig → Feeder Hardware Configuration */
#ifndef CONFIG_TOF_SAMPLES_TO_CONFIRM
#define CONFIG_TOF_SAMPLES_TO_CONFIRM 3
#endif

/* ── Module state ────────────────────────────────────────────────────────── */
static DFRobot_TMF8701_C *s_tof  = NULL;
static bool               s_ready = false;

/* ── Internal helpers ────────────────────────────────────────────────────── */

static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_PIN,
        .scl_io_num       = I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialised (SDA=%d SCL=%d @ %d Hz)",
             I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t tof_init(void)
{
    i2c_init();

    s_tof = tmf8701_create(GPIO_NUM_NC, GPIO_NUM_NC, I2C_PORT, TMF8701_ADDR);
    if (s_tof == NULL) {
        ESP_LOGE(TAG, "Failed to create TMF8701 instance");
        return ESP_FAIL;
    }

    /* Retry begin() up to 5 times — sensor can be slow to wake */
    for (int i = 0; i < 5; i++) {
        if (tmf8701_begin(s_tof) == 0) break;
        ESP_LOGW(TAG, "TMF8701 begin attempt %d/5 failed", i + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (i == 4) {
            ESP_LOGE(TAG, "TMF8701 init failed after 5 attempts");
            tmf8701_destroy(s_tof);
            s_tof = NULL;
            return ESP_FAIL;
        }
    }

    /* Load known-good calibration data */
    if (!tmf8701_set_calibration_data(s_tof,
                                      CALIB_DATA,
                                      TMF8701_CALIB_DATA_SIZE)) {
        ESP_LOGW(TAG, "Failed to set calibration data — continuing without");
    }

    /* Start continuous measurement in COMBINE mode (best range) */
    if (!tmf8701_start_measurement_ex(s_tof,
                                      TMF8701_CALIB_MODE_NO_CALIB,
                                      TMF8701_DISTANCE_MODE_COMBINE)) {
        ESP_LOGE(TAG, "TMF8701 start_measurement failed");
        tmf8701_destroy(s_tof);
        s_tof = NULL;
        return ESP_FAIL;
    }

    /* Warm-up delay */
    vTaskDelay(pdMS_TO_TICKS(500));

    s_ready = true;
    ESP_LOGI(TAG, "TMF8701 ready (threshold set per workflow, %d confirm samples)",
             CONFIG_TOF_SAMPLES_TO_CONFIRM);
    return ESP_OK;
}

bool tof_wait_for_presence(uint16_t threshold_mm, uint32_t timeout_ms)
{
    if (!s_ready || s_tof == NULL) {
        ESP_LOGW(TAG, "ToF not initialised — skipping presence check");
        return false;
    }

    ESP_LOGI(TAG, "Waiting for presence (threshold=%u mm, timeout=%lums)",
             threshold_mm, (unsigned long)timeout_ms);

    TickType_t deadline     = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    int        confirm_count = 0;

    while (xTaskGetTickCount() < deadline) {

        if (tmf8701_is_data_ready(s_tof)) {
            uint16_t dist_mm = tmf8701_get_distance_mm(s_tof);
            ESP_LOGD(TAG, "Distance: %u mm", dist_mm);

            if (dist_mm > 0 && dist_mm < threshold_mm) {
                confirm_count++;
                ESP_LOGI(TAG, "Sub-threshold reading %d/%d (%u mm)",
                         confirm_count, CONFIG_TOF_SAMPLES_TO_CONFIRM, dist_mm);

                if (confirm_count >= CONFIG_TOF_SAMPLES_TO_CONFIRM) {
                    ESP_LOGI(TAG, "Presence confirmed at %u mm", dist_mm);
                    return true;
                }
            } else {
                /* Reset counter on any reading above threshold */
                if (confirm_count > 0) {
                    ESP_LOGD(TAG, "Reading above threshold (%u mm) — resetting counter", dist_mm);
                    confirm_count = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "Presence timeout after %lu ms", (unsigned long)timeout_ms);
    return false;
}
