#include "audio_player.h"

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

static const char* TAG = "audio_player";

// --- Config (set by audio_player_init) ---
static int      s_gpio_pin      = 0;
static uint32_t s_sample_rate   = 16000;

// --- Playback state ---
static volatile bool s_playing  = false;
static TaskHandle_t  s_task     = NULL;

// --- LEDC channel config ---
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT   // 8-bit = 256 levels

// Write a duty value (0-255) to the LEDC channel
static inline void audio_write_duty(uint32_t duty) {
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
}

// --- Playback task ---
typedef struct {
    const int16_t* samples;
    uint32_t       sample_count;
} playback_args_t;

static void audio_playback_task(void* pvParameters) {
    playback_args_t* args = (playback_args_t*)pvParameters;
    const int16_t*   samples      = args->samples;
    uint32_t         sample_count = args->sample_count;
    free(args);

    ESP_LOGI(TAG, "Playback started: %lu samples (~%.1f sec)",
        sample_count, (float)sample_count / s_sample_rate);

    // Calculate delay per sample in microseconds
    s_playing = true;
    uint32_t period_us = 1000000 / s_sample_rate;  // 62us @ 16kHz
    int64_t next_sample_time = esp_timer_get_time();

    for (uint32_t i = 0; i < sample_count && s_playing; i++) {
        uint32_t duty = ((int32_t)samples[i] + 32768) >> 8;
        audio_write_duty(duty);

        next_sample_time += period_us;

        // Busy-wait until next sample time
        while (esp_timer_get_time() < next_sample_time) {}
    }

    // Silence output when done
    audio_write_duty(128);  // 50% = DC midpoint = silence
    s_playing = false;
    s_task    = NULL;

    ESP_LOGI(TAG, "Playback complete.");
    vTaskDelete(NULL);
}

// --- Public API ---

void audio_player_init(int gpio_pin, uint32_t pwm_freq_hz, uint32_t sample_rate_hz) {
    s_gpio_pin    = gpio_pin;
    s_sample_rate = sample_rate_hz;

    // Configure LEDC timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_SPEED_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = pwm_freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // Configure LEDC channel
    ledc_channel_config_t channel_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio_pin,
        .duty       = 128,   // start at midpoint = silence
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));

    ESP_LOGI(TAG, "Initialized: GPIO%d, PWM %luHz, Sample rate %luHz",
        gpio_pin, pwm_freq_hz, sample_rate_hz);
}

void audio_player_start(const int16_t* samples, uint32_t sample_count) {
    if (s_playing) {
        ESP_LOGW(TAG, "Already playing, stopping first.");
        audio_player_stop();
    }

    // Heap-allocate args so they survive until the task picks them up
    playback_args_t* args = malloc(sizeof(playback_args_t));
    args->samples      = samples;
    args->sample_count = sample_count;

    // Pin to core 1 so it doesn't compete with WiFi/BT on core 0
    xTaskCreatePinnedToCore(
        audio_playback_task,
        "audio_playback",
        4096,
        args,
        configMAX_PRIORITIES - 1,  // highest priority for tight timing
        &s_task,
        1   // core 1
    );
}

bool audio_player_is_playing(void) {
    return s_playing;
}

void audio_player_stop(void) {
    s_playing = false;
    if (s_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));  // give task time to exit cleanly
    }
    audio_write_duty(128);
    ESP_LOGI(TAG, "Playback stopped.");
}
