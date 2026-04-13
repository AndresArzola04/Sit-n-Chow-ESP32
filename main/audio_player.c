#include "audio_player.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "audio_player";

static int      s_gpio_pin    = 0;
static uint32_t s_sample_rate = 16000;

static volatile bool s_playing = false;
static TaskHandle_t  s_task    = NULL;

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT

static inline void audio_write_duty(uint32_t duty) {
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
}

static void audio_mute(void) {
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    ledc_timer_pause(LEDC_SPEED_MODE, LEDC_TIMER);
}

static void audio_unmute(void) {
    ledc_timer_resume(LEDC_SPEED_MODE, LEDC_TIMER);
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 128);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
}

typedef struct {
    const int16_t* samples;
    uint32_t       sample_count;
} playback_args_t;

static void audio_playback_task(void* pvParameters) {
    playback_args_t* args = (playback_args_t*)pvParameters;
    const int16_t*   samples      = args->samples;
    uint32_t         sample_count = args->sample_count;
    free(args);

    s_playing = true;
    uint32_t period_us = 1000000 / s_sample_rate;
    int64_t  next_time = esp_timer_get_time();

    for (uint32_t i = 0; i < sample_count && s_playing; i++) {
        uint32_t duty = ((int32_t)samples[i] + 32768) >> 8;
        audio_write_duty(duty);
        next_time += period_us;
        /* Pure busy-wait — do NOT use vTaskDelay() here.
         * vTaskDelay(1) sleeps for one FreeRTOS tick (10 ms at 100 Hz).
         * One sample period at 16 kHz is only 62.5 µs — sleeping a tick
         * stretches every sample by 160×, producing noise instead of audio. */
        while (esp_timer_get_time() < next_time) {}
    }

    audio_mute();
    s_playing = false;
    s_task    = NULL;
    ESP_LOGI(TAG, "Playback complete.");
    vTaskDelete(NULL);
}

void audio_player_init(int gpio_pin, uint32_t pwm_freq_hz, uint32_t sample_rate_hz) {
    s_gpio_pin    = gpio_pin;
    s_sample_rate = sample_rate_hz;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_SPEED_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = pwm_freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio_pin,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));

    ledc_timer_pause(LEDC_SPEED_MODE, LEDC_TIMER);

    ESP_LOGI(TAG, "Initialized: GPIO%d, PWM %luHz, Sample rate %luHz",
        gpio_pin, pwm_freq_hz, sample_rate_hz);
}

void audio_player_start(const int16_t* samples, uint32_t sample_count) {
    if (s_playing) {
        audio_player_stop();
    }

    audio_unmute();

    playback_args_t* args = malloc(sizeof(playback_args_t));
    args->samples      = samples;
    args->sample_count = sample_count;

    xTaskCreatePinnedToCore(
        audio_playback_task,
        "audio_playback",
        4096,
        args,
        configMAX_PRIORITIES - 1,
        &s_task,
        1
    );
}

bool audio_player_is_playing(void) {
    return s_playing;
}

void audio_player_stop(void) {
    s_playing = false;
    if (s_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    audio_mute();
    ESP_LOGI(TAG, "Playback stopped.");
}