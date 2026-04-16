/*
 * audio_player.c
 *
 * Two playback modes:
 *  1. One-shot (audio_player_start): semaphore wakes a CPU1 task. For beep.
 *  2. Synchronous (audio_player_play_sync): runs in caller's context.
 *     Caller must be on CPU1 at high priority. Used by audio_intercom for
 *     gapless chunk-to-chunk streaming with zero inter-task gap.
 */

#include "audio_player.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

static const char *TAG = "audio_player";

static int      s_gpio_pin    = 0;
static uint32_t s_sample_rate = 16000;

static volatile bool     s_playing  = false;
static TaskHandle_t      s_task     = NULL;
static SemaphoreHandle_t s_play_sem = NULL;

static const int16_t *s_samples      = NULL;
static uint32_t       s_sample_count = 0;

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT

static void ledc_mute(void); // forward declaration — mute on init and after playback

static void ledc_mute(void)
{
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    ledc_stop(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_timer_pause(LEDC_SPEED_MODE, LEDC_TIMER);  // ← stops the clock entirely
    gpio_set_direction(s_gpio_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_gpio_pin, 0);
}

static void ledc_unmute(void)
{
    // Re-attach GPIO to LEDC before resuming — ledc_stop() released it
    ledc_channel_config_t channel_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = s_gpio_pin,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&channel_cfg);
    ledc_timer_resume(LEDC_SPEED_MODE, LEDC_TIMER);
}

static inline void play_samples(const int16_t *samples, uint32_t count)
{
    uint32_t period_us = 1000000UL / s_sample_rate;
    int64_t  next_time = esp_timer_get_time();

    for (uint32_t i = 0; i < count && s_playing; i++) {
        uint32_t duty = ((int32_t)samples[i] + 32768) >> 8;
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        next_time += period_us;
        while (esp_timer_get_time() < next_time) {}
    }
}

static void audio_playback_task(void *arg)
{
    ESP_LOGI(TAG, "Playback started.");
    while (true) {
        xSemaphoreTake(s_play_sem, portMAX_DELAY);

        const int16_t *samples      = s_samples;
        uint32_t       sample_count = s_sample_count;

        if (!samples || sample_count == 0) { s_playing = false; continue; }

        ledc_unmute();
        play_samples(samples, sample_count);
        ledc_mute();
        s_playing = false;
        ESP_LOGI(TAG, "Playback complete.");
    }
}

void audio_player_init(int gpio_pin, uint32_t pwm_freq_hz, uint32_t sample_rate_hz)
{
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

    s_play_sem = xSemaphoreCreateBinary();
    configASSERT(s_play_sem);

    xTaskCreatePinnedToCore(audio_playback_task, "audio_beep", 4096, NULL,
                             configMAX_PRIORITIES - 1, &s_task, 1);

    ledc_mute(); // silence immediately — prevents noise on boot
    ESP_LOGI(TAG, "Initialized: GPIO%d, PWM %luHz, Sample rate %luHz",
             gpio_pin, (unsigned long)pwm_freq_hz, (unsigned long)sample_rate_hz);
}

void audio_player_start(const int16_t *samples, uint32_t sample_count)
{
    if (s_playing) { audio_player_stop(); vTaskDelay(pdMS_TO_TICKS(5)); }
    s_samples      = samples;
    s_sample_count = sample_count;
    s_playing      = true;
    xSemaphoreGive(s_play_sem);
}

bool audio_player_is_playing(void) { return s_playing; }

void audio_player_stop(void)
{
    s_playing = false;
    ledc_mute();
    ESP_LOGI(TAG, "Playback stopped.");
}

void audio_player_play_sync(const int16_t *samples, uint32_t sample_count)
{
    s_playing = true;
    play_samples(samples, sample_count);
    /* Caller decides when to mute — allows zero-gap chaining */
}

void audio_player_unmute_output(void) { ledc_unmute(); }

void audio_player_mute_output(void) { ledc_mute(); s_playing = false; }