/*
 * speaker.c
 *
 * Speaker module for Sit-N-Chow feeder.
 * Wraps audio_player (LEDC PWM) and generates tones programmatically.
 *
 * Beep tone is synthesised at runtime into a small stack buffer — no
 * audio header file needed for the attention beep.
 *
 * For future app-to-speaker streaming, speaker_play_pcm() passes
 * directly through to audio_player_start().
 */

#include "speaker.h"
#include "audio_player.h"
#include "pins.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG            "speaker"
#define SPEAKER_GPIO   PIN_SPEAKER
#define PWM_FREQ_HZ    20000
#define SAMPLE_RATE_HZ 16000

/* ── Beep parameters ─────────────────────────────────────────────────────── *
 * Two-tone pattern: 880 Hz for 250 ms, silence 100 ms, 880 Hz for 250 ms.
 * 880 Hz is in the dog hearing sweet spot and cuts through ambient noise.
 * ─────────────────────────────────────────────────────────────────────────── */
#define BEEP_FREQ_HZ       880
#define BEEP_DURATION_MS   250
#define BEEP_GAP_MS        100
#define BEEP_AMPLITUDE     28000    /* out of 32767 — loud but not clipping */

/* ── Public API ──────────────────────────────────────────────────────────── */

void speaker_init(void)
{
    audio_player_init(SPEAKER_GPIO, PWM_FREQ_HZ, SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "Speaker initialised (GPIO%d, %d Hz PWM, %d Hz sample rate)",
             SPEAKER_GPIO, PWM_FREQ_HZ, SAMPLE_RATE_HZ);
}

void speaker_beep(void)
{
    /* Number of samples for one beep tone */
    uint32_t tone_samples = (SAMPLE_RATE_HZ * BEEP_DURATION_MS) / 1000;  /* 4000 */
    uint32_t gap_samples  = (SAMPLE_RATE_HZ * BEEP_GAP_MS)      / 1000;  /* 1600 */
    uint32_t total        = tone_samples + gap_samples + tone_samples;    /* 9600 */

    int16_t *buf = malloc(total * sizeof(int16_t));
    if (!buf) {
        ESP_LOGE(TAG, "speaker_beep: malloc failed");
        return;
    }

    /* First tone */
    for (uint32_t i = 0; i < tone_samples; i++) {
        buf[i] = (int16_t)(BEEP_AMPLITUDE *
                 sinf(2.0f * (float)M_PI * BEEP_FREQ_HZ * i / SAMPLE_RATE_HZ));
    }

    /* Silence gap */
    memset(buf + tone_samples, 0, gap_samples * sizeof(int16_t));

    /* Second tone */
    for (uint32_t i = 0; i < tone_samples; i++) {
        buf[tone_samples + gap_samples + i] = (int16_t)(BEEP_AMPLITUDE *
                 sinf(2.0f * (float)M_PI * BEEP_FREQ_HZ * i / SAMPLE_RATE_HZ));
    }

    ESP_LOGI(TAG, "Playing beep (%d Hz, 2x %d ms)", BEEP_FREQ_HZ, BEEP_DURATION_MS);

    /* audio_player_start is non-blocking — we block here until done
     * so the feed workflow waits for the beep before activating the ToF */
    audio_player_start(buf, total);
    while (audio_player_is_playing()) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    free(buf);
    ESP_LOGI(TAG, "Beep complete");
}

void speaker_play_pcm(const int16_t *samples, uint32_t sample_count)
{
    /* Non-blocking — caller manages the buffer lifetime */
    audio_player_start(samples, sample_count);
}

void speaker_stop(void)
{
    audio_player_stop();
}

bool speaker_is_playing(void)
{
    return audio_player_is_playing();
}