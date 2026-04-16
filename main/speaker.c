#include "speaker.h"
#include "audio_player.h"
#include "pins.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG            "speaker"
#define SPEAKER_GPIO   PIN_SPEAKER
#define PWM_FREQ_HZ    78125
#define SAMPLE_RATE_HZ 16000

#define BEEP_FREQ_HZ       880
#define BEEP_DURATION_MS   250
#define BEEP_GAP_MS        100
#define BEEP_AMPLITUDE     32000

static int16_t  *s_beep_buf   = NULL;
static uint32_t  s_beep_total = 0;

void speaker_init(void)
{
    audio_player_init(SPEAKER_GPIO, PWM_FREQ_HZ, SAMPLE_RATE_HZ);

    uint32_t tone_samples = (SAMPLE_RATE_HZ * BEEP_DURATION_MS) / 1000;
    uint32_t gap_samples  = (SAMPLE_RATE_HZ * BEEP_GAP_MS)      / 1000;
    s_beep_total          = tone_samples + gap_samples + tone_samples;
    size_t buf_bytes      = s_beep_total * sizeof(int16_t);

    // Try internal RAM first, then PSRAM
    s_beep_buf = heap_caps_malloc(buf_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_beep_buf) {
        ESP_LOGW(TAG, "Internal alloc failed (%u bytes), trying PSRAM", buf_bytes);
        s_beep_buf = heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
    }

    // Only fill buffer if allocation succeeded
    if (!s_beep_buf) {
        ESP_LOGE(TAG, "Failed to allocate beep buffer (%u bytes) — speaker disabled", buf_bytes);
        s_beep_total = 0;  // prevent speaker_beep from trying to play
        return;
    }

    // First tone
    for (uint32_t i = 0; i < tone_samples; i++) {
        s_beep_buf[i] = (int16_t)(BEEP_AMPLITUDE *
            sinf(2.0f * (float)M_PI * BEEP_FREQ_HZ * i / SAMPLE_RATE_HZ));
    }
    // Silence gap
    memset(s_beep_buf + tone_samples, 0, gap_samples * sizeof(int16_t));
    // Second tone
    for (uint32_t i = 0; i < tone_samples; i++) {
        s_beep_buf[tone_samples + gap_samples + i] = (int16_t)(BEEP_AMPLITUDE *
            sinf(2.0f * (float)M_PI * BEEP_FREQ_HZ * i / SAMPLE_RATE_HZ));
    }

    ESP_LOGI(TAG, "Speaker initialised (GPIO%d, %u bytes, %s)",
             SPEAKER_GPIO, (unsigned)buf_bytes,
             heap_caps_check_integrity_addr((intptr_t)s_beep_buf, false) ? "PSRAM" : "IRAM");
}

void speaker_beep(void)
{
    if (!s_beep_buf || s_beep_total == 0) {
        ESP_LOGE(TAG, "speaker_beep: buffer not allocated");
        return;
    }
    ESP_LOGI(TAG, "Playing beep (%d Hz, 2x %d ms)", BEEP_FREQ_HZ, BEEP_DURATION_MS);
    audio_player_start(s_beep_buf, s_beep_total);
    vTaskDelay(pdMS_TO_TICKS(10));  // yield so playback task gets scheduled
    while (audio_player_is_playing()) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    audio_player_stop();
    ESP_LOGI(TAG, "Beep complete");
}

void speaker_play_pcm(const int16_t *samples, uint32_t sample_count)
{
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