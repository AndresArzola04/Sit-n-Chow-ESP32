/*
 * audio_intercom.c
 *
 * Polls Firebase RTDB for base64-encoded PCM audio chunks sent by the
 * Flutter app's live intercom feature, decodes them, and plays them
 * through the speaker.
 *
 * Poll interval: 1 second (faster than command_poll_task's 5 s, to keep
 * intercom lag low).
 *
 * Memory: decode buffer allocated in PSRAM (MALLOC_CAP_SPIRAM) since a
 * 2-second 16 kHz mono chunk is ~64 KB — too large to comfortably sit in
 * internal RAM alongside everything else.
 */

#include "audio_intercom.h"
#include "firebase_client.h"
#include "speaker.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "mbedtls/base64.h"

#define TAG               "intercom"
#define POLL_INTERVAL_MS  7000
#define DEVICE_ID_MAX_LEN 64

/* Path buffer: "devices/<id>/audio" */
#define PATH_BUF_SIZE     96

static char s_device_id[DEVICE_ID_MAX_LEN] = {0};

/* ── Base64 decode ───────────────────────────────────────────────────────────
 *
 * mbedtls_base64_decode is already available in ESP-IDF (mbedTLS is enabled
 * in sdkconfig). It decodes in-place into a caller-provided output buffer.
 *
 * Returns a heap_caps_malloc'd buffer in PSRAM that the caller must free(),
 * and writes the decoded byte count to *out_len.
 * Returns NULL on error.
 */
static uint8_t *base64_decode_to_psram(const char *b64_str,
                                        size_t      b64_len,
                                        size_t     *out_len)
{
    /* mbedtls needs the output size up-front — decoded is at most 3/4 of input */
    size_t buf_size = (b64_len / 4) * 3 + 4;

    uint8_t *buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    if (!buf) {
        /* Fall back to internal RAM if PSRAM not available */
        buf = malloc(buf_size);
    }
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes for decode buffer", buf_size);
        return NULL;
    }

    size_t decoded_len = 0;
    int ret = mbedtls_base64_decode(buf, buf_size, &decoded_len,
                                    (const unsigned char *)b64_str, b64_len);
    if (ret != 0) {
        ESP_LOGE(TAG, "base64 decode failed: -0x%04X", -ret);
        free(buf);
        return NULL;
    }

    *out_len = decoded_len;
    return buf;
}

/* ── Intercom poll task ───────────────────────────────────────────────────── */

static void intercom_task(void *arg)
{
    char path[PATH_BUF_SIZE];
    snprintf(path, sizeof(path), "devices/%s/audio", s_device_id);

    int last_chunk_index = -1;  /* track last played chunk to avoid replaying */

    ESP_LOGI(TAG, "Intercom task started, polling: %s", path);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));

        cJSON *json = NULL;
        esp_err_t err = firebase_get_large(path, &json, 0);  // size ignored, grows dynamically

        /* Node deleted (mic toggled off) or read error — nothing to do */
        if (err != ESP_OK || json == NULL) {
            last_chunk_index = -1;  /* reset so next activation plays immediately */
            continue;
        }

        /* Check command field */
        cJSON *command = cJSON_GetObjectItem(json, "command");
        if (!command || !cJSON_IsString(command) ||
            strcmp(command->valuestring, "play") != 0) {
            cJSON_Delete(json);
            continue;
        }

        /* Check chunkIndex — skip if we already played this chunk */
        cJSON *chunk_idx_j = cJSON_GetObjectItem(json, "chunkIndex");
        int chunk_index = (chunk_idx_j && cJSON_IsNumber(chunk_idx_j))
                          ? (int)chunk_idx_j->valuedouble
                          : 0;

        if (chunk_index == last_chunk_index) {
            cJSON_Delete(json);
            continue;  /* same chunk, already played */
        }

        /* Get the base64 PCM data */
        cJSON *data_j = cJSON_GetObjectItem(json, "data");
        if (!data_j || !cJSON_IsString(data_j) ||
            strlen(data_j->valuestring) == 0) {
            ESP_LOGW(TAG, "Chunk %d has no data field", chunk_index);
            cJSON_Delete(json);
            continue;
        }

        const char *b64 = data_j->valuestring;
        size_t b64_len  = strlen(b64);

        ESP_LOGI(TAG, "New chunk %d received (%u b64 chars)", chunk_index, b64_len);

        /* Decode base64 → raw PCM bytes */
        size_t pcm_bytes = 0;
        uint8_t *pcm_buf = base64_decode_to_psram(b64, b64_len, &pcm_bytes);

        cJSON_Delete(json);  /* done with JSON, free before we block on playback */

        if (!pcm_buf) {
            ESP_LOGE(TAG, "Decode failed for chunk %d", chunk_index);
            continue;
        }

        /* pcm_bytes is the number of raw bytes.
         * Each sample is int16_t (2 bytes), so sample_count = pcm_bytes / 2. */
        if (pcm_bytes < 2) {
            ESP_LOGW(TAG, "Chunk %d too small (%u bytes)", chunk_index, pcm_bytes);
            free(pcm_buf);
            continue;
        }

        uint32_t sample_count = (uint32_t)(pcm_bytes / 2);

        ESP_LOGI(TAG, "Playing chunk %d: %u bytes → %lu samples",
                 chunk_index, pcm_bytes, (unsigned long)sample_count);

        /* Stop any currently playing audio before starting new chunk */
        if (speaker_is_playing()) {
            speaker_stop();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* Hand the buffer to the speaker.
         * speaker_play_pcm is non-blocking — it spawns the playback task.
         * The playback task holds a pointer to pcm_buf, so we must NOT free
         * it here. Instead we wait for playback to finish, then free. */
        speaker_play_pcm((const int16_t *)pcm_buf, sample_count);

        /* Wait for this chunk to finish before polling for the next one.
         * This naturally rate-limits us to real-time audio playback. */
        while (speaker_is_playing()) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        free(pcm_buf);
        last_chunk_index = chunk_index;

        ESP_LOGI(TAG, "Chunk %d playback complete", chunk_index);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void audio_intercom_start(const char *device_id)
{
    strncpy(s_device_id, device_id, DEVICE_ID_MAX_LEN - 1);

    xTaskCreate(
        intercom_task,
        "audio_intercom",
        8192,        /* stack: needs room for cJSON + base64 string pointers */
        NULL,
        4,           /* same priority as command_poll_task */
        NULL
    );

    ESP_LOGI(TAG, "Audio intercom started (device=%s)", s_device_id);
}
