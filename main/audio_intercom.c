/*
 * audio_intercom.c
 *
 * Receives audio from the Flutter app via a persistent WebSocket connection
 * to the Cloud Run server (/audio-stream), and plays it through the speaker.
 *
 * The WebSocket event handler never blocks. If a chunk arrives while the
 * speaker is already playing, it is dropped — this keeps the event loop
 * unblocked so the WebSocket client stays healthy and future chunks arrive
 * on time.
 *
 * A dedicated playback task owns the PCM buffer and frees it when done.
 */

#include "audio_intercom.h"
#include "speaker.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_websocket_client.h"

#include "sdkconfig.h"

#define TAG                  "intercom"
#define DEVICE_ID_MAX_LEN    64
#define RECONNECT_DELAY_MS   5000
#define WS_BUFFER_SIZE       (32 * 1024)
#define WS_TASK_STACK        (8 * 1024)

static char s_device_id[DEVICE_ID_MAX_LEN] = {0};

/* ── PCM buffer handed to a playback task ───────────────────────────────── */

typedef struct {
    int16_t *buf;
    uint32_t sample_count;
} play_args_t;

static void playback_task(void *arg)
{
    play_args_t *a = (play_args_t *)arg;

    if (speaker_is_playing()) {
        /* Speaker still busy from previous chunk — drop this one */
        ESP_LOGW(TAG, "Speaker busy, dropping chunk of %lu samples",
                 (unsigned long)a->sample_count);
        free(a->buf);
        free(a);
        vTaskDelete(NULL);
        return;
    }

    speaker_play_pcm(a->buf, a->sample_count);

    /* Wait for playback to finish before freeing the buffer.
     * speaker_play_pcm is non-blocking and holds a pointer to buf. */
    while (speaker_is_playing()) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    free(a->buf);
    free(a);
    ESP_LOGI(TAG, "Playback complete");
    vTaskDelete(NULL);
}

/* ── Build the WebSocket URL ─────────────────────────────────────────────── */

static void build_ws_url(char *buf, size_t bufsz)
{
    const char *base = CONFIG_CLOUD_RUN_URL;
    const char *host = base;

    if (strncmp(base, "https://", 8) == 0)       host = base + 8;
    else if (strncmp(base, "http://", 7) == 0)   host = base + 7;

    snprintf(buf, bufsz, "wss://%s/audio-stream", host);
}

/* ── WebSocket event handler ─────────────────────────────────────────────── */

static void ws_event_handler(void *handler_args,
                              esp_event_base_t base,
                              int32_t event_id,
                              void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {

    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket connected to audio-stream");
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "WebSocket disconnected");
        if (speaker_is_playing()) speaker_stop();
        break;

    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x02) {
            /* Binary frame — raw PCM chunk */

            /* Only handle complete frames */
            bool complete = (data->payload_offset + data->data_len
                             >= (size_t)data->payload_len);
            if (!complete) {
                ESP_LOGD(TAG, "Partial WS frame (%d/%d bytes)",
                         data->payload_offset + data->data_len,
                         data->payload_len);
                break;
            }

            int pcm_bytes = data->payload_len;
            if (pcm_bytes < 2) break;

            /* If speaker is already playing, drop this chunk immediately
             * without allocating memory — keeps the event handler fast */
            if (speaker_is_playing()) {
                ESP_LOGW(TAG, "Speaker busy — dropping %d byte chunk", pcm_bytes);
                break;
            }

            /* Allocate in PSRAM */
            int16_t *pcm_buf = heap_caps_malloc(pcm_bytes, MALLOC_CAP_SPIRAM);
            if (!pcm_buf) pcm_buf = malloc(pcm_bytes);
            if (!pcm_buf) {
                ESP_LOGE(TAG, "Failed to alloc %d bytes for PCM", pcm_bytes);
                break;
            }

            memcpy(pcm_buf, data->data_ptr, pcm_bytes);

            uint32_t sample_count = (uint32_t)(pcm_bytes / 2);
            ESP_LOGI(TAG, "Received %d bytes → %lu samples",
                     pcm_bytes, (unsigned long)sample_count);

            /* Hand off to a dedicated task so this handler returns immediately */
            play_args_t *args = malloc(sizeof(play_args_t));
            if (!args) {
                free(pcm_buf);
                break;
            }
            args->buf          = pcm_buf;
            args->sample_count = sample_count;

            if (xTaskCreate(playback_task, "pcm_play", 4096, args, 5, NULL)
                != pdPASS) {
                ESP_LOGE(TAG, "Failed to create playback task");
                free(pcm_buf);
                free(args);
            }

        } else if (data->op_code == 0x01) {
            /* Text frame */
            if (data->data_len >= 4 &&
                strncmp(data->data_ptr, "stop", 4) == 0) {
                ESP_LOGI(TAG, "Received stop signal");
                if (speaker_is_playing()) speaker_stop();
            }
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WebSocket error");
        break;

    default:
        break;
    }
}

/* ── Intercom task ───────────────────────────────────────────────────────── */

static void intercom_task(void *arg)
{
    char ws_url[256];
    build_ws_url(ws_url, sizeof(ws_url));
    ESP_LOGI(TAG, "Audio stream URL: %s", ws_url);

    esp_websocket_client_config_t ws_cfg = {
        .uri                         = ws_url,
        .transport                   = WEBSOCKET_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .buffer_size                 = WS_BUFFER_SIZE,
        .task_stack                  = WS_TASK_STACK,
        .reconnect_timeout_ms        = RECONNECT_DELAY_MS,
        .network_timeout_ms          = 10000,
        .ping_interval_sec           = 20,
        .disable_auto_reconnect      = false,
    };

    esp_websocket_client_handle_t client = esp_websocket_client_init(&ws_cfg);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init WebSocket client");
        vTaskDelete(NULL);
        return;
    }

    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY,
                                   ws_event_handler, NULL);

    esp_websocket_client_start(client);
    ESP_LOGI(TAG, "Audio intercom started (device=%s)", s_device_id);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "Audio WS alive: %s",
                 esp_websocket_client_is_connected(client) ? "yes" : "reconnecting…");
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void audio_intercom_start(const char *device_id)
{
    strncpy(s_device_id, device_id, DEVICE_ID_MAX_LEN - 1);

    xTaskCreate(intercom_task, "audio_intercom", 4096, NULL, 4, NULL);
}