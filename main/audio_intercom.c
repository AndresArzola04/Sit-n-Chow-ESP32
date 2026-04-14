/*
 * audio_intercom.c  —  Buffer-and-play intercom
 *
 * Accumulates all PCM chunks from the Flutter app into a PSRAM buffer while
 * the mic is open, then plays the entire buffer at once when "stop" arrives.
 * This avoids the WiFi DMA jitter that disrupts real-time busy-wait playback.
 *
 * Both buffers (PSRAM accumulation + internal-RAM playback) are pre-allocated
 * at startup so heap fragmentation during WiFi operation cannot cause failures.
 */

#include "audio_intercom.h"
#include "audio_player.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_websocket_client.h"

#include "sdkconfig.h"

#define TAG                 "intercom"
#define DEVICE_ID_MAX_LEN   64
#define RECONNECT_DELAY_MS  5000
#define WS_BUFFER_SIZE      (64 * 1024)
#define WS_TASK_STACK       (8 * 1024)

/* 8 seconds max — 8 * 16000 * 2 = 256 KB */
#define MAX_SESSION_SAMPLES (16000 * 8)
#define MAX_SESSION_BYTES   (MAX_SESSION_SAMPLES * sizeof(int16_t))

static char s_device_id[DEVICE_ID_MAX_LEN] = {0};

/* PSRAM: accumulates incoming chunks */
static int16_t  *s_psram_buf     = NULL;
static uint32_t  s_psram_samples = 0;

/* Internal RAM: copy-before-play for glitch-free busy-wait timing.
 * Pre-allocated at startup when heap is unfragmented.
 * Falls back to PSRAM if allocation fails. */
static int16_t  *s_iram_buf      = NULL;   /* internal RAM playback buffer */
static bool      s_session_open  = false;

/* ── Session management ──────────────────────────────────────────────────── */

static void session_reset(void)
{
    /* Stop playback if running and wait for the beep task to finish */
    if (audio_player_is_playing()) {
        audio_player_stop();
        /* Spin-wait — beep task exits within one sample period (~63 µs) */
        int guard = 100;
        while (audio_player_is_playing() && guard-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    s_psram_samples = 0;
    s_session_open  = false;
}

static void session_play(void)
{
    if (s_psram_samples == 0) {
        ESP_LOGW(TAG, "Nothing to play");
        return;
    }

    uint32_t play_bytes = s_psram_samples * sizeof(int16_t);
    ESP_LOGI(TAG, "Playing %.1f s of buffered audio (%lu samples)",
             (float)s_psram_samples / 16000.0f,
             (unsigned long)s_psram_samples);

    /* Choose playback buffer: internal RAM preferred, PSRAM fallback */
    int16_t *play_buf;
    if (s_iram_buf) {
        memcpy(s_iram_buf, s_psram_buf, play_bytes);
        play_buf = s_iram_buf;
        ESP_LOGI(TAG, "Playback from internal RAM");
    } else {
        play_buf = s_psram_buf;
        ESP_LOGW(TAG, "Playback from PSRAM (internal RAM unavailable)");
    }

    /* Hand off to audio_player — uses the same proven path as the beep */
    audio_player_start(play_buf, s_psram_samples);
}

/* ── WebSocket URL builder ───────────────────────────────────────────────── */

static void build_ws_url(char *buf, size_t bufsz)
{
    const char *base = CONFIG_CLOUD_RUN_URL;
    const char *host = base;
    if      (strncmp(base, "https://", 8) == 0) host = base + 8;
    else if (strncmp(base, "http://",  7) == 0) host = base + 7;
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
        session_reset();
        break;

    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x02) {

            if (!s_psram_buf) break;

            /* Start a new session on the first fragment of a new message */
            if (!s_session_open && data->payload_offset == 0) {
                session_reset();
                s_session_open = true;
                ESP_LOGI(TAG, "Session started");
            }

            /* Bounds check against the session buffer */
            size_t msg_end_byte = (size_t)(s_psram_samples * 2) + data->payload_len;
            if (msg_end_byte > MAX_SESSION_BYTES) {
                ESP_LOGW(TAG, "Session buffer full (%d s max)",
                        MAX_SESSION_SAMPLES / 16000);
                break;
            }

            /* Copy THIS fragment into its correct position within the message */
            uint8_t *dst = (uint8_t *)s_psram_buf
                        + (s_psram_samples * 2)
                        + data->payload_offset;
            memcpy(dst, data->data_ptr, data->data_len);

            /* Only commit the samples once the full message is reassembled */
            bool complete = (data->payload_offset + data->data_len
                            >= (size_t)data->payload_len);
            if (complete) {
                uint32_t new_samples = (uint32_t)(data->payload_len / 2);
                s_psram_samples += new_samples;
                ESP_LOGI(TAG, "Buffered %lu samples total (%.1f s)",
                        (unsigned long)s_psram_samples,
                        (float)s_psram_samples / 16000.0f);
            }

        } else if (data->op_code == 0x01) {
            if (data->data_len >= 4 &&
                strncmp(data->data_ptr, "stop", 4) == 0) {
                ESP_LOGI(TAG, "Stop received — playing buffered audio");
                if (s_session_open) {
                    session_play();
                    s_session_open = false;
                }
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

/* ── Intercom background task ────────────────────────────────────────────── */

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

    /* Allocate buffers at startup — heap is unfragmented here */
    s_psram_buf = heap_caps_malloc(MAX_SESSION_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_psram_buf) {
        ESP_LOGE(TAG, "PSRAM buffer alloc failed — intercom disabled");
        return;
    }

    /* Try to get internal RAM for glitch-free playback timing.
     * This is the same memory region used by the beep buffer. */
    s_iram_buf = heap_caps_malloc(MAX_SESSION_BYTES,
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_iram_buf) {
        ESP_LOGW(TAG, "Internal RAM unavailable for playback — will use PSRAM");
    } else {
        ESP_LOGI(TAG, "Playback buffer: %u bytes internal RAM",
                 (unsigned)MAX_SESSION_BYTES);
    }

    xTaskCreate(intercom_task, "audio_intercom", 4096, NULL, 4, NULL);
}