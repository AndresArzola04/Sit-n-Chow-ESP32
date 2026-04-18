#include "audio_intercom.h"
#include "audio_player.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_websocket_client.h"
#include "esp_event.h"

#define TAG "intercom"

/* ── Audio format ─────────────────────────────────────────────────────────── */

#define SAMPLE_RATE_HZ       16000
#define BYTES_PER_SAMPLE     2

/* ── Session / buffering policy ────────────────────────────────────────────
 *
 * Flutter sends one WebSocket message per captured PCM chunk while the user
 * is holding the mic button. The ESP accumulates all chunks for a single
 * "push-to-talk" session until it receives the literal text message "stop".
 *
 * This build stores up to ~16 s in PSRAM and optionally stages the first
 * 4 seconds into internal RAM for faster playback.
 */

#define MAX_SESSION_SECONDS  16
#define MAX_SESSION_SAMPLES  (SAMPLE_RATE_HZ * MAX_SESSION_SECONDS)
#define MAX_SESSION_BYTES    (MAX_SESSION_SAMPLES * BYTES_PER_SAMPLE)

#define MAX_IRAM_SECONDS     4
#define MAX_IRAM_SAMPLES     (SAMPLE_RATE_HZ * MAX_IRAM_SECONDS)
#define MAX_IRAM_BYTES       (MAX_IRAM_SAMPLES * BYTES_PER_SAMPLE)

/* ── WebSocket config ────────────────────────────────────────────────────── */

#define RECONNECT_DELAY_MS   5000
#define WS_BUFFER_SIZE       4096
#define WS_TASK_STACK        8192

#define DEVICE_ID_MAX_LEN    64

static char s_device_id[DEVICE_ID_MAX_LEN] = {0};

/* ── Session buffers ─────────────────────────────────────────────────────── */

static int16_t  *s_psram_buf      = NULL;
static uint32_t  s_psram_samples  = 0;
static int16_t  *s_iram_buf       = NULL;
static bool      s_session_open   = false;
static SemaphoreHandle_t s_play_trigger = NULL;

/* ── Session helpers ─────────────────────────────────────────────────────── */

static void session_reset(void)
{
    s_psram_samples = 0;
    s_session_open  = false;
}

static void session_play(void)
{
    if (s_psram_samples == 0) {
        ESP_LOGW(TAG, "Nothing to play");
        return;
    }

    uint32_t play_count;
    int16_t *play_buf;

    if (s_iram_buf) {
        play_count = (s_psram_samples < MAX_IRAM_SAMPLES)
                   ? s_psram_samples : MAX_IRAM_SAMPLES;
        memcpy(s_iram_buf, s_psram_buf, play_count * sizeof(int16_t));
        play_buf = s_iram_buf;
        ESP_LOGI(TAG, "Playing %.1f s from internal RAM",
                 (float)play_count / 16000.0f);
    } else {
        play_count = s_psram_samples;
        play_buf   = s_psram_buf;
        ESP_LOGW(TAG, "Playing %.1f s from PSRAM",
                 (float)play_count / 16000.0f);
    }

    audio_player_start(play_buf, play_count);
}

static void session_play_task(void *arg)
{
    while (true) {
        xSemaphoreTake(s_play_trigger, portMAX_DELAY);
        session_play();
    }
}

/* ── WebSocket URL builder ───────────────────────────────────────────────── */

static void build_ws_url(char *buf, size_t bufsz)
{
    const char *base = CONFIG_CLOUD_RUN_URL;
    const char *host = base;
    if      (strncmp(base, "https://", 8) == 0) host = base + 8;
    else if (strncmp(base, "http://",  7) == 0) host = base + 7;

    snprintf(buf, bufsz, "wss://%s/audio-stream?device=%s", host, s_device_id);
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

            if (!s_session_open && data->payload_offset == 0) {
                session_reset();
                s_session_open = true;
                ESP_LOGI(TAG, "Session started");
            }

            size_t msg_end_byte = (size_t)(s_psram_samples * 2) + data->payload_len;
            if (msg_end_byte > MAX_SESSION_BYTES) {
                ESP_LOGW(TAG, "Session buffer full (%d s max)",
                         MAX_SESSION_SAMPLES / 16000);
                break;
            }

            uint8_t *dst = (uint8_t *)s_psram_buf
                         + (s_psram_samples * 2)
                         + data->payload_offset;
            memcpy(dst, data->data_ptr, data->data_len);

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
                    s_session_open = false;
                    xSemaphoreGive(s_play_trigger);
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
    s_device_id[DEVICE_ID_MAX_LEN - 1] = '\0';

    s_psram_buf = heap_caps_malloc(MAX_SESSION_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_psram_buf) {
        ESP_LOGE(TAG, "PSRAM buffer alloc failed — intercom disabled");
        return;
    }

    s_iram_buf = heap_caps_malloc(MAX_IRAM_BYTES,
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_iram_buf) {
        ESP_LOGW(TAG, "Internal RAM unavailable for playback — will use PSRAM");
    } else {
        ESP_LOGI(TAG, "Playback buffer: %u bytes internal RAM", (unsigned)MAX_IRAM_BYTES);
    }

    s_play_trigger = xSemaphoreCreateBinary();
    configASSERT(s_play_trigger);

    xTaskCreatePinnedToCoreWithCaps(
        session_play_task,
        "intercom_play",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        NULL,
        1,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );

    xTaskCreateWithCaps(
        intercom_task,
        "audio_intercom",
        8192,
        NULL,
        7,
        NULL,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
}