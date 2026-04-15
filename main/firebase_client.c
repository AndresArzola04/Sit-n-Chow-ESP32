/*
 * firebase_client.c
 *
 * Communicates with Firebase Realtime Database using its REST API over HTTPS.
 *
 * Authentication flow:
 *   1. At init, GET <CLOUD_RUN_URL>/esp-token?device=<DEVICE_ID>
 *   2. Store the returned custom token in s_auth_token[].
 *   3. Append ?auth=<token> to every RTDB REST request.
 *   4. A FreeRTOS timer refreshes the token every 55 minutes automatically
 *      (Firebase custom tokens expire after 60 minutes).
 *
 * Mutex strategy:
 *   - s_http_mutex      : serialises all non-intercom HTTP requests
 *                         (commands, heartbeat, schedule, patches, puts, etc.)
 *   - s_intercom_mutex  : dedicated mutex for firebase_get_intercom() so the
 *                         intercom poll is never blocked by a slow command/
 *                         heartbeat request, and vice-versa.
 *   - s_token_mutex     : protects s_auth_token during reads/writes.
 */

#include "firebase_client.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_http_client.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"

/* ── Kconfig symbols ─────────────────────────────────────────────────────── *
 * CONFIG_FIREBASE_DATABASE_URL     e.g. "sit-n-chow-...-rtdb.firebaseio.com"
 * CONFIG_FIREBASE_DEVICE_ID        e.g. "SIT_N_CHOW_AABBCC"
 * CONFIG_CLOUD_RUN_URL             e.g. "https://sit-n-chow-ws-xxx-uc.a.run.app"
 * CONFIG_CLOUD_RUN_DEVICE_SECRET   optional shared secret, "" to disable
 * ─────────────────────────────────────────────────────────────────────────── */

#define TAG            "firebase"
#define URL_BUF_SIZE   3072  /* ID tokens are ~900 chars + URL overhead */
#define BODY_BUF_SIZE  8192
#define TOKEN_MAX_LEN  1536  /* ID tokens can be up to ~1200 chars */

/*
 * MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY (-0x7100):
 * Firebase closes idle TLS connections with a close_notify alert. This is
 * normal behaviour — not a real error — so we treat it as EOF when we have
 * already received data.
 */
#define MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY  (-0x7100)

static char               s_auth_token[TOKEN_MAX_LEN] = {0};
static SemaphoreHandle_t  s_token_mutex               = NULL;
static SemaphoreHandle_t  s_http_mutex                = NULL;
static SemaphoreHandle_t  s_intercom_mutex            = NULL;  /* dedicated for intercom */
static esp_timer_handle_t s_refresh_timer             = NULL;
static SemaphoreHandle_t s_camera_poll_mutex          = NULL;

/* ── Response accumulator used by the HTTP event handler ─────────────────── */

typedef struct {
    char *buf;
    int   len;
    int   cap;
} resp_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    resp_ctx_t *ctx = (resp_ctx_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && ctx && evt->data_len > 0) {
        int remaining = ctx->cap - ctx->len - 1;
        int to_copy   = (evt->data_len < remaining) ? evt->data_len : remaining;
        if (to_copy > 0) {
            memcpy(ctx->buf + ctx->len, evt->data, to_copy);
            ctx->len += to_copy;
            ctx->buf[ctx->len] = '\0';
        }
    }
    return ESP_OK;
}

/* ── Generic HTTP request ────────────────────────────────────────────────── */

static esp_err_t do_request(const char *method,
                             const char *url,
                             const char *body,
                             const char *extra_header_name,
                             const char *extra_header_value,
                             char      **out_body)
{
    char *resp_buf = calloc(1, BODY_BUF_SIZE);
    if (!resp_buf) return ESP_ERR_NO_MEM;

    resp_ctx_t ctx = { .buf = resp_buf, .len = 0, .cap = BODY_BUF_SIZE };

    esp_http_client_config_t cfg = {
        .url                         = url,
        .event_handler               = http_event_handler,
        .user_data                   = &ctx,
        .transport_type              = HTTP_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .buffer_size                 = 2048,
        .buffer_size_tx              = 1024,
        .timeout_ms                  = 15000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) { free(resp_buf); return ESP_FAIL; }

    esp_http_client_set_method(client,
        strcmp(method, "GET")   == 0 ? HTTP_METHOD_GET    :
        strcmp(method, "PATCH") == 0 ? HTTP_METHOD_PATCH  :
        strcmp(method, "PUT")   == 0 ? HTTP_METHOD_PUT    :
        strcmp(method, "POST")  == 0 ? HTTP_METHOD_POST   :
                                       HTTP_METHOD_DELETE);

    if (body && strlen(body) > 0) {
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, body, (int)strlen(body));
    }

    if (extra_header_name && extra_header_value) {
        esp_http_client_set_header(client, extra_header_name, extra_header_value);
    }

    esp_err_t err = esp_http_client_perform(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", method, esp_err_to_name(err));
    } else {
        int status = esp_http_client_get_status_code(client);
        if (status < 200 || status >= 300) {
            ESP_LOGW(TAG, "%s → HTTP %d: %s", method, status, resp_buf);
            err = ESP_FAIL;
        }
    }

    esp_http_client_cleanup(client);

    if (out_body) {
        *out_body = resp_buf;
    } else {
        free(resp_buf);
    }
    return err;
}

/* ── Token fetch from Cloud Run ──────────────────────────────────────────── */

static esp_err_t fetch_token(void)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    snprintf(url, URL_BUF_SIZE,
             "%s/esp-token?device=%s",
             CONFIG_CLOUD_RUN_URL,
             CONFIG_FIREBASE_DEVICE_ID);

    const char *secret_name  = NULL;
    const char *secret_value = NULL;

#ifdef CONFIG_CLOUD_RUN_DEVICE_SECRET
    if (strlen(CONFIG_CLOUD_RUN_DEVICE_SECRET) > 0) {
        secret_name  = "X-Device-Secret";
        secret_value = CONFIG_CLOUD_RUN_DEVICE_SECRET;
    }
#endif

    char *body = NULL;
    esp_err_t err = do_request("GET", url, NULL,
                                secret_name, secret_value,
                                &body);
    free(url);

    if (err != ESP_OK || !body) {
        free(body);
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(body);
    free(body);

    if (!json) {
        ESP_LOGE(TAG, "Token response parse error");
        return ESP_FAIL;
    }

    cJSON *token_j = cJSON_GetObjectItem(json, "token");
    if (!token_j || !cJSON_IsString(token_j)) {
        ESP_LOGE(TAG, "No 'token' field in response");
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    xSemaphoreTake(s_token_mutex, portMAX_DELAY);
    strncpy(s_auth_token, token_j->valuestring, TOKEN_MAX_LEN - 1);
    s_auth_token[TOKEN_MAX_LEN - 1] = '\0';
    xSemaphoreGive(s_token_mutex);

    ESP_LOGI(TAG, "Auth token refreshed");
    cJSON_Delete(json);
    return ESP_OK;
}

static void token_refresh_cb(void *arg)
{
    ESP_LOGI(TAG, "Refreshing Firebase auth token…");
    fetch_token();
}

/* ── URL builder with auth token ─────────────────────────────────────────── */

static void build_url(char *buf, size_t bufsz, const char *path)
{
    xSemaphoreTake(s_token_mutex, portMAX_DELAY);
    snprintf(buf, bufsz,
             "https://%s/%s.json?auth=%s",
             CONFIG_FIREBASE_DATABASE_URL,
             path,
             s_auth_token);
    xSemaphoreGive(s_token_mutex);
}

/* ── Shared streaming GET implementation ─────────────────────────────────── *
 *
 * Used by both firebase_get_large() and firebase_get_intercom().
 * The caller passes in the mutex to take/give so each can use its own.
 *
 * Key fix vs. original:
 *   - esp_http_client_read() returning < 0 with MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY
 *     is treated as a clean EOF when data has already been received, not an error.
 *   - content_length == -1 (chunked / no Content-Length header) is handled
 *     gracefully — we just read until rlen == 0.
 *   - HTTP status != 2xx is logged with the actual status code, not -1.
 * ─────────────────────────────────────────────────────────────────────────── */

static esp_err_t streaming_get(const char          *url,
                                SemaphoreHandle_t    mutex,
                                cJSON              **out_json)
{
    esp_http_client_config_t cfg = {
        .url                         = url,
        .transport_type              = HTTP_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .buffer_size                 = 8192,
        .buffer_size_tx              = 2048,
        .timeout_ms                  = 20000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return ESP_ERR_NO_MEM;

    esp_http_client_set_method(client, HTTP_METHOD_GET);

    xSemaphoreTake(mutex, portMAX_DELAY);

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "streaming_get open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        xSemaphoreGive(mutex);
        return err;
    }

    /* fetch_headers returns content-length or -1 for chunked/unknown */
    int content_length = esp_http_client_fetch_headers(client);
    int status         = esp_http_client_get_status_code(client);

    ESP_LOGD(TAG, "streaming_get: HTTP %d, Content-Length: %d", status, content_length);

    if (status < 200 || status >= 300) {
        ESP_LOGW(TAG, "streaming_get: HTTP %d (non-2xx)", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        xSemaphoreGive(mutex);
        return ESP_FAIL;
    }

    /* Allocate response buffer in PSRAM; start at 64 KB, grow as needed */
    size_t cap      = 65536;
    char  *resp_buf = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!resp_buf) {
        /* Fall back to internal heap for small responses */
        cap      = 8192;
        resp_buf = malloc(cap);
    }
    if (!resp_buf) {
        ESP_LOGE(TAG, "streaming_get: failed to allocate response buffer");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        xSemaphoreGive(mutex);
        return ESP_ERR_NO_MEM;
    }

    int   total_read = 0;
    char  read_buf[512];
    err = ESP_OK;

    while (1) {
        int rlen = esp_http_client_read(client, read_buf, sizeof(read_buf));

        if (rlen < 0) {
            /*
             * MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY is fired when Firebase
             * closes the TLS connection with a close_notify alert after
             * sending the full response body. This is normal — treat it
             * as EOF when we already have data.
             */
            if (total_read > 0) {
                ESP_LOGD(TAG, "streaming_get: peer close after %d bytes (normal)", total_read);
            } else {
                ESP_LOGE(TAG, "streaming_get: read error with no data received: %d", rlen);
                err = ESP_FAIL;
            }
            break;
        }

        if (rlen == 0) {
            break;  /* clean EOF */
        }

        /* Grow buffer if needed (double until 512 KB cap) */
        if (total_read + rlen + 1 > (int)cap) {
            size_t new_cap = cap * 2;
            if (new_cap > 524288) {
                ESP_LOGE(TAG, "streaming_get: response exceeds 512 KB limit");
                err = ESP_FAIL;
                break;
            }
            char *new_buf = realloc(resp_buf, new_cap);
            if (!new_buf) {
                ESP_LOGE(TAG, "streaming_get: realloc to %u bytes failed", new_cap);
                err = ESP_FAIL;
                break;
            }
            resp_buf = new_buf;
            cap      = new_cap;
        }

        memcpy(resp_buf + total_read, read_buf, rlen);
        total_read              += rlen;
        resp_buf[total_read]     = '\0';
    }

    ESP_LOGI(TAG, "streaming_get: received %d bytes total", total_read);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    xSemaphoreGive(mutex);

    if (out_json) *out_json = NULL;

    if (err == ESP_OK && total_read > 0 && out_json) {
        if (strcmp(resp_buf, "null") == 0) {
            /* Node does not exist — caller gets NULL json, ESP_OK */
            ESP_LOGD(TAG, "streaming_get: node is null");
        } else {
            cJSON *parsed = cJSON_Parse(resp_buf);
            if (!parsed) {
                ESP_LOGW(TAG, "streaming_get: JSON parse error (first 64 chars: %.64s)", resp_buf);
                err = ESP_FAIL;
            } else {
                *out_json = parsed;
            }
        }
    }

    free(resp_buf);
    return err;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t firebase_client_init(void)
{
    s_token_mutex = xSemaphoreCreateMutex();
    configASSERT(s_token_mutex);

    s_http_mutex = xSemaphoreCreateMutex();
    configASSERT(s_http_mutex);

    s_intercom_mutex = xSemaphoreCreateMutex();
    configASSERT(s_intercom_mutex);

    s_camera_poll_mutex = xSemaphoreCreateMutex();
    configASSERT(s_camera_poll_mutex);

    /* Fetch initial token — retry up to 5 times */
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 5; i++) {
        err = fetch_token();
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "Token fetch attempt %d/5 failed, retrying in 5s…", i + 1);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not obtain Firebase auth token");
        return ESP_FAIL;
    }

    /* Refresh timer — every 55 minutes (tokens expire at 60 min) */
    const esp_timer_create_args_t timer_args = {
        .callback = token_refresh_cb,
        .name     = "fb_token_refresh",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_refresh_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_refresh_timer,
                                             (uint64_t)55 * 60 * 1000000ULL));

    ESP_LOGI(TAG, "Firebase client ready (DB: %s)", CONFIG_FIREBASE_DATABASE_URL);
    return ESP_OK;
}

esp_err_t firebase_get(const char *path, cJSON **out_json)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);

    char *body = NULL;
    esp_err_t err = do_request("GET", url, NULL, NULL, NULL, &body);
    free(url);

    xSemaphoreGive(s_http_mutex);

    if (out_json) *out_json = NULL;

    if (err == ESP_OK && body && strcmp(body, "null") != 0 && out_json) {
        cJSON *parsed = cJSON_Parse(body);
        if (!parsed) {
            ESP_LOGW(TAG, "JSON parse error for: %s", path);
            err = ESP_FAIL;
        } else {
            *out_json = parsed;
        }
    }

    free(body);
    return err;
}

/*
 * firebase_get_intercom — streaming GET dedicated to the audio intercom poller.
 *
 * Uses its own mutex (s_intercom_mutex) so the intercom task is never blocked
 * by slow command/heartbeat/schedule requests, and those tasks are never
 * blocked waiting for an intercom poll to complete.
 *
 * The audio node is typically a few KB of base64 PCM — well within the 64 KB
 * initial buffer — so this should be fast in the normal case.
 */
esp_err_t firebase_get_intercom(const char *path, cJSON **out_json)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);

    ESP_LOGD(TAG, "firebase_get_intercom: polling %s", path);

    esp_err_t err = streaming_get(url, s_intercom_mutex, out_json);
    free(url);
    return err;
}

esp_err_t firebase_get_camera_poll(const char *path, cJSON **out_json)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);
    esp_err_t err = streaming_get(url, s_camera_poll_mutex, out_json);
    free(url);
    return err;
}

esp_err_t firebase_patch(const char *path, cJSON *json)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);

    char *body = cJSON_PrintUnformatted(json);
    if (!body) { free(url); return ESP_ERR_NO_MEM; }

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    esp_err_t err = do_request("PATCH", url, body, NULL, NULL, NULL);
    xSemaphoreGive(s_http_mutex);

    free(url);
    free(body);
    return err;
}

esp_err_t firebase_put(const char *path, cJSON *json)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);

    char *body = cJSON_PrintUnformatted(json);
    if (!body) { free(url); return ESP_ERR_NO_MEM; }

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    esp_err_t err = do_request("PUT", url, body, NULL, NULL, NULL);
    xSemaphoreGive(s_http_mutex);

    free(url);
    free(body);
    return err;
}

esp_err_t firebase_push(const char *path, cJSON *json)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);

    char *body = cJSON_PrintUnformatted(json);
    if (!body) { free(url); return ESP_ERR_NO_MEM; }

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    esp_err_t err = do_request("POST", url, body, NULL, NULL, NULL);
    xSemaphoreGive(s_http_mutex);

    free(url);
    free(body);
    return err;
}

esp_err_t firebase_delete(const char *path)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    esp_err_t err = do_request("DELETE", url, NULL, NULL, NULL, NULL);
    xSemaphoreGive(s_http_mutex);

    free(url);
    return err;
}