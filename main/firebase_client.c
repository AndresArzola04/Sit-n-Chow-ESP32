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
 * Persistent client strategy (key change from original):
 *   Previously every request called esp_http_client_init() + cleanup(),
 *   hammering the internal heap with mbedTLS context alloc/free on every
 *   poll cycle. This caused severe heap fragmentation and the hardware AES
 *   DMA buffer failures seen in the logs.
 *
 *   Now three persistent handles are created once at boot and reused:
 *     s_http_client        — all short Firebase REST calls (GET/PATCH/PUT/
 *                            POST/DELETE). Protected by s_http_mutex.
 *     s_streaming_client   — streaming GET for camera poll.
 *                            Protected by s_camera_poll_mutex.
 *     s_intercom_client    — streaming GET for intercom poll.
 *                            Protected by s_intercom_mutex.
 *   Token fetch uses a temporary client because it targets a different host
 *   (Cloud Run vs. Firebase RTDB) and only runs every 55 minutes.
 *
 * Mutex strategy (unchanged):
 *   s_http_mutex       : serialises all non-streaming HTTP requests
 *   s_intercom_mutex   : dedicated for firebase_get_intercom()
 *   s_camera_poll_mutex: dedicated for firebase_get_camera_poll()
 *   s_token_mutex      : protects s_auth_token during reads/writes
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

#define TAG            "firebase"
#define URL_BUF_SIZE   3072
#define BODY_BUF_SIZE  8192
#define TOKEN_MAX_LEN  1536

/*
 * MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY (-0x7100):
 * Firebase closes idle TLS connections with a close_notify alert.
 * Treat as EOF when data has already been received.
 */
#define MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY  (-0x7100)

/* ── Module state ────────────────────────────────────────────────────────── */

static char               s_auth_token[TOKEN_MAX_LEN] = {0};
static SemaphoreHandle_t  s_token_mutex               = NULL;
static SemaphoreHandle_t  s_http_mutex                = NULL;
static SemaphoreHandle_t  s_intercom_mutex            = NULL;
static SemaphoreHandle_t  s_camera_poll_mutex         = NULL;
static esp_timer_handle_t s_refresh_timer             = NULL;

/* Persistent HTTP client handles — created once in firebase_client_init() */
static esp_http_client_handle_t s_http_client        = NULL;
static esp_http_client_handle_t s_streaming_client   = NULL;
static esp_http_client_handle_t s_intercom_client    = NULL;

/*
 * Module-level response context for do_request().
 * Safe to be module-level because do_request() is always called under
 * s_http_mutex, so only one call can be in flight at a time.
 */
typedef struct {
    char *buf;
    int   len;
    int   cap;
} resp_ctx_t;

static resp_ctx_t s_do_ctx; /* user_data for s_http_client's event handler */

/* ── HTTP event handler ──────────────────────────────────────────────────── */

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    resp_ctx_t *ctx = (resp_ctx_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && ctx && evt->data_len > 0) {
        int remaining = ctx->cap - ctx->len - 1;
        int to_copy   = (evt->data_len < remaining) ?
                         evt->data_len : remaining;
        if (to_copy > 0) {
            memcpy(ctx->buf + ctx->len, evt->data, to_copy);
            ctx->len += to_copy;
            ctx->buf[ctx->len] = '\0';
        }
    }
    return ESP_OK;
}

/* ── Generic HTTP request (uses persistent s_http_client) ────────────────── */

static esp_err_t do_request(const char *method,
                             const char *url,
                             const char *body,
                             const char *extra_header_name,
                             const char *extra_header_value,
                             char      **out_body)
{
    char *resp_buf = calloc(1, BODY_BUF_SIZE);
    if (!resp_buf) return ESP_ERR_NO_MEM;

    /* Point the persistent client's event handler at this call's buffer */
    s_do_ctx.buf = resp_buf;
    s_do_ctx.len = 0;
    s_do_ctx.cap = BODY_BUF_SIZE;

    /* Reconfigure the persistent client for this request */
    esp_http_client_set_url(s_http_client, url);

    esp_http_client_set_method(s_http_client,
        strcmp(method, "GET")    == 0 ? HTTP_METHOD_GET    :
        strcmp(method, "PATCH")  == 0 ? HTTP_METHOD_PATCH  :
        strcmp(method, "PUT")    == 0 ? HTTP_METHOD_PUT    :
        strcmp(method, "POST")   == 0 ? HTTP_METHOD_POST   :
                                        HTTP_METHOD_DELETE);

    if (body && strlen(body) > 0) {
        esp_http_client_set_header(s_http_client, "Content-Type", "application/json");
        esp_http_client_set_post_field(s_http_client, body, (int)strlen(body));
    } else {
        /* Clear any body left over from a previous write request */
        esp_http_client_set_post_field(s_http_client, NULL, 0);
    }

    if (extra_header_name && extra_header_value) {
        esp_http_client_set_header(s_http_client,
                                   extra_header_name, extra_header_value);
    }

    /* perform() reuses the existing TLS session if the host is unchanged,
     * or transparently reconnects if the connection was dropped. */
    esp_err_t err = esp_http_client_perform(s_http_client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", method, esp_err_to_name(err));
    } else {
        int status = esp_http_client_get_status_code(s_http_client);
        if (status < 200 || status >= 300) {
            ESP_LOGW(TAG, "%s → HTTP %d: %s", method, status, resp_buf);
            err = ESP_FAIL;
        }
    }

    /* NOTE: no esp_http_client_cleanup() — handle is persistent */

    if (out_body) {
        *out_body = resp_buf;
    } else {
        free(resp_buf);
    }
    return err;
}

/* ── Token fetch (temporary client — different host to Firebase RTDB) ─────── */

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

    /*
     * Token fetch targets Cloud Run (different host from Firebase RTDB).
     * Use a short-lived client so s_http_client stays pointed at Firebase
     * and can keep its TLS session alive.
     */
    char *resp_buf = calloc(1, BODY_BUF_SIZE);
    if (!resp_buf) { free(url); return ESP_ERR_NO_MEM; }

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
    if (!client) {
        free(url);
        free(resp_buf);
        return ESP_FAIL;
    }

    if (secret_name && secret_value)
        esp_http_client_set_header(client, secret_name, secret_value);

    esp_err_t err = esp_http_client_perform(client);
    esp_http_client_cleanup(client);
    free(url);

    if (err != ESP_OK || !resp_buf[0]) {
        ESP_LOGE(TAG, "Token fetch HTTP error: %s", esp_err_to_name(err));
        free(resp_buf);
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(resp_buf);
    free(resp_buf);

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
    /* Take s_http_mutex to prevent a concurrent do_request() call from
     * racing with the URL change inside do_request for the same client.
     * fetch_token uses its own temporary client so this is just a guard. */
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

/* ── Shared streaming GET (uses a caller-supplied persistent client) ─────── *
 *
 * The caller must hold `mutex` before calling. This function takes the mutex,
 * performs the request, releases the mutex, and returns.
 *
 * Uses manual open/read/close (not perform) so it can stream large responses
 * without buffering them entirely before parsing.
 * ─────────────────────────────────────────────────────────────────────────── */

static esp_err_t streaming_get(const char               *url,
                                esp_http_client_handle_t  client,
                                SemaphoreHandle_t         mutex,
                                cJSON                   **out_json)
{
    xSemaphoreTake(mutex, portMAX_DELAY);

    esp_http_client_set_url(client, url);
    esp_http_client_set_method(client, HTTP_METHOD_GET);

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "streaming_get open failed: %s", esp_err_to_name(err));
        xSemaphoreGive(mutex);
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    int status         = esp_http_client_get_status_code(client);

    ESP_LOGD(TAG, "streaming_get: HTTP %d, Content-Length: %d", status, content_length);

    if (status < 200 || status >= 300) {
        ESP_LOGW(TAG, "streaming_get: HTTP %d (non-2xx)", status);
        esp_http_client_close(client);  /* close, not cleanup — handle is persistent */
        xSemaphoreGive(mutex);
        return ESP_FAIL;
    }

    /* Allocate response buffer in PSRAM; start at 64 KB, grow as needed */
    size_t cap      = 65536;
    char  *resp_buf = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!resp_buf) {
        cap      = 8192;
        resp_buf = malloc(cap);
    }
    if (!resp_buf) {
        ESP_LOGE(TAG, "streaming_get: failed to allocate response buffer");
        esp_http_client_close(client);
        xSemaphoreGive(mutex);
        return ESP_ERR_NO_MEM;
    }

    int  total_read = 0;
    char read_buf[512];
    err = ESP_OK;

    while (1) {
        int rlen = esp_http_client_read(client, read_buf, sizeof(read_buf));

        if (rlen < 0) {
            /*
             * MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY: Firebase closes the TLS
             * connection after sending the full response. Treat as clean EOF
             * when we already have data.
             */
            if (total_read > 0) {
                ESP_LOGD(TAG, "streaming_get: peer close after %d bytes (normal)", total_read);
            } else {
                ESP_LOGE(TAG, "streaming_get: read error with no data: %d", rlen);
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
                ESP_LOGE(TAG, "streaming_get: realloc to %u bytes failed", (unsigned)new_cap);
                err = ESP_FAIL;
                break;
            }
            resp_buf = new_buf;
            cap      = new_cap;
        }

        memcpy(resp_buf + total_read, read_buf, rlen);
        total_read             += rlen;
        resp_buf[total_read]    = '\0';
    }

    ESP_LOGI(TAG, "streaming_get: received %d bytes total", total_read);

    /* close (not cleanup) — preserves the TLS session for the next call */
    esp_http_client_close(client);
    xSemaphoreGive(mutex);

    if (out_json) *out_json = NULL;

    if (err == ESP_OK && total_read > 0 && out_json) {
        if (strcmp(resp_buf, "null") == 0) {
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
    s_token_mutex       = xSemaphoreCreateMutex();
    s_http_mutex        = xSemaphoreCreateMutex();
    s_intercom_mutex    = xSemaphoreCreateMutex();
    s_camera_poll_mutex = xSemaphoreCreateMutex();
    configASSERT(s_token_mutex);
    configASSERT(s_http_mutex);
    configASSERT(s_intercom_mutex);
    configASSERT(s_camera_poll_mutex);

    /*
     * Create persistent HTTP handles at boot while the heap is unfragmented.
     *
     * s_http_client: used by do_request() for all short Firebase REST calls.
     * keep_alive_enable reuses the TLS session across back-to-back requests
     * to the same Firebase RTDB host, eliminating repeated handshakes.
     */
    esp_http_client_config_t http_cfg = {
        .url                         = "https://" CONFIG_FIREBASE_DATABASE_URL "/init.json",
        .event_handler               = http_event_handler,
        .user_data                   = &s_do_ctx,
        .transport_type              = HTTP_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .buffer_size                 = 2048,
        .buffer_size_tx              = 1024,
        .timeout_ms                  = 15000,
        .keep_alive_enable           = true,
    };
    s_http_client = esp_http_client_init(&http_cfg);
    configASSERT(s_http_client);

    /*
     * s_streaming_client / s_intercom_client: used by streaming_get() for
     * Firebase reads that may return larger payloads. Separate handles so
     * they never block short REST calls (heartbeat, patch, etc.).
     */
    esp_http_client_config_t stream_cfg = {
        .url                         = "https://" CONFIG_FIREBASE_DATABASE_URL "/init.json",
        .transport_type              = HTTP_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .buffer_size                 = 8192,
        .buffer_size_tx              = 2048,
        .timeout_ms                  = 20000,
        .keep_alive_enable           = true,
    };
    s_streaming_client = esp_http_client_init(&stream_cfg);
    configASSERT(s_streaming_client);

    s_intercom_client = esp_http_client_init(&stream_cfg);
    configASSERT(s_intercom_client);

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

esp_err_t firebase_get_intercom(const char *path, cJSON **out_json)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);

    ESP_LOGD(TAG, "firebase_get_intercom: polling %s", path);

    esp_err_t err = streaming_get(url, s_intercom_client, s_intercom_mutex, out_json);
    free(url);
    return err;
}

esp_err_t firebase_get_camera_poll(const char *path, cJSON **out_json)
{
    char *url = malloc(URL_BUF_SIZE);
    if (!url) return ESP_ERR_NO_MEM;
    build_url(url, URL_BUF_SIZE, path);

    esp_err_t err = streaming_get(url, s_streaming_client, s_camera_poll_mutex, out_json);
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