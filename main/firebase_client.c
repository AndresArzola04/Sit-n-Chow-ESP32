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
 */

#include "firebase_client.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_http_client.h"
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
#define URL_BUF_SIZE   640
#define BODY_BUF_SIZE  4096
#define TOKEN_MAX_LEN  1024   /* custom tokens are ~900 chars */

static char               s_auth_token[TOKEN_MAX_LEN] = {0};
static SemaphoreHandle_t  s_token_mutex               = NULL;
static esp_timer_handle_t s_refresh_timer             = NULL;

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
        .buffer_size_tx              = 2048,
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
    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url),
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

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t firebase_client_init(void)
{
    s_token_mutex = xSemaphoreCreateMutex();
    configASSERT(s_token_mutex);

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
    char url[URL_BUF_SIZE];
    build_url(url, sizeof(url), path);

    char *body = NULL;
    esp_err_t err = do_request("GET", url, NULL, NULL, NULL, &body);

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

esp_err_t firebase_patch(const char *path, cJSON *json)
{
    char url[URL_BUF_SIZE];
    build_url(url, sizeof(url), path);
    char *body = cJSON_PrintUnformatted(json);
    if (!body) return ESP_ERR_NO_MEM;
    esp_err_t err = do_request("PATCH", url, body, NULL, NULL, NULL);
    free(body);
    return err;
}

esp_err_t firebase_put(const char *path, cJSON *json)
{
    char url[URL_BUF_SIZE];
    build_url(url, sizeof(url), path);
    char *body = cJSON_PrintUnformatted(json);
    if (!body) return ESP_ERR_NO_MEM;
    esp_err_t err = do_request("PUT", url, body, NULL, NULL, NULL);
    free(body);
    return err;
}

esp_err_t firebase_push(const char *path, cJSON *json)
{
    char url[URL_BUF_SIZE];
    build_url(url, sizeof(url), path);
    char *body = cJSON_PrintUnformatted(json);
    if (!body) return ESP_ERR_NO_MEM;
    esp_err_t err = do_request("POST", url, body, NULL, NULL, NULL);
    free(body);
    return err;
}

esp_err_t firebase_delete(const char *path)
{
    char url[URL_BUF_SIZE];
    build_url(url, sizeof(url), path);
    return do_request("DELETE", url, NULL, NULL, NULL, NULL);
}
