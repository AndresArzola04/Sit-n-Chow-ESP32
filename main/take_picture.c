/*
 * take_picture.c  —  main application entry point
 *
 * Responsibilities:
 *   1. Connect WiFi (BLE provisioning via app_wifi_main)
 *   2. Sync time via SNTP
 *   3. Initialise Firebase client
 *   4. Fetch device config (owner UID) from Firebase
 *   5. Report online status heartbeat every 30 s
 *   6. Poll commands/pending every 5 s  (feed-now from the app)
 *   7. Start schedule runner (timed auto-feeds)
 *   8. Stream camera frames to Cloud Run WebSocket
 *
 * Feed workflow (on_feed_trigger):
 *   [beep] → [ToF wait] → [camera detect] → [dispense] →
 *   [ultrasonic check] → [log event] → [push notification]
 *
 * Hardware not yet wired: speaker, ToF, ultrasonic.
 * Those sections are marked TODO and will be filled in subsequent PRs.
 */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

#include "camera_pin.h"
#include "app_wifi.h"
#include "esp_camera.h"
#include "esp_websocket_client.h"

#include "firebase_client.h"
#include "schedule_runner.h"
#include "cJSON.h"

/* ── Build-time config (set in idf.py menuconfig) ───────────────────────── */
// CONFIG_FIREBASE_DEVICE_ID   — unique string that identifies this feeder
// CONFIG_FIREBASE_DATABASE_URL
// CONFIG_FIREBASE_DB_SECRET

#define TAG "feeder"

/* ── Camera / WebSocket (unchanged from original) ────────────────────────── */

#define TEST_ESP_OK(ret)          assert(ret == ESP_OK)
#define TEST_ASSERT_NOT_NULL(ret) assert(ret != NULL)

static bool              auto_jpeg_support = false;
static QueueHandle_t     xQueueIFrame      = NULL;
static esp_websocket_client_handle_t ws_client = NULL;

/* ── Device identity (fetched from Firebase at boot) ─────────────────────── */

static char s_owner_uid[64] = {0};     /* fetched from devices/<id>/ownerUid */
static bool s_identity_ready = false;

/* ── Feed workflow mutex (prevent concurrent feeds) ──────────────────────── */
static SemaphoreHandle_t s_feed_mutex = NULL;

/* ════════════════════════════════════════════════════════════════════════════
 *  Camera / WebSocket helpers  (unchanged from original take_picture.c)
 * ════════════════════════════════════════════════════════════════════════════ */

void ws_client_init(const char *uri)
{
    esp_websocket_client_config_t ws_cfg = {
        .uri                       = uri,
        .transport                 = WEBSOCKET_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .cert_pem                  = NULL,
        .disable_auto_reconnect    = false,
        .buffer_size               = 1024 * 32,
        .network_timeout_ms        = 10000,
        .reconnect_timeout_ms      = 5000,
    };
    ws_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_client_start(ws_client);
}

void ws_send_task(void *arg)
{
    camera_fb_t *frame;
    while (true) {
        if (xQueueReceive(xQueueIFrame, &frame, portMAX_DELAY)) {
            if (esp_websocket_client_is_connected(ws_client)) {
                esp_websocket_client_send_bin(ws_client,
                    (const char *)frame->buf, frame->len,
                    pdMS_TO_TICKS(5000));
            }
            esp_camera_fb_return(frame);
        }
    }
}

static esp_err_t init_camera(uint32_t xclk_freq_hz,
                              pixformat_t pixel_format,
                              framesize_t frame_size,
                              uint8_t fb_count)
{
    camera_config_t camera_config = {
        .pin_pwdn      = CAMERA_PIN_PWDN,
        .pin_reset     = CAMERA_PIN_RESET,
        .pin_xclk      = CAMERA_PIN_XCLK,
        .pin_sscb_sda  = CAMERA_PIN_SIOD,
        .pin_sscb_scl  = CAMERA_PIN_SIOC,
        .pin_d7        = CAMERA_PIN_D7,
        .pin_d6        = CAMERA_PIN_D6,
        .pin_d5        = CAMERA_PIN_D5,
        .pin_d4        = CAMERA_PIN_D4,
        .pin_d3        = CAMERA_PIN_D3,
        .pin_d2        = CAMERA_PIN_D2,
        .pin_d1        = CAMERA_PIN_D1,
        .pin_d0        = CAMERA_PIN_D0,
        .pin_vsync     = CAMERA_PIN_VSYNC,
        .pin_href      = CAMERA_PIN_HREF,
        .pin_pclk      = CAMERA_PIN_PCLK,
        .xclk_freq_hz  = xclk_freq_hz,
        .ledc_timer    = LEDC_TIMER_0,
        .ledc_channel  = LEDC_CHANNEL_0,
        .pixel_format  = pixel_format,
        .frame_size    = frame_size,
        .jpeg_quality  = 25,
        .fb_count      = fb_count,
        .grab_mode     = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location   = CAMERA_FB_IN_PSRAM,
    };

    esp_err_t ret = esp_camera_init(&camera_config);

    sensor_t *s = esp_camera_sensor_get();
    s->set_reg(s, 0x3035, 0xff, 0x21);

    if (s->id.PID == OV5640_PID) {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 0);
        s->set_quality(s, 10);
    }

    camera_sensor_info_t *s_info = esp_camera_sensor_get_info(&(s->id));
    if (ret == ESP_OK && pixel_format == PIXFORMAT_JPEG && s_info->support_jpeg) {
        auto_jpeg_support = true;
    }

    return ret;
}

static esp_err_t reinit_camera(void)
{
    ESP_LOGW(TAG, "Reinitialising camera…");
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t err = init_camera(20000000, PIXFORMAT_JPEG, FRAMESIZE_HVGA, 2);
    ESP_LOGI(TAG, "Camera reinit: %s", esp_err_to_name(err));
    return err;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  SNTP  —  synchronise local clock with NTP
 * ════════════════════════════════════════════════════════════════════════════ */

static void sntp_sync_wait(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    ESP_LOGI(TAG, "Waiting for SNTP sync…");
    int retry = 0;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < 20) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    time_t now;
    time(&now);
    char buf[32];
    struct tm tm_info;
    localtime_r(&now, &tm_info);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_info);
    ESP_LOGI(TAG, "Clock synced: %s", buf);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Firebase helpers
 * ════════════════════════════════════════════════════════════════════════════ */

/** Report online presence and firmware version. */
static void report_online(void)
{
    cJSON *status = cJSON_CreateObject();
    cJSON_AddBoolToObject(status,   "online",   true);
    cJSON_AddNumberToObject(status, "lastSeen", (double)time(NULL) * 1000.0);
    cJSON_AddStringToObject(status, "fw",       "1.0.0");

    char path[128];
    snprintf(path, sizeof(path), "devices/%s/status", CONFIG_FIREBASE_DEVICE_ID);
    firebase_patch(path, status);
    cJSON_Delete(status);
}

/** Fetch ownerUid from devices/<id> so the schedule runner knows which user. */
static bool fetch_owner_uid(void)
{
    char path[128];
    snprintf(path, sizeof(path), "devices/%s/ownerUid", CONFIG_FIREBASE_DEVICE_ID);

    cJSON *json = NULL;
    esp_err_t err = firebase_get(path, &json);

    if (err == ESP_OK && json && cJSON_IsString(json)) {
        strncpy(s_owner_uid, json->valuestring, sizeof(s_owner_uid) - 1);
        ESP_LOGI(TAG, "Owner UID: %s", s_owner_uid);
        cJSON_Delete(json);
        return true;
    }

    if (json) cJSON_Delete(json);
    ESP_LOGW(TAG, "Could not fetch owner UID — device may not be provisioned yet");
    return false;
}

/**
 * Log a completed feed event to Firebase:
 *   feedEvents/<deviceId>/<pushKey>
 */
static void log_feed_event(int grams, const char *source)
{
    cJSON *event = cJSON_CreateObject();
    cJSON_AddNumberToObject(event, "ts",     (double)time(NULL) * 1000.0);
    cJSON_AddNumberToObject(event, "grams",  grams);
    cJSON_AddStringToObject(event, "source", source);
    cJSON_AddStringToObject(event, "by",     "device");

    char path[128];
    snprintf(path, sizeof(path), "feedEvents/%s", CONFIG_FIREBASE_DEVICE_ID);
    firebase_push(path, event);
    cJSON_Delete(event);
}

/**
 * Write a FCM notification request into Firebase so a Cloud Function
 * (or your Node.js backend) can forward it to the user's device.
 * Structure: notifications/<uid>/<pushKey>
 */
static void push_notification(const char *title, const char *body)
{
    if (!s_identity_ready) return;

    cJSON *notif = cJSON_CreateObject();
    cJSON_AddStringToObject(notif, "title", title);
    cJSON_AddStringToObject(notif, "body",  body);
    cJSON_AddNumberToObject(notif, "ts",    (double)time(NULL) * 1000.0);
    cJSON_AddStringToObject(notif, "deviceId", CONFIG_FIREBASE_DEVICE_ID);

    char path[128];
    snprintf(path, sizeof(path), "notifications/%s", s_owner_uid);
    firebase_push(path, notif);
    cJSON_Delete(notif);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Feed workflow
 * ════════════════════════════════════════════════════════════════════════════
 *
 *  Steps:
 *   1. Beep speaker            (TODO: wire GPIO/DAC)
 *   2. Activate ToF sensor     (TODO: wire I²C ToF)
 *   3. Wait for pet approach   (TODO: threshold logic)
 *   4. Run sitting detector    (TODO: ML inference)
 *   5. Dispense food           (TODO: stepper/servo GPIO)
 *   6. Ultrasonic capacity check (TODO: wire HC-SR04)
 *   7. Log event + notify user
 */

static void do_feed_workflow(int grams, const char *source)
{
    /* Guard: only one feed at a time */
    if (xSemaphoreTake(s_feed_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Feed already in progress — ignoring trigger");
        return;
    }

    ESP_LOGI(TAG, "=== Feed workflow START (%d g, source=%s) ===", grams, source);

    /* ── Step 1: Beep ────────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "[1] Beep speaker");
    // TODO: gpio_set_level(SPEAKER_GPIO, 1);
    // vTaskDelay(pdMS_TO_TICKS(500));
    // gpio_set_level(SPEAKER_GPIO, 0);

    /* ── Step 2 & 3: ToF — wait for pet to approach ─────────────────────── */
    ESP_LOGI(TAG, "[2] Activating ToF sensor, waiting for pet…");
    bool pet_detected = false;
    // TODO: tof_start();
    // TickType_t tof_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(60000);
    // while (xTaskGetTickCount() < tof_deadline) {
    //     int dist_mm = tof_read_mm();
    //     if (dist_mm > 0 && dist_mm < TOF_THRESHOLD_MM) { pet_detected = true; break; }
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }
    // tof_stop();
    pet_detected = true;  /* placeholder until hardware is wired */

    if (!pet_detected) {
        ESP_LOGW(TAG, "No pet detected within timeout — aborting feed");
        xSemaphoreGive(s_feed_mutex);
        return;
    }

    /* ── Step 4: Camera sitting detection ────────────────────────────────── */
    ESP_LOGI(TAG, "[3] Running sitting detector…");
    bool sitting_confirmed = false;
    // TODO: sitting_confirmed = sitting_detector_run(timeout_ms=180000);
    sitting_confirmed = true;  /* placeholder */

    if (!sitting_confirmed) {
        ESP_LOGW(TAG, "Pet not sitting after 3 min — dispensing anyway");
    }

    /* ── Step 5: Dispense ────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "[4] Dispensing %d g…", grams);
    // TODO: dispenser_run(grams);
    vTaskDelay(pdMS_TO_TICKS(2000));  /* placeholder dispense delay */

    /* ── Step 6: Ultrasonic food level check ─────────────────────────────── */
    ESP_LOGI(TAG, "[5] Checking food level…");
    bool food_low = false;
    // TODO: int distance_cm = ultrasonic_read_cm();
    // food_low = (distance_cm > LOW_FOOD_THRESHOLD_CM);
    food_low = false;   /* placeholder */

    /* ── Step 7: Log event + notifications ───────────────────────────────── */
    log_feed_event(grams, source);

    push_notification("🐾 Feeding complete",
                      sitting_confirmed
                          ? "Your pet was detected sitting and has been fed!"
                          : "Your pet was fed (sitting not confirmed).");

    if (food_low) {
        push_notification("⚠️ Low food level",
                          "Your feeder is running low — time to refill!");

        /* Also write a flag to Firebase so the app can show a badge */
        char path[128];
        snprintf(path, sizeof(path), "devices/%s/inventory/lowFood",
                 CONFIG_FIREBASE_DEVICE_ID);
        cJSON *flag = cJSON_CreateTrue();
        firebase_put(path, flag);
        cJSON_Delete(flag);
    }

    ESP_LOGI(TAG, "=== Feed workflow DONE ===");
    xSemaphoreGive(s_feed_mutex);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Command polling  —  reads commands/<deviceId>/pending from Firebase
 * ════════════════════════════════════════════════════════════════════════════ */

static void handle_pending_command(cJSON *cmd)
{
    cJSON *action = cJSON_GetObjectItem(cmd, "action");
    if (!action || !cJSON_IsString(action)) return;

    if (strcmp(action->valuestring, "dispense") == 0) {
        cJSON *grams_j = cJSON_GetObjectItem(cmd, "grams");
        int grams = (grams_j && cJSON_IsNumber(grams_j))
                    ? (int)grams_j->valuedouble : 25;

        ESP_LOGI(TAG, "Command: dispense %d g (manual)", grams);

        /* Acknowledge by deleting the pending command before running workflow */
        char ack_path[128];
        snprintf(ack_path, sizeof(ack_path), "commands/%s/pending",
                 CONFIG_FIREBASE_DEVICE_ID);
        firebase_delete(ack_path);

        /* Run workflow directly — blocking the poll task is intentional
         * since overlapping feeds must not happen. */
        do_feed_workflow(grams, "manual");
    }
    /* Future commands: refill_alert_ack, firmware_update, etc. */
}

static void command_poll_task(void *arg)
{
    char path[128];
    snprintf(path, sizeof(path), "commands/%s/pending",
             CONFIG_FIREBASE_DEVICE_ID);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));   /* poll every 5 s */

        cJSON *cmd = NULL;
        esp_err_t err = firebase_get(path, &cmd);

        if (err == ESP_OK && cmd != NULL) {
            handle_pending_command(cmd);
            cJSON_Delete(cmd);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Heartbeat task  —  keeps online/lastSeen fresh in Firebase
 * ════════════════════════════════════════════════════════════════════════════ */

static void heartbeat_task(void *arg)
{
    while (true) {
        report_online();
        vTaskDelay(pdMS_TO_TICKS(30000));   /* every 30 s */
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Camera capture loop  (unchanged logic from original take_picture.c)
 * ════════════════════════════════════════════════════════════════════════════ */

static void camera_capture_task(void *arg)
{
    int consecutive_failures = 0;
    const int MAX_FAILURES = 3;

    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame) {
            consecutive_failures = 0;

            /* Discard corrupt frames */
            if (frame->len < 100 ||
                frame->buf[0] != 0xFF ||
                frame->buf[1] != 0xD8 ||
                frame->buf[frame->len - 2] != 0xFF ||
                frame->buf[frame->len - 1] != 0xD9) {
                ESP_LOGW(TAG, "Corrupt frame discarded");
                esp_camera_fb_return(frame);
                continue;
            }

            if (xQueueSend(xQueueIFrame, &frame, 0) != pdTRUE) {
                esp_camera_fb_return(frame);
            }
        } else {
            consecutive_failures++;
            ESP_LOGW(TAG, "Failed to get frame (%d/%d)",
                     consecutive_failures, MAX_FAILURES);

            if (consecutive_failures >= MAX_FAILURES) {
                consecutive_failures = 0;
                reinit_camera();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  app_main
 * ════════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Sitnchow Feeder booting ===");

    /* 1. WiFi */
    app_wifi_main();
    ESP_LOGI(TAG, "WiFi connected");

    /* 2. Time sync (needed for schedule matching) */
    setenv("TZ", "UTC0", 1);   /* Change to your timezone, e.g. "EST5EDT" */
    tzset();
    sntp_sync_wait();

    /* 3. Firebase — fetch token from Cloud Run */
    esp_err_t fb_err = firebase_client_init();
    if (fb_err != ESP_OK) {
        ESP_LOGE(TAG, "Firebase init failed — continuing without cloud features");
    }

    /* 4. Fetch owner UID */
    for (int retry = 0; retry < 5; retry++) {
        if (fetch_owner_uid()) {
            s_identity_ready = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    /* 5. Report online immediately */
    report_online();

    /* 6. Camera */
    xQueueIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    esp_err_t cam_err = init_camera(20000000, PIXFORMAT_JPEG, FRAMESIZE_HVGA, 2);
    ESP_LOGI(TAG, "Camera init: %s", esp_err_to_name(cam_err));

    /* 7. WebSocket stream */
    ws_client_init("wss://sit-n-chow-ws-5jph4zpsja-uc.a.run.app/ingest");

    /* 8. Feed mutex */
    s_feed_mutex = xSemaphoreCreateMutex();
    configASSERT(s_feed_mutex);

    /* 9. Schedule runner */
    if (s_identity_ready) {
        schedule_runner_start(CONFIG_FIREBASE_DEVICE_ID,
                              s_owner_uid,
                              do_feed_workflow);
    } else {
        ESP_LOGW(TAG, "Identity not ready — schedule runner skipped");
    }

    /* 10. Background tasks */
    xTaskCreate(heartbeat_task,      "heartbeat",  4096, NULL, 3, NULL);
    xTaskCreate(command_poll_task,   "cmd_poll",   8192, NULL, 4, NULL);
    xTaskCreate(ws_send_task,        "ws_send",   16384, NULL, 5, NULL);
    xTaskCreate(camera_capture_task, "cam_cap",    4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "All tasks started");
    /* app_main returns; FreeRTOS scheduler keeps everything running */
}
