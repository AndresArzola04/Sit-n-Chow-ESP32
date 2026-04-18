/*
 * main.c  —  main application entry point
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

/*  Device IDs:     ESP Dev Board   SIT_N_CHOW_3533EC
 *                  Project Board   SIT_N_CHOW_D1191C
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

#include "pins.h"
#include "app_wifi.h"
#include "esp_camera.h"
#include "esp_websocket_client.h"
#include "firebase_client.h"

#include "firebase_client.h"
#include "schedule_runner.h"
#include "ultrasonic.h"
#include "dispenser.h"
#include "tof.h"
#include "speaker.h"
#include "manual_button.h"
#include "cJSON.h"
#include "audio_intercom.h"
#include "img_converters.h"
#include "esp_system.h"

#include "driver/i2c.h"


/* ── Build-time config (set in idf.py menuconfig) ───────────────────────── */
// firebase_client_get_device_id()   — unique string that identifies this feeder
// CONFIG_FIREBASE_DATABASE_URL
// CONFIG_FIREBASE_DB_SECRET

#define TAG "feeder"

/* ── Camera / WebSocket (unchanged from original) ────────────────────────── */

#define TEST_ESP_OK(ret)          assert(ret == ESP_OK)
#define TEST_ASSERT_NOT_NULL(ret) assert(ret != NULL)

static bool              auto_jpeg_support = false;
static QueueHandle_t     xQueueIFrame      = NULL;
static esp_websocket_client_handle_t ws_client = NULL;

// Device that is on sends "hello" and its deviceID when it connects to the websocket
static void ws_send_hello(void)
{
    if (!ws_client || !esp_websocket_client_is_connected(ws_client)) {
        return;
    }

    char hello[128];
    snprintf(hello, sizeof(hello),
             "{\"type\":\"hello\",\"deviceId\":\"%s\"}",
             firebase_client_get_device_id());

    int ret = esp_websocket_client_send_text(
        ws_client,
        hello,
        strlen(hello),
        pdMS_TO_TICKS(3000)
    );

    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to send websocket hello");
    } else {
        ESP_LOGI(TAG, "Sent websocket hello for device %s", firebase_client_get_device_id());
    }
}

/* ── Device identity (fetched from Firebase at boot) ─────────────────────── */

static char s_owner_uid[64] = {0};     /* fetched from devices/<id>/ownerUid */
static bool s_identity_ready = false;

/* ── Feed workflow mutex (prevent concurrent feeds) ──────────────────────── */
static SemaphoreHandle_t s_feed_mutex = NULL;

// Static task control blocks — placed in .bss at link time, bypass heap
static StaticTask_t s_ws_task_tcb;
static StaticTask_t s_cap_task_tcb;

static StaticTask_t s_cmd_poll_tcb;
static StaticTask_t s_heartbeat_tcb;

static StaticTask_t s_cam_poll_tcb;

/* ════════════════════════════════════════════════════════════════════════════
 *  Camera / WebSocket helpers  (unchanged from original take_picture.c)
 * ════════════════════════════════════════════════════════════════════════════ */

static volatile bool s_camera_streaming = false;
static SemaphoreHandle_t s_camera_mutex = NULL;

void camera_streaming_start(void)
{
    if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_camera_streaming = true;
        xSemaphoreGive(s_camera_mutex);
        ESP_LOGI(TAG, "Camera streaming started");
    }
}

void camera_streaming_stop(void)
{
    if (xSemaphoreTake(s_camera_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_camera_streaming = false;
        xSemaphoreGive(s_camera_mutex);
        ESP_LOGI(TAG, "Camera streaming stopped");
    }
}

// Added ws_send_hello function
static void camera_ws_event_handler(void *handler_args, esp_event_base_t base,
                                     int32_t event_id, void *event_data)
{
    if (event_id == WEBSOCKET_EVENT_CONNECTED){
        ESP_LOGI(TAG, "Camera WebSocket connected to /ingest");
        ws_send_hello();
    }
    else if (event_id == WEBSOCKET_EVENT_DISCONNECTED)
        ESP_LOGW(TAG, "Camera WebSocket disconnected");
    else if (event_id == WEBSOCKET_EVENT_ERROR)
        ESP_LOGE(TAG, "Camera WebSocket error");
}

void ws_client_init(const char *uri)
{
    esp_websocket_client_config_t ws_cfg = {
        .uri                         = uri,
        .transport                   = WEBSOCKET_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .cert_pem                    = NULL,
        .disable_auto_reconnect      = false,
        .buffer_size                 = 1024 * 32,
        .network_timeout_ms          = 10000,
        .reconnect_timeout_ms        = 5000,
    };
    ws_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(ws_client, WEBSOCKET_EVENT_ANY,
                                   camera_ws_event_handler, NULL);
    esp_websocket_client_start(ws_client);
}

void ws_send_task(void *arg)
{
    camera_fb_t *frame;
    int sent = 0;

    ESP_LOGI(TAG, "ws_send_task started, free heap: %lu", esp_get_free_heap_size());

    while (true) {
        if (xQueueReceive(xQueueIFrame, &frame, pdMS_TO_TICKS(2000))) {

            uint8_t *jpg_buf = NULL;
            size_t   jpg_len = 0;
            bool converted = false;

            if (frame->format == PIXFORMAT_JPEG) {
                jpg_buf = frame->buf;
                jpg_len = frame->len;
            } else {
                converted = frame2jpg(frame, 30, &jpg_buf, &jpg_len); // Change camera quality here
                if (!converted) {
                    ESP_LOGE(TAG, "frame2jpg FAILED, free heap: %lu",
                             esp_get_free_heap_size());
                    esp_camera_fb_return(frame);
                    continue;
                }
            }

            if (esp_websocket_client_is_connected(ws_client)) {
                int result = esp_websocket_client_send_bin(ws_client,
                    (const char *)jpg_buf, jpg_len,
                    pdMS_TO_TICKS(5000));
                if (result < 0) {
                    ESP_LOGE(TAG, "WebSocket send failed: %d", result);
                } else {
                    if (++sent % 25 == 0)
                        ESP_LOGI(TAG, "Sent %d frames", sent);
                }
            } else {
                ESP_LOGW(TAG, "WS not connected");
            }

            if (converted) free(jpg_buf);
            esp_camera_fb_return(frame);

        } else {
            ESP_LOGW(TAG, "No frame received in 2s — queue empty?");
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
        .ledc_timer    = LEDC_TIMER_1,    // TIMER_1 to avoid audio conflict
        .ledc_channel  = LEDC_CHANNEL_1,  // CHANNEL_1 to avoid audio conflict
        .pixel_format  = pixel_format,
        .frame_size    = frame_size,
        .jpeg_quality  = 12,
        .fb_count      = fb_count,
        .grab_mode     = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location   = CAMERA_FB_IN_PSRAM,
    };

    esp_err_t ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) {
        ESP_LOGE(TAG, "Camera sensor get returned NULL");
        return ESP_FAIL;
    }

    s->set_vflip(s, 1);
    s->set_lenc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_aec2(s, 1);

    camera_sensor_info_t *s_info = esp_camera_sensor_get_info(&(s->id));
    if (s_info && pixel_format == PIXFORMAT_JPEG && s_info->support_jpeg) {
        auto_jpeg_support = true;
    }

    return ret;
}

static esp_err_t reinit_camera(void)
{
    ESP_LOGW(TAG, "Reinitialising camera…");
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t err = init_camera(20000000, PIXFORMAT_JPEG, FRAMESIZE_SVGA, 2);
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
    snprintf(path, sizeof(path), "devices/%s/status", firebase_client_get_device_id());
    firebase_patch(path, status);
    cJSON_Delete(status);
}

/** Fetch ownerUid from devices/<id> so the schedule runner knows which user. */
static bool fetch_owner_uid(void)
{
    char path[128];
    snprintf(path, sizeof(path), "devices/%s/ownerUid", firebase_client_get_device_id());

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
    snprintf(path, sizeof(path), "feedEvents/%s", firebase_client_get_device_id());
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
    cJSON_AddStringToObject(notif, "deviceId", firebase_client_get_device_id());

    char path[128];
    snprintf(path, sizeof(path), "notifications/%s", s_owner_uid);
    firebase_push(path, notif);
    cJSON_Delete(notif);
}

/**
 * Blocks until the ML service writes status="finished" to Firebase,
 * or until timeout_ms elapses.
 *
 * Returns true  if the ML result indicates sitting was confirmed.
 * Returns false if sitting was not confirmed OR timeout was reached.
 *
 * The workflow continues either way — we never starve the pet.
 */
static bool sitting_detector_run(uint32_t timeout_ms)
{
    const uint32_t POLL_INTERVAL_MS = 500;
    uint32_t       elapsed_ms       = 0;

    char updated_at_path[128];
    snprintf(updated_at_path, sizeof(updated_at_path),
             "final_output/%s/updatedAt", firebase_client_get_device_id());

    char posture_path[128];
    snprintf(posture_path, sizeof(posture_path),
             "final_output/%s/posture", firebase_client_get_device_id());

    /* ── Snapshot the current updatedAt value before ML starts ────────── */
    double baseline_updated_at = 0.0;
    cJSON *baseline_val = NULL;
    esp_err_t err = firebase_get(updated_at_path, &baseline_val);
    if (err == ESP_OK && baseline_val != NULL && cJSON_IsNumber(baseline_val)) {
        baseline_updated_at = baseline_val->valuedouble;
        ESP_LOGI(TAG, "[sitting_detector] Baseline updatedAt=%.0f", baseline_updated_at);
    } else {
        ESP_LOGW(TAG, "[sitting_detector] Could not read baseline updatedAt — will trigger on any value");
    }
    if (baseline_val) cJSON_Delete(baseline_val);

    ESP_LOGI(TAG, "[sitting_detector] Waiting for updatedAt to change (timeout=%lu ms)…", timeout_ms);

    /* ── Poll until updatedAt changes ─────────────────────────────────── */
    while (elapsed_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
        elapsed_ms += POLL_INTERVAL_MS;

        cJSON *current_val = NULL;
        err = firebase_get(updated_at_path, &current_val);

        if (err != ESP_OK || current_val == NULL || !cJSON_IsNumber(current_val)) {
            if (current_val) cJSON_Delete(current_val);
            continue;
        }

        double current_updated_at = current_val->valuedouble;
        cJSON_Delete(current_val);

        if (current_updated_at <= baseline_updated_at) {
            continue;   /* No change yet */
        }

        /* ── updatedAt changed — ML result is ready ───────────────────── */
        ESP_LOGI(TAG, "[sitting_detector] updatedAt changed (%.0f → %.0f) after %lu ms",
                 baseline_updated_at, current_updated_at, elapsed_ms);

        cJSON *posture_val = NULL;
        err = firebase_get(posture_path, &posture_val);

        bool sitting = false;
        const char *posture_str = "unknown";

        if (err == ESP_OK && posture_val != NULL && cJSON_IsString(posture_val)) {
            posture_str = posture_val->valuestring;
            sitting = (strcmp(posture_str, "sitting") == 0);
        }

        ESP_LOGI(TAG, "[sitting_detector] posture=\"%s\" → sitting=%s",
                 posture_str, sitting ? "YES" : "NO");

        if (posture_val) cJSON_Delete(posture_val);
        return sitting;
    }

    ESP_LOGW(TAG, "[sitting_detector] Timed out after %lu ms — proceeding anyway", timeout_ms);
    return false;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Feed workflow
 * ════════════════════════════════════════════════════════════════════════════
 *
 *  Steps:
 *   1. Beep speaker            
 *   2. Activate ToF sensor     
 *   3. Wait for pet approach   
 *   4. Run sitting detector    (TODO: ML inference)
 *   5. Dispense food           
 *   6. Ultrasonic capacity check 
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
    speaker_beep();

    /* ── Step 2 & 3: ToF — wait for pet to approach ─────────────────────── */
    ESP_LOGI(TAG, "[2] Activating ToF sensor, waiting for pet…");
    uint32_t tof_timeout_ms = (uint32_t)CONFIG_PET_APPROACH_TIMEOUT_S * 1000;
    bool pet_detected = tof_wait_for_presence(CONFIG_TOF_THRESHOLD_MM, tof_timeout_ms);

    // if (!pet_detected) {
    //     ESP_LOGW(TAG, "No pet detected within timeout — aborting feed");
    //     xSemaphoreGive(s_feed_mutex);
    //     return;
    // }

    /* ── Step 4: Camera sitting detection ────────────────────────────────── */
    ESP_LOGI(TAG, "[3] Running sitting detector…");
    camera_streaming_start();  // turn on for ML
    
    // bool sitting_confirmed = sitting_detector_run(
    //     (uint32_t)CONFIG_SITTING_DETECT_TIMEOUT_S * 1000   /* e.g. 180 s */
    // );

    bool sitting_confirmed = false;


    camera_streaming_stop();

    if (!sitting_confirmed) {
        ESP_LOGW(TAG, "Pet not confirmed sitting — dispensing anyway");
    }

    /* ── Step 5: Dispense ────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "[4] Dispensing %d g…", grams);
    dispenser_run(grams);

    /* ── Step 6: Ultrasonic food level check ─────────────────────────────── */
    ESP_LOGI(TAG, "[5] Checking food level…");
    float distance_cm = ultrasonic_read_cm();
    bool food_low = false;
    if (distance_cm < 0) {
        ESP_LOGW(TAG, "Ultrasonic read failed — skipping low food check");
    } else {
        ESP_LOGI(TAG, "Food level distance: %.1f cm (threshold: %d cm)",
                 distance_cm, CONFIG_LOW_FOOD_THRESHOLD_CM);
        food_low = (distance_cm > (float)CONFIG_LOW_FOOD_THRESHOLD_CM);
    }

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
                 firebase_client_get_device_id());
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
                 firebase_client_get_device_id());
        firebase_delete(ack_path);

        /* Run workflow directly — blocking the poll task is intentional
         * since overlapping feeds must not happen. */
        do_feed_workflow(grams, "manual");
    }
    /* Future commands: refill_alert_ack, firmware_update, etc. */
}

static void command_poll_task(void *arg)
{   
    vTaskDelay(pdMS_TO_TICKS(2500));
    
    char path[128];
    snprintf(path, sizeof(path), "commands/%s/pending",
             firebase_client_get_device_id());

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG, "Polling commands...");
        cJSON *cmd = NULL;
        esp_err_t err = firebase_get(path, &cmd);
        ESP_LOGI(TAG, "Command poll result: %s, cmd=%s",
                 esp_err_to_name(err),
                 cmd ? cJSON_PrintUnformatted(cmd) : "NULL");

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
        if (!s_camera_streaming) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        camera_fb_t *frame = esp_camera_fb_get();
        if (frame) {
            consecutive_failures = 0;

            if (frame->len < 100) {
                esp_camera_fb_return(frame);
                continue;
            }

            if (xQueueSend(xQueueIFrame, &frame, 0) != pdTRUE) {
                esp_camera_fb_return(frame);
            }
        } else {
            consecutive_failures++;
            if (consecutive_failures >= MAX_FAILURES) {
                consecutive_failures = 0;
                reinit_camera();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void camera_poll_task(void *arg)
{
    char path[128];
    snprintf(path, sizeof(path), "devices/%s/cameraActive",
             firebase_client_get_device_id());

    bool last_state = false;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        cJSON *val = NULL;
        // Uses dedicated mutex — never blocks command_poll_task
        esp_err_t err = firebase_get_camera_poll(path, &val);

        if (err == ESP_OK && val != NULL) {
            bool active = cJSON_IsTrue(val);
            if (active != last_state) {
                last_state = active;
                if (active) camera_streaming_start();
                else        camera_streaming_stop();
            }
            cJSON_Delete(val);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  app_main
 * ════════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    /* ── Silence LM386 input immediately ─────────────────────────────────
     * GPIO13 floats from reset until speaker_init() ~15s later.
     * A floating input gets amplified as noise. Drive it LOW now. */
    gpio_set_direction(PIN_SPEAKER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SPEAKER, 0);
    
    ESP_LOGI(TAG, "=== Sitnchow Feeder booting ===");

    /* 1. WiFi */
    app_wifi_main();
    ESP_LOGI(TAG, "WiFi connected");

    /* 2. Time sync (needed for schedule matching) */
    setenv("TZ", "EST5EDT", 1);   /* Change to your timezone, e.g. "EST5EDT" */
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
    xQueueIFrame = xQueueCreate(4, sizeof(camera_fb_t *));
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t cam_err = init_camera(20000000, PIXFORMAT_JPEG, FRAMESIZE_SVGA, 2);
    ESP_LOGI(TAG, "Camera init: %s", esp_err_to_name(cam_err));

    /* 7. WebSocket stream (only if camera working) */
    const bool camera_ok = (cam_err == ESP_OK);
    if (camera_ok) {
        ws_client_init(CONFIG_WS_INGEST_URL); // Changed to new websocket url
        ESP_LOGI(TAG, "Camera WebSocket stream connected");
    } else {
        ESP_LOGW(TAG, "Camera not available — skipping WebSocket stream");
    }

    /* 7b. Sensors & actuators */
    speaker_init();
    vTaskDelay(pdMS_TO_TICKS(2000));   // let WiFi/Firebase settle
    audio_intercom_start(firebase_client_get_device_id());
    vTaskDelay(pdMS_TO_TICKS(500));
    ultrasonic_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    dispenser_init();

    esp_err_t tof_err = tof_init();
    if (tof_err != ESP_OK) {
        ESP_LOGW(TAG, "ToF sensor not detected — presence detection disabled");
    }

    

    /* 8. Feed mutex */
    s_feed_mutex = xSemaphoreCreateMutex();
    configASSERT(s_feed_mutex);

    s_camera_mutex = xSemaphoreCreateMutex();
    configASSERT(s_camera_mutex);

    /* 9. Schedule runner */
    if (s_identity_ready) {
        schedule_runner_start(firebase_client_get_device_id(),
                              s_owner_uid,
                              do_feed_workflow);
    } else {
        ESP_LOGW(TAG, "Identity not ready — schedule runner skipped");
    }

    /* 9b. Manual override button */
    manual_button_start();

    /* 10. Background tasks */
    vTaskDelay(pdMS_TO_TICKS(3000));  // give BLE time to release BTDM memory
    StackType_t *heartbeat_stack = heap_caps_malloc(
        8192 * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    StackType_t *cmd_poll_stack = heap_caps_malloc(
        8192 * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (heartbeat_stack) {
        xTaskCreateStaticPinnedToCore(heartbeat_task, "heartbeat",
            8192, NULL, 3, heartbeat_stack, &s_heartbeat_tcb, 0);
        ESP_LOGI(TAG, "heartbeat_task created");
    } else {
        ESP_LOGE(TAG, "heartbeat_task stack alloc failed");
    }

    if (cmd_poll_stack) {
        xTaskCreateStaticPinnedToCore(command_poll_task, "cmd_poll",
            8192, NULL, 4, cmd_poll_stack, &s_cmd_poll_tcb, 0);
        ESP_LOGI(TAG, "command_poll_task created");
    } else {
        ESP_LOGE(TAG, "command_poll_task stack alloc failed");
    }
    if (camera_ok) {    
        StackType_t *ws_stack = heap_caps_malloc(
            16384 * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        StackType_t *cap_stack = heap_caps_malloc(
            16384 * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        if (ws_stack) {
            xTaskCreateStaticPinnedToCore(ws_send_task, "ws_send",
                16384, NULL, 5, ws_stack, &s_ws_task_tcb, 0);
            ESP_LOGI(TAG, "ws_send_task created");
        } else {
            ESP_LOGE(TAG, "ws_send_task stack alloc failed");
        }

        if (cap_stack) {
            xTaskCreateStaticPinnedToCore(camera_capture_task, "cam_cap",
                16384, NULL, 5, cap_stack, &s_cap_task_tcb, 0);
            ESP_LOGI(TAG, "camera_capture_task created");
        } else {
            ESP_LOGE(TAG, "camera_capture_task stack alloc failed");
        }
    }

    StackType_t *cam_poll_stack = heap_caps_malloc(
        8192 * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (cam_poll_stack) {
        xTaskCreateStaticPinnedToCore(camera_poll_task, "cam_poll",
            8192, NULL, 3, cam_poll_stack, &s_cam_poll_tcb, 0);
        ESP_LOGI(TAG, "camera_poll_task created");
    } else {
        ESP_LOGE(TAG, "camera_poll_task stack alloc failed");
    }

    ESP_LOGI(TAG, "All tasks started");
    /* app_main returns; FreeRTOS scheduler keeps everything running */
}