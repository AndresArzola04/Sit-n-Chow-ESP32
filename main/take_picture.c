/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "camera_pin.h"
#include "app_wifi.h"
#include "esp_camera.h"
#include "esp_websocket_client.h"

#define TEST_ESP_OK(ret) assert(ret == ESP_OK)
#define TEST_ASSERT_NOT_NULL(ret) assert(ret != NULL)

static bool auto_jpeg_support = false; // whether the camera sensor support compression or JPEG encode
static QueueHandle_t xQueueIFrame = NULL;

static const char *TAG = "video s_server";

static esp_websocket_client_handle_t ws_client = NULL;

void ws_client_init(const char *uri) {
    esp_websocket_client_config_t ws_cfg = {
        .uri = uri,
        .transport = WEBSOCKET_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .cert_pem = NULL,
        .disable_auto_reconnect = false,
        .buffer_size = 1024 * 32,
        .network_timeout_ms = 10000,
        .reconnect_timeout_ms = 5000,
    };
    ws_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_client_start(ws_client);
}

void ws_send_task(void *arg) {
    camera_fb_t *frame;
    while (true) {
        if (xQueueReceive(xQueueIFrame, &frame, portMAX_DELAY)) {
            if (esp_websocket_client_is_connected(ws_client)) {
                esp_websocket_client_send_bin(ws_client,
                    (const char *)frame->buf,
                    frame->len,
                    pdMS_TO_TICKS(5000));
            }
            esp_camera_fb_return(frame);
        }
    }
}

static esp_err_t init_camera(uint32_t xclk_freq_hz, pixformat_t pixel_format, framesize_t frame_size, uint8_t fb_count)
{
    camera_config_t camera_config = {
        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,
        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_sscb_sda = CAMERA_PIN_SIOD,
        .pin_sscb_scl = CAMERA_PIN_SIOC,

        .pin_d7 = CAMERA_PIN_D7,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d0 = CAMERA_PIN_D0,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_pclk = CAMERA_PIN_PCLK,

        //EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
        .xclk_freq_hz = xclk_freq_hz,
        .ledc_timer = LEDC_TIMER_0, 
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = pixel_format, //YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = frame_size,    //QQVGA-UXGA, sizes above QVGA are not been recommended when not JPEG format.

        .jpeg_quality = 25, //0-63
        .fb_count = fb_count,       // For ESP32/ESP32-S2, if more than one, i2s runs in continuous mode. Use only with JPEG.
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM
    };

    //initialize the camera
    esp_err_t ret = esp_camera_init(&camera_config);

    sensor_t *s = esp_camera_sensor_get();
    s->set_reg(s, 0x3035, 0xff, 0x21); // increase PLL clock divider

    if (s->id.PID == OV5640_PID) {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 0);
        // Optionally boost quality:
        s->set_quality(s, 10);  // 0-63, lower = better
    }

    camera_sensor_info_t *s_info = esp_camera_sensor_get_info(&(s->id));

    if (ESP_OK == ret && PIXFORMAT_JPEG == pixel_format && s_info->support_jpeg == true) {
        auto_jpeg_support = true;
    }

    return ret;
}

static esp_err_t reinit_camera() {
    ESP_LOGW(TAG, "Reinitializing camera...");
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t err = init_camera(20000000, PIXFORMAT_JPEG, FRAMESIZE_HVGA, 2);
    ESP_LOGI(TAG, "Camera reinit: %s", esp_err_to_name(err));
    return err;
}

void app_main()
{
    ESP_LOGI(TAG, "Starting app_main");
    app_wifi_main();
    ESP_LOGI(TAG, "WiFi done");

    xQueueIFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    esp_err_t err = init_camera(20000000, PIXFORMAT_JPEG, FRAMESIZE_HVGA, 2);
    ESP_LOGI(TAG, "Camera init returned: %s", esp_err_to_name(err));

    ws_client_init("wss://sit-n-chow-ws-96817124249.us-central1.run.app/ingest");
    ESP_LOGI(TAG, "Connected to websocket server");

    xTaskCreate(ws_send_task, "ws_send", 16384, NULL, 5, NULL);

    ESP_LOGI(TAG, "Begin capture frame");

    int consecutive_failures = 0;
    const int MAX_FAILURES = 3;

    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame) {
            consecutive_failures = 0;

            // Discard corrupt frames before queuing
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
            ESP_LOGW(TAG, "Failed to get frame (%d/%d)", consecutive_failures, MAX_FAILURES);

            if (consecutive_failures >= MAX_FAILURES) {
                consecutive_failures = 0;
                reinit_camera();
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}