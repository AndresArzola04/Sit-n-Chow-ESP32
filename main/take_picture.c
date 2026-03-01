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

#define CAMERA_PIN_PWDN 0

#define TEST_ESP_OK(ret) assert(ret == ESP_OK)
#define TEST_ASSERT_NOT_NULL(ret) assert(ret != NULL)

static bool auto_jpeg_support = false; // whether the camera sensor support compression or JPEG encode
static QueueHandle_t xQueueIFrame = NULL;

static const char *TAG = "video s_server";

static esp_websocket_client_handle_t ws_client = NULL;

void ws_client_init(const char *uri) {
    esp_websocket_client_config_t ws_cfg = {
        .uri = "wss://sit-n-chow-ws-96817124249.us-central1.run.app/ingest",
        .transport = WEBSOCKET_TRANSPORT_OVER_SSL,
    };
    ws_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_client_start(ws_client);
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

        .jpeg_quality = 10, //0-63
        .fb_count = fb_count,       // For ESP32/ESP32-S2, if more than one, i2s runs in continuous mode. Use only with JPEG.
        .grab_mode = CAMERA_GRAB_LATEST,
        .fb_location = CAMERA_FB_IN_PSRAM
    };

    //initialize the camera
    esp_err_t ret = esp_camera_init(&camera_config);

    sensor_t *s = esp_camera_sensor_get();

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

void app_main()
{
    app_wifi_main();

    xQueueIFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    /* It is recommended to use a camera sensor with JPEG compression to maximize the speed */
    TEST_ESP_OK(init_camera(20000000, PIXFORMAT_JPEG, FRAMESIZE_VGA, 2));

    TEST_ESP_OK(ws_client_init("ws://YOUR_SERVER_IP:YOUR_PORT"););

    ESP_LOGI(TAG, "Begin capture frame");

    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame) {
            // Non-blocking send; if no one is listening, just drop frame
            if (esp_websocket_client_is_connected(ws_client)) {
                esp_websocket_client_send_bin(ws_client, (const char *)frame->buf, frame->len, portMAX_DELAY);
                ESP_LOGI(TAG, "Frame captured and queued");
            } else {
                ESP_LOGW(TAG, "Frame queue full, dropping frame");
            }

            // Producer always returns frame when done with it
            esp_camera_fb_return(frame);
        } else {
            ESP_LOGW(TAG, "Failed to get frame");
        }
        vTaskDelay(pdMS_TO_TICKS(33));  // ~30fps cap
    }
}
