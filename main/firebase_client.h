/*
 * firebase_client.h
 *
 * Thin wrapper around esp_http_client for reading/writing to
 * Firebase Realtime Database via its REST API.
 *
 * Authentication: fetches a short-lived custom token from your Cloud Run
 * /esp-token endpoint at boot and refreshes it every 55 minutes.
 * Tokens expire after 60 minutes — no database secret needed on the device.
 *
 * All paths are relative to the database root, e.g.
 *   "devices/MY_ID/commands/pending"
 */

#pragma once

#include "esp_err.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the Firebase client and fetch the first auth token
 *        from your Cloud Run /esp-token endpoint.
 *        Must be called once after WiFi is connected.
 *        Blocks until a token is obtained (retries up to 5x).
 *
 * @return ESP_OK if a token was obtained, ESP_FAIL otherwise.
 */
esp_err_t firebase_client_init(void);

/**
 * @brief Return the device ID generated at init time.
 *
 *        Format: "SIT_N_CHOW_AABBCC" where AABBCC are the last three
 *        octets of the WiFi station MAC address in upper-case hex.
 *
 *        Valid only after firebase_client_init() returns ESP_OK.
 *        The returned pointer is to a static buffer — do not free it.
 *
 * @return Pointer to null-terminated device ID string.
 */
const char *firebase_client_get_device_id(void);

/**
 * @brief HTTP GET a path from RTDB.
 *
 * @param path      RTDB path (e.g. "devices/abc/commands/pending")
 * @param out_json  Caller-owned cJSON* written on success; caller must
 *                  cJSON_Delete() it. Set to NULL on error / null node.
 * @return ESP_OK on success (even if the node is JSON null).
 */
esp_err_t firebase_get(const char *path, cJSON **out_json);

/**
 * @brief HTTP PATCH (merge) a JSON object into a RTDB path.
 */
esp_err_t firebase_patch(const char *path, cJSON *json);

/**
 * @brief HTTP PUT (overwrite) a RTDB path with a JSON value.
 */
esp_err_t firebase_put(const char *path, cJSON *json);

/**
 * @brief HTTP POST (push) a JSON object under a RTDB path.
 *        Firebase generates a unique push key.
 */
esp_err_t firebase_push(const char *path, cJSON *json);

/**
 * @brief HTTP DELETE a RTDB path (sets it to null).
 */
esp_err_t firebase_delete(const char *path);

esp_err_t firebase_get_intercom(const char *path, cJSON **out_json);

esp_err_t firebase_get_camera_poll(const char *path, cJSON **out_json);

#ifdef __cplusplus
}
#endif
