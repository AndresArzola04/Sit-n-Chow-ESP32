/*
 * audio_intercom.h
 *
 * Live audio intercom module for the Sit-N-Chow feeder.
 *
 * Polls devices/<deviceId>/audio in Firebase RTDB every second.
 * When a new chunk arrives (detected by chunkIndex change), it:
 *   1. Base64-decodes the raw PCM data field
 *   2. Passes the int16_t buffer to speaker_play_pcm()
 *   3. Does NOT delete the node — the Flutter app clears it when mic is toggled off
 *
 * The WAV header is stripped on the Flutter side before base64 encoding,
 * so the data field contains raw 16-bit signed PCM at 16 kHz mono.
 *
 * RTDB node structure written by the Flutter app:
 * {
 *   "command":    "play",
 *   "data":       "<base64 encoded raw PCM>",
 *   "sampleRate": 16000,
 *   "chunkIndex": <int>,   // increments each chunk — used to detect new audio
 *   "ts":         <ms>
 * }
 *
 * When the Flutter app toggles mic off it deletes the node entirely.
 * The intercom task detects the missing node and stops playing.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the audio intercom background task.
 *        Must be called after firebase_client_init() and speaker_init().
 *
 * @param device_id   Firebase device ID (e.g. "SIT_N_CHOW_AABBCC")
 */
void audio_intercom_start(const char *device_id);

#ifdef __cplusplus
}
#endif
