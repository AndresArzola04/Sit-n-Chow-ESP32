/*
 * manual_button.h
 *
 * Physical manual override button for the Sit-N-Chow feeder.
 *
 * Behavior:
 *   - While held:   motor runs continuously at dispense speed
 *   - On release:   motor completes the current revolution cleanly then stops
 *
 * This provides a hardware fallback for feeding the pet if the mobile
 * app is unavailable (no WiFi, dead phone, etc.).
 *
 * GPIO: 13  (active low, internal pull-up)
 *
 * Note: GPIO 13 is shared with the speaker LEDC output. The button
 * task only reads the pin as an input — LEDC only drives it as output
 * during audio playback. These don't conflict because LEDC output is
 * only active during beep/audio, not during button monitoring.
 * If GPIO conflicts arise in testing, reassign BUTTON_GPIO to a free pin.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the manual override button monitor task.
 *        Must be called after dispenser_init().
 *        Runs a background FreeRTOS task that watches the button indefinitely.
 */
void manual_button_start(void);

#ifdef __cplusplus
}
#endif
