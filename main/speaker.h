/*
 * speaker.h
 *
 * Speaker module wrapping audio_player for the Sit-N-Chow feeder.
 *
 * Currently provides:
 *   - speaker_init()     — initialise LEDC PWM output
 *   - speaker_beep()     — play a short attention beep at feed time
 *
 * Future:
 *   - speaker_play_pcm() — play arbitrary PCM audio streamed from the app
 *
 * Hardware:
 *   LM386 analog amplifier
 *   GPIO: 13  (matches tested audio_player configuration)
 *   PWM carrier: 20 kHz
 *   Sample rate: 16 kHz
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise LEDC PWM output for the speaker.
 *        Must be called once before any other speaker function.
 */
void speaker_init(void);

/**
 * @brief Play a short attention beep to signal feed time.
 *
 * Plays two short 880 Hz tones with a brief gap — audible to dogs
 * and distinct from background noise.
 * Blocks until the beep is complete (~600 ms total).
 */
void speaker_beep(void);

/**
 * @brief Play raw 16-bit signed PCM audio.
 *        Non-blocking — spawns a playback task and returns immediately.
 *        Used for future app-to-speaker audio streaming.
 *
 * @param samples      Pointer to int16_t PCM sample array.
 * @param sample_count Number of samples (at 16 kHz sample rate).
 */
void speaker_play_pcm(const int16_t *samples, uint32_t sample_count);

/**
 * @brief Stop any currently playing audio immediately.
 */
void speaker_stop(void);

/**
 * @brief Returns true if audio is currently playing.
 */
bool speaker_is_playing(void);

#ifdef __cplusplus
}
#endif
