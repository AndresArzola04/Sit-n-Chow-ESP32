#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ── Initialisation ──────────────────────────────────────────────────────── */

// Initialize LEDC peripheral for PWM audio output.
// Call once before any other audio_player function.
void audio_player_init(int gpio_pin, uint32_t pwm_freq_hz, uint32_t sample_rate_hz);

/* ── Beep / one-shot playback (non-blocking, semaphore-driven) ───────────── */

// Start playback of a PCM buffer in a background task.
// Returns immediately. Use audio_player_is_playing() to poll for completion.
// Used by speaker_beep() — not for streaming.
void audio_player_start(const int16_t *samples, uint32_t sample_count);

bool audio_player_is_playing(void);

// Stop any active playback immediately and silence output.
void audio_player_stop(void);

/* ── Streaming playback (synchronous, zero-gap) ──────────────────────────── */

// Play samples synchronously in the calling task's context.
// Blocks until all samples are played or audio_player_stop() is called.
// The calling task MUST be pinned to CPU1 at high priority.
// Used by audio_intercom for gapless chunk-to-chunk streaming.
void audio_player_play_sync(const int16_t *samples, uint32_t sample_count);

// Unmute/mute the LEDC output without affecting playback state.
// Call unmute once before the first streaming chunk, mute after the last.
void audio_player_unmute_output(void);
void audio_player_mute_output(void);