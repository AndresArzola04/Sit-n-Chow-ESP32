#pragma once

#include <stdint.h>
#include <stdbool.h>

// Initialize LEDC peripheral for PWM audio output
// Call once before audio_player_start()
void audio_player_init(int gpio_pin, uint32_t pwm_freq_hz, uint32_t sample_rate_hz);

// Start playback of a 16-bit signed PCM audio buffer
// Spawns a FreeRTOS task that plays through the buffer and self-deletes when done
// samples     : pointer to int16_t sample array
// sample_count: total number of samples
void audio_player_start(const int16_t* samples, uint32_t sample_count);

// Returns true if audio is currently playing
bool audio_player_is_playing(void);

// Stop playback immediately and silence output
void audio_player_stop(void);
