#pragma once

#include "driver/gpio.h"

/* ── Camera (OV5640) ────────────────────────────────────────────────────── */
#define CAMERA_PIN_PWDN         -1
#define CAMERA_PIN_RESET        -1
#define CAMERA_PIN_SIOD         GPIO_NUM_7
#define CAMERA_PIN_SIOC         GPIO_NUM_6

#define CAMERA_PIN_XCLK         GPIO_NUM_3
#define CAMERA_PIN_D7           GPIO_NUM_9
#define CAMERA_PIN_D6           GPIO_NUM_10
#define CAMERA_PIN_D5           GPIO_NUM_11
#define CAMERA_PIN_D4           GPIO_NUM_20
#define CAMERA_PIN_D3           GPIO_NUM_17
#define CAMERA_PIN_D2           GPIO_NUM_16
#define CAMERA_PIN_D1           GPIO_NUM_15
#define CAMERA_PIN_D0           GPIO_NUM_4
#define CAMERA_PIN_VSYNC        GPIO_NUM_19
#define CAMERA_PIN_HREF         GPIO_NUM_5
#define CAMERA_PIN_PCLK         GPIO_NUM_18

/* ── Ultrasonic (HC-SR04) ────────────────────────────────────────────────── */
#define PIN_ULTRASONIC_TRIG     GPIO_NUM_45
#define PIN_ULTRASONIC_ECHO     GPIO_NUM_41   // was 35 (PSRAM), now M2 pin

/* ── Stepper motor (DRV8825) ─────────────────────────────────────────────── */
#define PIN_STEPPER_STEP        GPIO_NUM_1
#define PIN_STEPPER_DIR         GPIO_NUM_39
#define PIN_STEPPER_EN          GPIO_NUM_48
#define PIN_STEPPER_SLP         GPIO_NUM_2
#define PIN_STEPPER_RST         GPIO_NUM_42
// M0, M1, M2 removed — motor hardwired to 1/8 step

/* ── ToF sensor (TMF8701) ────────────────────────────────────────────────── */
#define PIN_TOF_SDA             GPIO_NUM_14   // was 37 (PSRAM), now M0 pin
#define PIN_TOF_SCL             GPIO_NUM_38   // was 36 (PSRAM), now M1 pin
 
/* ── Speaker (LM386 via LEDC PWM) ───────────────────────────────────────── */
#define PIN_SPEAKER             GPIO_NUM_13
 
/* ── Manual override button ──────────────────────────────────────────────── */
#define PIN_BUTTON              GPIO_NUM_40   
