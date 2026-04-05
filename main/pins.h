#pragma once

#include "driver/gpio.h"

/* ── Camera (OV5640) ────────────────────────────────────────────────────── */
#define CAMERA_PIN_PWDN         -1
#define CAMERA_PIN_RESET        -1
#define CAMERA_PIN_SIOD         GPIO_NUM_7
#define CAMERA_PIN_SIOC         GPIO_NUM_6

#define CAMERA_PIN_XCLK         GPIO_NUM_4
#define CAMERA_PIN_D7           GPIO_NUM_5
#define CAMERA_PIN_D6           GPIO_NUM_15
#define CAMERA_PIN_D5           GPIO_NUM_11
#define CAMERA_PIN_D4           GPIO_NUM_19
#define CAMERA_PIN_D3           GPIO_NUM_9
#define CAMERA_PIN_D2           GPIO_NUM_18
#define CAMERA_PIN_D1           GPIO_NUM_10
#define CAMERA_PIN_D0           GPIO_NUM_20
#define CAMERA_PIN_VSYNC        GPIO_NUM_17
#define CAMERA_PIN_HREF         GPIO_NUM_3
#define CAMERA_PIN_PCLK         GPIO_NUM_16

/* ── Ultrasonic (HC-SR04) ────────────────────────────────────────────────── */
#define PIN_ULTRASONIC_TRIG     GPIO_NUM_45
#define PIN_ULTRASONIC_ECHO     GPIO_NUM_35
 
/* ── Stepper motor (DRV8825) ─────────────────────────────────────────────── */
#define PIN_STEPPER_STEP        GPIO_NUM_1
#define PIN_STEPPER_DIR         GPIO_NUM_39
#define PIN_STEPPER_EN          GPIO_NUM_48
#define PIN_STEPPER_SLP         GPIO_NUM_2
#define PIN_STEPPER_RST         GPIO_NUM_42
#define PIN_STEPPER_M0          GPIO_NUM_14
#define PIN_STEPPER_M1          GPIO_NUM_38
#define PIN_STEPPER_M2          GPIO_NUM_41
 
/* ── ToF sensor (TMF8701) ────────────────────────────────────────────────── */
#define PIN_TOF_SDA             GPIO_NUM_37
#define PIN_TOF_SCL             GPIO_NUM_36
 
/* ── Speaker (LM386 via LEDC PWM) ───────────────────────────────────────── */
#define PIN_SPEAKER             GPIO_NUM_13
 
/* ── Manual override button ──────────────────────────────────────────────── */
#define PIN_BUTTON              GPIO_NUM_40   
