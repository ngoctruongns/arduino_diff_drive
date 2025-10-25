#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>

/* Device configuration */

// If you want to enable a feature, uncomment the corresponding line
// #define USE_ULTRASONIC_SENSOR    // Use ultrasonic sensor HC-SR04
#define USE_LED_STRIP            // Use LED strip WS2812B


/* Define type */
typedef unsigned char UINT8;
typedef unsigned int UINT16;
typedef unsigned long UINT32;

/* Define print debug */
#define PRINT Serial.print
#define PRINTLN Serial.println

/* Pin define */

// Motor with module L298 and encoder
#define M1_ENCA_PIN         2
#define M1_ENCB_PIN         4
#define M1_PWM_PIN          6
#define M1_IN1_PIN          7
#define M1_IN2_PIN          8

#define M2_ENCA_PIN         3
#define M2_ENCB_PIN         5
#define M2_PWM_PIN          9
#define M2_IN1_PIN          10
#define M2_IN2_PIN          11

#define DISABLE_RUN_CM      15   // Distance will stop motor


// Sensor and LED
#define BUZZER_PIN          25
#define ULTRA_TRIG_PIN      22
#define ULTRA_ECHO_PIN      24
#define LED_RGB_PIN         12
#define NUMBER_LEDS         22 // Number of LEDs in strip
#define LED_DELAY_VAL       300 // Delay for LED effect in ms

/* Communication */
#define BLE_SERIAL          Serial3
#define BLE_BAUDRATE        9600
#define SERIAL_BAUDRATE     19200

/* Joystick */
#define JOYSTICK_MID_X  127       // Giá trị trung bình của joystick trục X
#define JOYSTICK_MID_Y  128       // Giá trị trung bình của joystick trục Y
#define JOYSTICK_MAX    128.0     // Giá trị max độ lệch của joystick
#define OMEGA_MAX       180       // Vận tốc góc tối đa (rad/s)

/* Define parameter */
#define CMD_TIME_OVER       1000       // time to stop motor if not receive control cmd (ms)
#define PWM_MAX             170        // Max PWM for all motor
#define PWM_MIN             60         // Min PWM for motor can rotary
#define PWM_STOP_THRESHOLD  10         // If pwm < threshold will be stop motor
#define PWM_STEP_NUM        10         // PWM step form MIN to MAX
#define ENCODER_PPR         330        // 330 pulse per rev

#endif  // _CONFIG_H_