#ifndef BASE_R1_CONFIG_H
#define BASE_R1_CONFIG_H

#include "ESP32BoardV1Pin.h"

// ================== Pin Configuration ==================
// Motor Pins (FL, FR, BL, BR)
#define MOTOR_ELEVATION_EN     ESP32_MTR3_EN
#define MOTOR_ELEVATION_FWD    ESP32_MTR3_FWD
#define MOTOR_ELEVATION_REV    ESP32_MTR3_REV

#define MOTOR_CATCHER_EN     ESP32_MTR0_EN
#define MOTOR_CATCHER_FWD    ESP32_MTR0_FWD
#define MOTOR_CATCHER_REV    ESP32_MTR0_REV

#define MOTOR_LAUNCHER_EN     ESP32_MTR2_EN
#define MOTOR_LAUNCHER_FWD    ESP32_MTR2_REV
#define MOTOR_LAUNCHER_REV    ESP32_MTR2_FWD

// Internal Encoder Pins (FL, FR, BL, BR)
#define ENCODER_LAUNCHER_A    ESP32_ENC1_A
#define ENCODER_LAUNCHER_B    ESP32_ENC1_B
#define ENCODER_CATCHER_A    ESP32_ENC0_A
#define ENCODER_CATCHER_B    ESP32_ENC0_B
#define ENCODER_ELEVATION_A    ESP32_ENC2_A
#define ENCODER_ELEVATION_B    ESP32_ENC2_B

#define ENCODER_SHOOTER_A ESP32_ENC3_A
#define ENCODER_SHOOTER_B ESP32_ENC3_B

#define FALCON_KIRI 8
#define FALCON_KANAN 9

// ================== Encoder Configuration ==================
#define ENCODER_SPEC_PPR_LAUNCHER     7
#define ENCODER_SPEC_PPR_CATCHER      7
#define ENCODER_SPEC_PPR_ELEVATION    7

#define ENCODER_SPEC_GEAR_LAUNCHER    19.2f
#define ENCODER_SPEC_GEAR_CATCHER     19.2f
#define ENCODER_SPEC_GEAR_ELEVATION   19.2f

#define ENCODER_PPR_LAUNCHER          (ENCODER_SPEC_PPR_LAUNCHER * ENCODER_SPEC_GEAR_LAUNCHER) * 4.0f
#define ENCODER_PPR_CATCHER           (ENCODER_SPEC_PPR_CATCHER * ENCODER_SPEC_GEAR_CATCHER) * 4.0f
#define ENCODER_PPR_ELEVATION         (ENCODER_SPEC_PPR_ELEVATION * ENCODER_SPEC_GEAR_ELEVATION) * 4.0f

#define WHEEL_DIAMETER      0.1f
#define BAUDRATE            921600

// ================== PID Parameters ==================
#define MAX_VOLTAGE         24.0f
#define PID_KP_LAUNCHER     62.0f
#define PID_KI_LAUNCHER     28.0f
#define PID_KD_LAUNCHER     0.7f
#define PID_MAX_I_OUTPUT_LAUNCHER 1000.0f

#define PID_KP_CATCHER      62.0f
#define PID_KI_CATCHER      28.0f
#define PID_KD_CATCHER      0.7f
#define PID_MAX_I_OUTPUT_CATCHER 1000.0f

#define PID_KP_ELEVATION    62.0f
#define PID_KI_ELEVATION    28.0f
#define PID_KD_ELEVATION    0.7f
#define PID_MAX_I_OUTPUT_ELEVATION 1000.0f

// ================== MicroROS Configuration ==================
// Node name
#define NODE_NAME           "esp32_mekanisme_r1"

// Topic names
#define VEL_COMMAND_TOPIC   "/hw_command/vel"
#define ENC_INT_PUB_TOPIC   "/hw_state/enc_int"

#define AUTO_RESET          1 // Auto reset if MicroROS fails

// =================== Miscellaneous Configuration ===================
// Timing
#define TS                  10 // ms (Time Sampling for Motor Control and Encoder Updates)
#define TIMEOUT_MS          1000 // ms (Timeout for MicroROS commands)

// LED Status Colors
#define LED_COLOR_IDLE      0xFFFF00    // Yellow - Idle state
#define LED_COLOR_WAITING   0x800080    // Purple - Waiting for commands
#define LED_COLOR_ACTIVE    0x00FF00    // Green - Receiving commands
#define LED_COLOR_ERROR     0xFF0000    // Red - Error

#endif