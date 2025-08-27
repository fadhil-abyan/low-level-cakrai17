#pragma once
#include <Arduino.h>

// ================== XBOX Button Indices ==================
#define XBOX_A         0
#define XBOX_B         1
#define XBOX_X         2
#define XBOX_Y         3
#define XBOX_LB        4
#define XBOX_RB        5
#define XBOX_BACK      6
#define XBOX_START     7
#define XBOX_GUIDE     8
#define XBOX_LS        9
#define XBOX_RS        10

// ================== XBOX Axes Indices ==================
#define XBOX_AX_LX     0
#define XBOX_AX_LY     1
#define XBOX_AX_LT     2
#define XBOX_AX_RX     3
#define XBOX_AX_RY     4
#define XBOX_AX_RT     5
#define XBOX_AX_DX     6
#define XBOX_AX_DY     7

// ================== PS3 Button Indices ==================
#define PS3_CROSS      XBOX_A
#define PS3_CIRCLE     XBOX_B
#define PS3_SQUARE     XBOX_X
#define PS3_TRIANGLE   XBOX_Y
#define PS3_L1         XBOX_LB
#define PS3_R1         XBOX_RB
#define PS3_SELECT     XBOX_BACK
#define PS3_START      XBOX_START
#define PS3_PS         XBOX_GUIDE
#define PS3_L3         XBOX_LS
#define PS3_R3         XBOX_RS

// ================== PS3 Axes Indices ==================
#define PS3_AX_LX      XBOX_AX_LX
#define PS3_AX_LY      XBOX_AX_LY
#define PS3_AX_L2      XBOX_AX_LT
#define PS3_AX_RX      XBOX_AX_RX
#define PS3_AX_RY      XBOX_AX_RY
#define PS3_AX_R2      XBOX_AX_RT
#define PS3_AX_DX      XBOX_AX_DX
#define PS3_AX_DY      XBOX_AX_DY

// ================== Source Modes ==================
#define ESP32_JOY       0
#define MICROROS_JOY    1

// ================== Class ==================
#pragma once
#include <Arduino.h>

#define XBOX_BTN_COUNT 11
#define XBOX_AXES_COUNT 8

#define ESP32_JOY       0
#define MICROROS_JOY    1

class JoyInterface {
public:
    JoyInterface();

    void init();

    // For ESP32: update from uart (PS3 + ESP32)
    void readESP32Joy();

    // For micro-ROS: update from callback (Xbox/ROS2 /joy)
    void setFromJoyMsg(const int32_t *buttons, size_t btn_size, const float *axes, size_t axes_size);

    // Getters (always use Xbox-style indices or PS3 aliases)
    bool getButton(uint8_t btn_idx) const;
    float getAxis(uint8_t axis_idx) const;

private:
    bool buttons_[XBOX_BTN_COUNT];   // Xbox button order
    float axes_[XBOX_AXES_COUNT];    // Xbox axes order

    String serialBuffer_;
    void parseEsp32Line(const String& line);

    // Internal helpers for mapping PS3 to Xbox format
    void mapPs3ToXbox(const int32_t *ps3_buttons, const float *ps3_axe);
};