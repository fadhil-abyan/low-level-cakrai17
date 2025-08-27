#include "JoyInterface.h"

// Map PS3 button indices to Xbox indices
static const uint8_t ps3_to_xbox_btn[17] = {
    XBOX_BACK,    // 0: Select
    XBOX_LS,      // 1: L3
    XBOX_RS,      // 2: R3
    XBOX_START,   // 3: Start
    255,          // 4: D-pad Up (handled via axes)
    255,          // 5: D-pad Right
    255,          // 6: D-pad Down
    255,          // 7: D-pad Left
    255,          // 8: L2 (handled via axes)
    255,          // 9: R2 (handled via axes)
    XBOX_LB,      // 10: L1
    XBOX_RB,      // 11: R1
    XBOX_Y,       // 12: Triangle
    XBOX_B,       // 13: Circle
    XBOX_A,       // 14: Cross
    XBOX_X,       // 15: Square
    XBOX_GUIDE    // 16: PS
};

JoyInterface::JoyInterface() {
    memset(buttons_, 0, sizeof(buttons_));
    memset(axes_, 0, sizeof(axes_));
}

void JoyInterface::init() {
    serialBuffer_ = "";
}

void JoyInterface::readESP32Joy() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            parseEsp32Line(serialBuffer_);
            serialBuffer_ = "";
        } else {
            serialBuffer_ += c;
        }
    }
}

void JoyInterface::setFromJoyMsg(const int32_t *buttons, size_t btn_size, const float *axes, size_t axes_size) {
    // Direct copy for Xbox/ROS2
    memset(buttons_, 0, sizeof(buttons_));
    memset(axes_, 0, sizeof(axes_));
    for (size_t i = 0; i < XBOX_BTN_COUNT && i < btn_size; ++i) {
        buttons_[i] = (buttons[i] != 0);
    }
    for (size_t i = 0; i < XBOX_AXES_COUNT && i < axes_size; ++i) {
        axes_[i] = axes[i];
    }
}

bool JoyInterface::getButton(uint8_t btn_idx) const {
    if (btn_idx < XBOX_BTN_COUNT) return buttons_[btn_idx];
    return false;
}

float JoyInterface::getAxis(uint8_t axis_idx) const {
    if (axis_idx < XBOX_AXES_COUNT) return axes_[axis_idx];
    return 0.0f;
}

void JoyInterface::mapPs3ToXbox(const int32_t *ps3_buttons, const float *ps3_axes) {
    // Clear
    memset(buttons_, 0, sizeof(buttons_));
    memset(axes_, 0, sizeof(axes_));

    // Map PS3 buttons to Xbox buttons
    buttons_[XBOX_A]      = ps3_buttons[14]; // Cross
    buttons_[XBOX_B]      = ps3_buttons[13]; // Circle
    buttons_[XBOX_X]      = ps3_buttons[15]; // Square
    buttons_[XBOX_Y]      = ps3_buttons[12]; // Triangle
    buttons_[XBOX_LB]     = ps3_buttons[10]; // L1
    buttons_[XBOX_RB]     = ps3_buttons[11]; // R1
    buttons_[XBOX_BACK]   = ps3_buttons[0];  // Select
    buttons_[XBOX_START]  = ps3_buttons[3];  // Start
    buttons_[XBOX_GUIDE]  = ps3_buttons[16]; // PS
    buttons_[XBOX_LS]     = ps3_buttons[1];  // L3
    buttons_[XBOX_RS]     = ps3_buttons[2];  // R3

    // Map PS3 axes to Xbox axes
    axes_[XBOX_AX_LX] = ps3_axes[0];
    axes_[XBOX_AX_LY] = ps3_axes[1];
    axes_[XBOX_AX_RX] = ps3_axes[2];
    axes_[XBOX_AX_RY] = ps3_axes[3];
    axes_[XBOX_AX_LT] = ps3_axes[4]; // L2 analog
    axes_[XBOX_AX_RT] = ps3_axes[5]; // R2 analog

    // D-pad: map PS3 D-pad buttons to Xbox D-pad axes
    axes_[XBOX_AX_DX] = 0.0f;
    axes_[XBOX_AX_DY] = 0.0f;
    if (ps3_buttons[4]) axes_[XBOX_AX_DY] = 1.0f;   // Up
    if (ps3_buttons[6]) axes_[XBOX_AX_DY] = -1.0f;  // Down
    if (ps3_buttons[7]) axes_[XBOX_AX_DX] = -1.0f;  // Left
    if (ps3_buttons[5]) axes_[XBOX_AX_DX] = 1.0f;   // Right
}

void JoyInterface::parseEsp32Line(const String& line) {
    int idx = 0;
    int lastIdx = 0;
    int32_t ps3_buttons[17] = {0};
    float ps3_axes[6] = {0.0f};

    // Parse 17 buttons
    for (int i = 0; i < 17; ++i) {
        idx = line.indexOf(',', lastIdx);
        if (idx == -1) return; // Not enough data
        ps3_buttons[i] = line.substring(lastIdx, idx).toInt();
        lastIdx = idx + 1;
    }
    // Parse 6 axes
    for (int i = 0; i < 6; ++i) {
        idx = line.indexOf(',', lastIdx);
        if (idx == -1 && i < 5) return; // Not enough data
        ps3_axes[i] = line.substring(lastIdx, idx == -1 ? line.length() : idx).toFloat();
        lastIdx = idx + 1;
    }

    // Map PS3 to Xbox
    mapPs3ToXbox(ps3_buttons, ps3_axes);
}
