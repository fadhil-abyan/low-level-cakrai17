#pragma once
#include <Arduino.h>
#include "FalconMotor.h"  // asumsi FalconPWM sudah di sini

class DualFalconPWM {
private:
    FalconPWM left, right;
    int config[2];  // [0] = kiri, [1] = kanan
    bool attached = false;

public:
    DualFalconPWM() {
        config[0] = 1;  // kiri normal
        config[1] = -1; // kanan dibalik
    }

    void attach(int leftPin, int rightPin) {
        left.attach(leftPin);
        right.attach(rightPin);
        attached = true;
    }

    void setConfig(int kiri, int kanan) {
        config[0] = kiri;
        config[1] = kanan;
    }

    void speed(float pwmVal) {
        if (!attached) return;
        pwmVal = constrain(pwmVal, -1.0f, 1.0f);
        left.speed(pwmVal * config[0]);
        right.speed(pwmVal * config[1]);
    }
};
