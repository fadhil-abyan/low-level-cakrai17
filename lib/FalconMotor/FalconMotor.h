#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class FalconPWM {
private:
    Servo motor;
    int pin;
    int pwmMin = 1000;
    int pwmMax = 2000;
    int pwmNeutral = 1500;

public:
    FalconPWM() {}

    void attach(int pwmPin, int freqHz = 50) {
        pin = pwmPin;
        motor.setPeriodHertz(freqHz);
        motor.attach(pin, pwmMin, pwmMax);
    }

    void speed(float value) {
        value = constrain(value, -1.0f, 1.0f);
        int pulse = pwmNeutral + (int)(value * 500.0f);
        motor.writeMicroseconds(pulse);
    }
};
