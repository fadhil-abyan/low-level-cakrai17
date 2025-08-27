// ServoBLDC.h
#ifndef SERVO_BLDC_H
#define SERVO_BLDC_H

#include <Arduino.h>
#include <ESP32Servo.h> // Keep this include! It's correct.

class ServoBLDC {
public:
    // Constructor: Takes throttle pin, direction pin, and pulse width limits for the throttle ESC
    ServoBLDC(int throttlePin, int directionPin, int throttleMinUs, int throttleMaxUs);

    // Initializes the internal Servo objects
    void attach();

    // Sets the motor speed and direction based on a percentage input.
    // percentage: -100 (full reverse) to 100 (full forward). 0 is stop.
    // dirCwUs: The microsecond value for CW direction (e.g., 1250 us)
    // dirCcwUs: The microsecond value for CCW direction (e.g., 1750 us)
    void setMotorPercentage(float percentage, int dirCwUs, int dirCcwUs);

private:
    // FIX: Change ESP32Servo to Servo
    Servo _throttleServo;
    Servo _directionServo;

    int _throttlePin;
    int _directionPin;
    int _minPulseWidth; // Min pulse width for the throttle ESC (e.g., 1000 us)
    int _maxPulseWidth; // Max pulse width for the throttle ESC (e.g., 2000 us)
};

#endif // SERVO_BLDC_H