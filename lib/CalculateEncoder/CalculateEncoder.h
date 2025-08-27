#ifndef CALCULATE_ENCODER_H
#define CALCULATE_ENCODER_H

#include <Arduino.h>

class CalculateEncoder {
public:
    // Constructor
    CalculateEncoder(float wheelDiameter, int ppr);

    // Update encoder readings
    void update(int32_t count);

    // Getters
    int32_t getPulse() const;
    float getLinearPos() const;
    float getAngularPos() const;
    float getLinearVel() const;
    float getAngularVel() const;

private:
    float wheelDiameter;   // Wheel diameter in meters
    int ppr;              // Pulses per revolution

    int32_t lastCount;    // Last encoder count
    unsigned long lastTime; // Last update time in ms

    float ang_pos;        // Angular position in radians
    float lin_pos;        // Linear position in meters
    float ang_vel;        // Angular velocity in rad/s
    float lin_vel;        // Linear velocity in m/s
};

#endif // CALCULATE_ENCODER_H