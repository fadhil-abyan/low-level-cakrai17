#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  public:
    Motor(int enPin, int lpwmPin, int rpwmPin, int pwmChannelL, int pwmChannelR, int freq = 20000, int resolution = 8);

    void speed(float pwm);  // pwm range: -1.0 to 1.0

  private:
    int enPin;
    int lpwmPin;
    int rpwmPin;
    int pwmChannelL;
    int pwmChannelR;
    int freq;
    int resolution;
};

#endif
