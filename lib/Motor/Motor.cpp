#include "Motor.h"

Motor::Motor(int enPin, int lpwmPin, int rpwmPin, int pwmChannelL, int pwmChannelR, int freq, int resolution)
  : enPin(enPin), lpwmPin(lpwmPin), rpwmPin(rpwmPin),
    pwmChannelL(pwmChannelL), pwmChannelR(pwmChannelR),
    freq(freq), resolution(resolution) {

  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH); // Enable motor driver

  // Attach PWM to pins
  ledcSetup(pwmChannelL, freq, resolution);
  ledcAttachPin(lpwmPin, pwmChannelL);

  ledcSetup(pwmChannelR, freq, resolution);
  ledcAttachPin(rpwmPin, pwmChannelR);
}

void Motor::speed(float pwm) {
  // Clamp pwm range [-1,1]
  if (pwm > 1.0) pwm = 1.0;
  if (pwm < -1.0) pwm = -1.0;

  int maxDuty = (1 << resolution) - 1;  // e.g. 255 for 8-bit
  int pwmValue = abs(pwm) * maxDuty;

  if (pwm > 0) {
    ledcWrite(pwmChannelL, pwmValue);
    ledcWrite(pwmChannelR, 0);
  } else {
    ledcWrite(pwmChannelL, 0);
    ledcWrite(pwmChannelR, pwmValue);
  }
}
