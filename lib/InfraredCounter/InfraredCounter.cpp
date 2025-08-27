#include "InfraredCounter.h"

InfraredCounter::InfraredCounter(uint8_t pin, int ppr) {
  _pin = pin;
  _ppr = ppr;
  _lastState = 0;
  _count = 0;
  _lastUpdateTime = 0;
  _lastCount = 0;
  _angularSpeed = 0;
  _speedIndex = 0;
  _bufferFilled = false;

  for (int i = 0; i < SPEED_AVG_WINDOW; i++) {
    _speedBuffer[i] = 0;
  }
}

void InfraredCounter::begin() {
  pinMode(_pin, INPUT);
  _lastState = digitalRead(_pin);
}

void InfraredCounter::update() {
  int currentState = digitalRead(_pin);
  if (_lastState == 0 && currentState == 1) {
    _count++;
  }
  _lastState = currentState;
}

void InfraredCounter::updateWithInterval(unsigned long intervalMs) {
  update();

  unsigned long now = millis();
  if (now - _lastUpdateTime >= intervalMs) {
    int deltaCount = _count - _lastCount;
    float deltaTheta = (deltaCount * 2.0 * PI) / _ppr;  // radian
    float instSpeed = deltaTheta / (intervalMs / 1000.0); // rad/s

    _speedBuffer[_speedIndex] = instSpeed;
    _speedIndex = (_speedIndex + 1) % SPEED_AVG_WINDOW;
    if (_speedIndex == 0) _bufferFilled = true;

    _angularSpeed = instSpeed;
    _lastCount = _count;
    _lastUpdateTime = now;
  }
}

int InfraredCounter::getCount() {
  return _count;
}

void InfraredCounter::reset() {
  _count = 0;
  _lastCount = 0;
  _angularSpeed = 0;
  _speedIndex = 0;
  _bufferFilled = false;
  for (int i = 0; i < SPEED_AVG_WINDOW; i++) _speedBuffer[i] = 0;
}

float InfraredCounter::getAngularSpeed(bool smoothed) {
  if (!smoothed) return _angularSpeed;

  int n = _bufferFilled ? SPEED_AVG_WINDOW : _speedIndex;
  if (n == 0) return 0;

  float sum = 0;
  for (int i = 0; i < n; i++) {
    sum += _speedBuffer[i];
  }
  return sum / n;
}

float InfraredCounter::getLinearSpeed(float wheelRadius) {
  return getAngularSpeed(true) * wheelRadius;
}

float InfraredCounter::getAngle() {
  return (_count * 2.0 * PI) / _ppr;
}
