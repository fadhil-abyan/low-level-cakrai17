#ifndef INFRAREDCOUNTER_H
#define INFRAREDCOUNTER_H

#include <Arduino.h>

#define SPEED_AVG_WINDOW 5  // Jumlah sampel moving average

class InfraredCounter {
  public:
    InfraredCounter(uint8_t pin, int ppr);

    void begin();
    void update();                     // Update biasa (pulse rising)
    void updateWithInterval(unsigned long intervalMs); // Hitung kecepatan tiap interval

    int getCount();                    // Total pulse (rising edge)
    void reset();                      // Reset semua data

    float getAngularSpeed(bool smoothed = true);         // rad/s
    float getLinearSpeed(float wheelRadius);             // m/s
    float getAngle();                                     // radian total

  private:
    uint8_t _pin;
    int _ppr;
    int _lastState;
    int _count;

    // Untuk kecepatan
    unsigned long _lastUpdateTime;
    int _lastCount;
    float _angularSpeed;

    // Moving average buffer
    float _speedBuffer[SPEED_AVG_WINDOW];
    int _speedIndex;
    bool _bufferFilled;
};

#endif
