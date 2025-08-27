#include "CalculateEncoder.h"
#include <Arduino.h>
#include <math.h>  // untuk isfinite dan PI

#define MIN_DT 1e-6f
#define MAX_VEL 100.0f  // batas kecepatan wajar

CalculateEncoder::CalculateEncoder(float wheelDiameter, int ppr){
    this->wheelDiameter = wheelDiameter;
    this->ppr = ppr;
    this->lastCount = 0;
    this->lastTime = millis();

    this->ang_pos = 0.0f;
    this->lin_pos = 0.0f;
    this->ang_vel = 0.0f;
    this->lin_vel = 0.0f;
}

void CalculateEncoder::update(int32_t count) {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime;
    float deltaTime_s = (float)deltaTime / 1000.0f;

    if (deltaTime_s < MIN_DT) deltaTime_s = MIN_DT;

    int32_t deltaCount = count - this->lastCount;
    float deltaRevolutions = (float)deltaCount / this->ppr;

    this->ang_pos += deltaRevolutions * 2.0f * PI;
    this->lin_pos += deltaRevolutions * (PI * wheelDiameter);

    this->ang_vel = (deltaRevolutions * 2.0f * PI) / deltaTime_s;
    this->lin_vel = (deltaRevolutions * (PI * wheelDiameter)) / deltaTime_s;

    // Proteksi nilai tidak valid
    if (!isfinite(this->ang_vel) || fabs(this->ang_vel) > MAX_VEL)
        this->ang_vel = 0.0f;
    if (!isfinite(this->lin_vel) || fabs(this->lin_vel) > MAX_VEL)
        this->lin_vel = 0.0f;

    this->lastCount = count;
    this->lastTime = currentTime;
}

int32_t CalculateEncoder::getPulse() const {
    return this->lastCount;
}
float CalculateEncoder::getLinearPos() const {
    return this->lin_pos;
}
float CalculateEncoder::getAngularPos() const {
    return this->ang_pos;
}
float CalculateEncoder::getLinearVel() const {
    return this->lin_vel;
}
float CalculateEncoder::getAngularVel() const {
    return this->ang_vel;
}
