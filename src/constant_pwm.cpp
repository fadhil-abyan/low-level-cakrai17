// TEST PWM Motor Konstan
#include <Arduino.h>
#include "Motor.h"

// Definisi pin motor dan encoder, serta parameter lain
#define EN_PIN 41       // Pin enable motor (PWM output ke motor driver)
#define FWD_PIN 40      // Pin arah motor maju (forward)
#define REV_PIN 39      // Pin arah motor mundur (reverse)

// Membuat objek motor, dengan parameter pin ENA, FWD, REV
Motor motorF(41, 40, 39);
Motor motorL(16, 10, 11);
Motor motorR(1, 2, 42);

float pwm = 0.0f;

uint32_t lastMillis = 0;

void setup(){
  Serial.begin(115200);
}


void loop(){
  if(Serial.available() > 0){
    pwm = Serial.parseFloat();
  }
  motorF.speed(pwm);
  motorR.speed(pwm);
  motorL.speed(pwm);
  Serial.print(pwm);
  Serial.print("\n");
  delay(1000);
}
