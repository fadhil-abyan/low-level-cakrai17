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

void IRAM_ATTR onEncoderChange() {
  bool A = digitalRead(ENCODER_PIN_A); // baca channel A
  bool B = digitalRead(ENCODER_PIN_B); // baca channel B
  
  // Jika A == B → arah searah jarum jam (CW)
  // Jika A != B → arah berlawanan jarum jam (CCW)
  if (A == B) {
    encoderPos++; 
    direction = true;   // CW
  } else {
    encoderPos--;
    direction = false;  // CCW
  }
}

void setup(){
    // Inisialisasi serial monitor
  Serial.begin(115200);
  
  // Set encoder pin sebagai input
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  
  // Pasang interrupt di pin encoder A
  // Akan memicu ISR onEncoderChange() setiap ada perubahan sinyal (naik/turun)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), onEncoderChange, CHANGE);
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
