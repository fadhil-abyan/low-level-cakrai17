#include <Arduino.h>
#include "Motor.h"
#include "FFPID.h"
#include "CalculateEncoder.h"

#define PPR 40900       // Pulses per revolution dari encoder (nilai spesifik motor)
#define ENCODER1_PIN_A 19 // Channel A encoder
#define ENCODER1_PIN_B 20 // Channel B encoder
#define ENCODER2_PIN_A 37 // Channel A encoder
#define ENCODER2_PIN_B 38 // Channel B encoder
#define ENCODER3_PIN_A 21 // Channel A encoder
#define ENCODER3_PIN_B 47 // Channel B encoder
#define TS_ENCODER 10   // Time sampling encoder dalam ms

unsigned long millisEncoder = 0;

// Membuat objek motor, dengan parameter pin ENA, FWD, REV
Motor motorF(41, 40, 39);
Motor motorL(16, 10, 11);
Motor motorR(1, 2, 42);

FFPID tespid(TS_ENCODER/1000.0f, 0.10f, 0.0f, 0.0f);

CalculateEncoder motor1 (100, PPR);
CalculateEncoder motor2 (100, PPR);
CalculateEncoder motor3 (100, PPR);

volatile int encoderPos1 = 0;   
volatile bool direction1 = false;
volatile int encoderPos2 = 0;   
volatile bool direction2 = false;
volatile int encoderPos3 = 0;   
volatile bool direction3 = false;

float pwm = 0.0f;

uint32_t lastMillis = 0;

void IRAM_ATTR onEncoder1Change() {
  bool A = digitalRead(ENCODER1_PIN_A); 
  bool B = digitalRead(ENCODER1_PIN_B); 
  

  if (A == B) {
    encoderPos1++; 
    direction1 = true;   
  } else {
    encoderPos1--;
    direction1 = false;  
  }
}

void IRAM_ATTR onEncoder2Change() {
  bool A = digitalRead(ENCODER2_PIN_A);
  bool B = digitalRead(ENCODER2_PIN_B); 
  
  if (A == B) {
    encoderPos2++; 
    direction2 = true;  
  } else {
    encoderPos2--;
    direction2 = false; 
  }
}

void IRAM_ATTR onEncoder3Change() {
  bool A = digitalRead(ENCODER3_PIN_A);
  bool B = digitalRead(ENCODER3_PIN_B); 
  
  if (A == B) {
    encoderPos3++; 
    direction3 = true;
  } else {
    encoderPos3--;
    direction3 = false;
  }
}


void setup(){
    // Inisialisasi serial monitor
  Serial.begin(115200);
  
  // Set encoder pin sebagai input
  pinMode(ENCODER1_PIN_A, INPUT);
  pinMode(ENCODER1_PIN_B, INPUT);
  pinMode(ENCODER2_PIN_A, INPUT);
  pinMode(ENCODER2_PIN_B, INPUT);
  pinMode(ENCODER3_PIN_A, INPUT);
  pinMode(ENCODER3_PIN_B, INPUT);
  
  
  // Pasang interrupt di pin encoder A
  // Akan memicu ISR onEncoderChange() setiap ada perubahan sinyal (naik/turun)
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), onEncoder1Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), onEncoder2Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PIN_A), onEncoder3Change, CHANGE);
}


void loop(){
  if (millis() - millisEncoder >= TS_ENCODER) {
    millisEncoder = millis(); 
    
    // float currentSpeed1 = motor1.getAngularVel();
    // float currentSpeed2 = motor2.getAngularVel();
    // float currentSpeed3 = motor3.getAngularVel();

    // float targetSpeed1;
    // float targetSpeed2;
    // float targetSpeed3;
    
  
    // // Hitung output PWM dari PID
    // // Input: posisi saat ini (rad), target (rad), gain (1.0f)
    // float pwm1 = tespid.getOutput_velControl(currentSpeed1, targetSpeed1, 24.0f);
    // float pwm2 = tespid.getOutput_velControl(currentSpeed1, targetSpeed2, 24.0f);
    // float pwm3 = tespid.getOutput_velControl(currentSpeed1, targetSpeed3, 24.0f);

    
  
     motorF.speed(0.1); 
    // motorR.speed(pwm2); 
    // motorL.speed(pwm3); 

    Serial.print("Encoder 1: ");
    Serial.print(encoderPos1);
    Serial.print("/n");
  }
}
