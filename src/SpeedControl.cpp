// KODE KONTROL PWM MOTOR DENGAN PID
#include <Arduino.h>
#include "Motor.h"
#include "FFPID.h"
#include "CalculateEncoder.h"
#include "MovingAverage.h"

#define PPR 7680       // Pulses per revolution dari encoder (nilai spesifik motor)
#define ENCODER1_PIN_B 19 // Channel A encoder
#define ENCODER1_PIN_A 20 // Channel B encoder
#define ENCODER2_PIN_B 37 // Channel A encoder
#define ENCODER2_PIN_A 38 // Channel B encoder
#define ENCODER3_PIN_A 21 // Channel A encoder
#define ENCODER3_PIN_B 47 // Channel B encoder
#define TS_ENCODER 10   // Time sampling encoder dalam ms

unsigned long millisEncoder = 0;

// Membuat objek motor, dengan parameter pin ENA, FWD, REV
Motor motorF(41, 40, 39, 0, 1); // uses channels 0 and 1
Motor motorL(16, 10, 11, 2, 3); // uses channels 2 and 3
Motor motorR(1, 2, 42, 4, 5);   // uses channels 4 and 5

//Moving Average Object
MovingAverage avgVel1(10);
MovingAverage avgVel2(100);
MovingAverage avgVel3(10);

FFPID tespid1(TS_ENCODER/1000.0f, 34.0f, 0.0f, 7.0f);
FFPID tespid2(TS_ENCODER/1000.0f, 20.0f, 0.0f, 7.0f);
FFPID tespid3(TS_ENCODER/1000.0f, 15.0f, 0.0f, 7.0f);

CalculateEncoder motor1 (100, PPR);
CalculateEncoder motor2 (100, PPR);
CalculateEncoder motor3 (100, PPR);

float targetSpeed = 0.0f;

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
  bool B = digitalRead(ENCODER3_PIN_A);
  bool A = digitalRead(ENCODER3_PIN_B); 
  
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
  if(Serial.available() > 0){
    targetSpeed = Serial.parseFloat();
  }

  if (millis() - millisEncoder >= TS_ENCODER) {
    millisEncoder = millis(); 
   
    motor1.update(encoderPos1);
    motor2.update(encoderPos2);
    motor3.update(encoderPos3);

    float currentSpeed1 = avgVel1.movingAverage(motor1.getAngularVel());
    float currentSpeed2 = avgVel2.movingAverage(motor2.getAngularVel());
    float currentSpeed3 = avgVel3.movingAverage(motor3.getAngularVel());

    float pwm1 = tespid1.getOutput_velControl(currentSpeed1, targetSpeed, 24.0f);
    float pwm2 = tespid2.getOutput_velControl(currentSpeed2, targetSpeed, 24.0f);
    float pwm3 = tespid3.getOutput_velControl(currentSpeed3, targetSpeed, 24.0f);

    motorF.speed(pwm1);
    motorL.speed(pwm2);
    motorR.speed(pwm3);

    Serial.print("Target: ");
    Serial.print(targetSpeed);

    Serial.print(",Current1: ");
    Serial.print(currentSpeed1);
    Serial.print(",pwm1: ");
    Serial.println(pwm1);

    // Serial.print(",Current2: ");
    // Serial.print(currentSpeed2);
    // Serial.print(",pwm2: ");
    // Serial.println(pwm2);

    // Serial.print(",Current3: ");
    // Serial.print(currentSpeed3);
    // Serial.print(",pwm3: ");
    // Serial.println(pwm3);
  }
}

