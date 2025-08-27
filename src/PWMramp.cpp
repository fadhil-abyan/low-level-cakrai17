// Ramp Nilai PWM Motor
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

float userpwm = 0; // Nilai pwm dari input user
float currentpwm = 0; // Nilai pwm pada motor sekarang
float originalCurrentpwm = 0; // Nilai pwm konstan pada motor terakhir

uint32_t lastpwmMillis = 0; // Waktu terakhir user meng-input nilai pwm
uint32_t lastMillis = 0; // Waktu pengecekan terakhir


// Fungsi mengubah kecepatan motor
void speedChange(){
  if(currentpwm != userpwm){ // Cek jika pwm pada motor sudah sesuai dengan pwm yang diinginkan user
    unsigned long delta_ms = millis() - lastpwmMillis; // Hitung perubahan waktu sekarang dengan ketika user menginput nilai pwm
    
    float delta_s = static_cast<float>(delta_ms) / 1000.0f; // Ubah delta waktu dari ms ke s
    
    // Cek jika pwm sekarang lebih atau kurang dari yang diinginkan user
    // Jika kurang, pwm akan ditambah dengan rate 0.05/s
    // Jika lebih, pwm akan berkurang dengan rate yang sama
    if(currentpwm < userpwm){
      currentpwm = originalCurrentpwm + (0.05*delta_s);
    }else{
      currentpwm = originalCurrentpwm - (0.05*delta_s);
    }
  }
  
  // Cek jika pwm sekarang meng-overshoot nilai pwm user
  // Jika ya, maka set pwm sekarang sama dengan pwm user
  if((currentpwm >= userpwm && originalCurrentpwm < userpwm) || (currentpwm <= userpwm && originalCurrentpwm > userpwm)){
    currentpwm = userpwm;
    originalCurrentpwm = userpwm;
  }

  // Set kecepatan masing-masing motor dengan nilai pwm sekarang
  motorF.speed(currentpwm);
  motorL.speed(currentpwm);
  motorR.speed(currentpwm);
}

void setup()
{
  Serial.begin(115200);
}

void loop(){
  // Cek jika ada input di serial monitor
  if(Serial.available() > 0){
    userpwm = Serial.parseFloat();
    lastpwmMillis = millis(); // Catat waktu ketika user melakukan input
  }

  // Panggil prosedur speedChange() dan print nilai PWM dan waktu setiap 100 ms
  if(millis() - lastMillis >= 100){
    lastMillis = millis();

    speedChange();

    Serial.print("Elapsed time: ");
    Serial.println((static_cast<float>(lastMillis) / 1000.0f));
    Serial.println(" | ");
    Serial.print("Current PWM: ");
    Serial.println(currentpwm);
    Serial.print("\n");
  }
}

