#include <Arduino.h>
#include "Motor.h"
#include "FFPID.h"

// Definisi pin motor dan encoder, serta parameter lain
#define EN_PIN 41       // Pin enable motor (PWM output ke motor driver)
#define FWD_PIN 40      // Pin arah motor maju (forward)
#define REV_PIN 39      // Pin arah motor mundur (reverse)
#define PPR 40900       // Pulses per revolution dari encoder (nilai spesifik motor)
#define ENCODER_PIN_A 21 // Channel A encoder
#define ENCODER_PIN_B 47 // Channel B encoder
#define TS_ENCODER 10   // Time sampling encoder dalam ms

uint32_t millisEncoder = 0;  // Variabel penanda waktu sampling encoder

// Membuat objek motor, dengan parameter pin ENA, FWD, REV
Motor tesMotor(EN_PIN, FWD_PIN, REV_PIN);

// Membuat objek PID (Feed-Forward PID) dengan Ts, Kp, Ki, Kd
FFPID tespid(TS_ENCODER/1000.0f, 0.10f, 0.0f, 0.0f);

// Membuat objek PID lain (sepertinya miniPID library tambahan)
MiniPID pid(0, 0, 0);

// Variabel global untuk encoder
volatile int encoderPos = 0;   // Menyimpan posisi encoder (berubah setiap interrupt)
volatile bool direction = false; // Menyimpan arah putaran encoder (CW atau CCW)

// ISR (Interrupt Service Routine) untuk encoder
// Dipanggil setiap kali ada perubahan di pin A encoder
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

void setup() {
  // Inisialisasi serial monitor
  Serial.begin(115200);
  
  // Set encoder pin sebagai input
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  
  // Pasang interrupt di pin encoder A
  // Akan memicu ISR onEncoderChange() setiap ada perubahan sinyal (naik/turun)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), onEncoderChange, CHANGE);
}

void loop() {
  // Setiap TS_ENCODER ms, lakukan perhitungan posisi dan kontrol motor
  if (millis() - millisEncoder >= TS_ENCODER) {
    millisEncoder = millis(); 
    
    // Konversi posisi encoder (dalam pulsa) ke radian
    float currentPos1 = encoderPos * (2 * PI / PPR); 
    
    // Target posisi (dalam derajat → diubah ke radian)
    float target = 90.0f; // target = 90 derajat
    float targetRad = (target / 180.0f) * PI; // konversi ke radian
    
    // Hitung output PWM dari PID
    // Input: posisi saat ini (rad), target (rad), gain (1.0f)
    float pwm = tespid.getOutput_miniPID(currentPos1, targetRad, 1.0f);
    
    // Debug print ke serial monitor
    Serial.print("Encoder : ");
    Serial.print(encoderPos);
    
    Serial.print("PWM : ");
    Serial.println(pwm);

    //Serial.print(" | Target : ");
    //Serial.print(target);
    //Serial.print(" | Posisi : ");
    //Serial.println((currentPos1 / PI) * 180.0f); // tampilkan posisi dalam derajat
    
    // Kirim nilai PWM ke motor
    //tesMotor.speed(pwm); 
  }
}
