#include <Arduino.h>
#include "ServoBLDC.h"

// Tentukan pin GPIO untuk ESC throttle dan direction
const int THROTTLE_PIN = 4; // Ganti dengan pin yang terhubung ke ESC throttle
const int DIRECTION_PIN = 5; // Ganti dengan pin yang terhubung ke ESC direction (jika ada)

// Tentukan batas pulse width ESC throttle dalam mikrosekon
const int THROTTLE_MIN_US = 1000;
const int THROTTLE_MAX_US = 2000;

// Tentukan pulse width untuk arah putaran
const int DIRECTION_CW_US = 1250; // Contoh: CW (clockwise)
const int DIRECTION_CCW_US = 1750; // Contoh: CCW (counter-clockwise)

// Buat objek dari kelas ServoBLDC
ServoBLDC myBLDCMotor(THROTTLE_PIN, DIRECTION_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);

void setup() {
  Serial.begin(115200);

  myBLDCMotor.attach();
  }

void loop() {
  float motorPercentage = 50;

  // Atur kecepatan motor dengan persentase yang sudah dihitung
  // Positif untuk putaran CW
  myBLDCMotor.setMotorPercentage(motorPercentage, DIRECTION_CW_US, DIRECTION_CCW_US);
}
