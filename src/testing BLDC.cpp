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

// Tentukan kecepatan putaran maksimum motor dalam rad/s
// Anda harus mengukur nilai ini secara eksperimental atau melihat datasheet motor Anda.
// Nilai ini sangat penting untuk akurasi.
const float MAX_SPEED_RAD_PER_SEC = 25.13; // Contoh: Asumsi kecepatan maks adalah 240 RPM (25.13 rad/s)


// Fungsi kustom untuk memetakan nilai float
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Memulai kalibrasi BLDC motor...");
  // Hubungkan objek servo ke pin
  myBLDCMotor.attach();
  
  // Kalibrasi ESC. Kirim sinyal 0% throttle terlebih dahulu untuk beberapa detik.
  // Ini adalah langkah penting untuk banyak ESC.
  myBLDCMotor.setMotorPercentage(0, DIRECTION_CW_US, DIRECTION_CCW_US);
  Serial.println("Tunggu 3 detik sampai ESC berbunyi...");
  delay(3000); 
  Serial.println("Kalibrasi selesai.");
}

void loop() {
  // Target kecepatan yang ingin dicapai (dalam rad/s)
  const float TARGET_SPEED_RAD_PER_SEC = 6.28;

  // Ubah target kecepatan (rad/s) menjadi persentase throttle
  // Gunakan fungsi map_float() untuk hasil yang lebih presisi
  float motorPercentage = map_float(TARGET_SPEED_RAD_PER_SEC, 0.0, MAX_SPEED_RAD_PER_SEC, 0.0, 100.0);
  
  // Pastikan persentase tidak melebihi 100%
  motorPercentage = constrain(motorPercentage, 0.0, 100.0);

  Serial.print("Target Speed: ");
  Serial.print(TARGET_SPEED_RAD_PER_SEC);
  Serial.print(" rad/s, Mapped to Percentage: ");
  Serial.print(motorPercentage);
  Serial.println(" %");

  // Atur kecepatan motor dengan persentase yang sudah dihitung
  // Positif untuk putaran CW
  myBLDCMotor.setMotorPercentage(motorPercentage, DIRECTION_CW_US, DIRECTION_CCW_US);
}
