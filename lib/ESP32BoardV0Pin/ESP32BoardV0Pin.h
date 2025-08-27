
// Communication Interface Pins
#define ESP32_UART0_RX  44
#define ESP32_UART0_TX  43
// * The ESP32_CAN_RX is also ESP32_UART1_RX
// * The ESP32_CAN_TX is also ESP32_UART1_TX
// * Configure this using header in board (One at a time)
#define ESP32_UART1_RX  18
#define ESP32_UART1_TX  17
#define ESP32_CAN_RX    18
#define ESP32_CAN_TX    17
#define ESP32_I2C_SDA   8
#define ESP32_I2C_SCL   9

// Motor Control Pins
// * All of the motor's EN is shorted
// * ENABLE is always set to HIGH for the motor to move
// * Motor Speed is controlled through difference value between FWD and REV
// * FWD is RRPM and REV is LPWM
#define ESP32_MTR0_EN   12
#define ESP32_MTR0_FWD  40
#define ESP32_MTR0_REV  39
#define ESP32_MTR1_EN   12
#define ESP32_MTR1_FWD  42
#define ESP32_MTR1_REV  41
#define ESP32_MTR2_EN   12
#define ESP32_MTR2_FWD  1
#define ESP32_MTR2_REV  2
#define ESP32_MTR3_EN   12
#define ESP32_MTR3_FWD  10
#define ESP32_MTR3_REV  11

// Encoder Pins
#define ESP32_ENC0_A 19
#define ESP32_ENC0_B 20
#define ESP32_ENC1_A 35
#define ESP32_ENC1_B 36
#define ESP32_ENC2_A 47
#define ESP32_ENC2_B 21
#define ESP32_ENC3_A 37
#define ESP32_ENC3_B 38

// PWM Pins
#define ESP32_PWM0 4
#define ESP32_PWM1 5
#define ESP32_PWM2 6
#define ESP32_RGB  14

// Miscellaneous Pins
#define ESP32_BUILTIN_LED 48