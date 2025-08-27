//======== Section Includes ========// 
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <vector>
#include "ESP32Encoder.h"
#include <ESP32BoardV1Pin.h>
#include <HardwareSerial.h>
#include <Motor.h>
#include <MovingAverage.h>
#include <Adafruit_NeoPixel.h>
#include <FFPID.h>
#include <math.h>
#include "../config/mekanisme_r1_config.h"
#include "DualFalcon.h"

//======== Section Constants and Enums ========//
#define PPR 537.6
#define PPR_SHOOTER 2400.0f
#define TS_LEDBLINK 200 
#define TS_ENCODER 10
#define RXD2 16
#define TXD2 17  
#define CATCHER_MAX_PULSE 5800.0f


MiniPID pidShooter(0.002, 0.0001, 0);
float targetShooterSpeed = 0.0f;


enum stateMachine {
  STATE_IDLE,
  STATE_MAIN
};

enum LED_COLOR_STATE {
  YELLOW,
  GREEN,
  RED,
  BLUE,
};

stateMachine currentState;
uint32_t heartBeatFromMaster = 0;
uint32_t millisHeartbeat = 0;
uint32_t millisLed       = 0;
uint32_t millisEncoder   = 0;
uint32_t millisElevation = 0;
uint32_t millisLauncher    = 0;
uint32_t millisCatcher    = 0;
bool ledState = false;

void led_blink(LED_COLOR_STATE ledColorState);
void hardware_init();
void controlCatcher(); 
void controlElevasi();
void controlLauncher(); 
void controlShooter();

float smooth_target(float t, float start, float end, float t0);
float ramp_target(float t, float start, float end, float t0, float duration);
String getJoy(const String &data, int index);
void onSerialReceive_testingAll();

// Mechanism related variables
// Pusher Controller
float prevPos_pusher = 0;
float startTarget_pusher = 0; 
float endTarget_pusher = 0; 
float lastPositionCommand_pusher = 0.0f; 
float current_speed = 0.0f;
float prevTime = 0.0f;
float target_phase = 0.0f; 
bool targetPusherDone = true;

// Elevation shooter
int prox_elevation = digitalRead(4);
unsigned long lastDpadRepeatTime = 0;
const unsigned long DPAD_REPEAT_INTERVAL = 200;  // ms

//rotation shooter 

//======== Section Global Variables ========//
Adafruit_NeoPixel led(1, ESP32_BUILTIN_LED, NEO_GRB + NEO_KHZ800);
HardwareSerial SerialPort(2); // UART2

//======== Section Hardware Objects ========//

Motor motorLauncher(MOTOR_LAUNCHER_EN, MOTOR_LAUNCHER_REV, MOTOR_LAUNCHER_FWD);
Motor motorElevation(MOTOR_ELEVATION_EN, MOTOR_ELEVATION_FWD, MOTOR_ELEVATION_REV);
Motor motorCatcher(MOTOR_CATCHER_EN, MOTOR_CATCHER_REV, MOTOR_CATCHER_FWD);

DualFalconPWM shooter;

ESP32Encoder encoderLauncher;
ESP32Encoder encoderElevation;
ESP32Encoder encoderCatcher;

ESP32Encoder encoderShooter;

FFPID pidLauncher(TS_ENCODER/1000.0f, 0.2f, 0.0f, 0.0f);
FFPID pidCatcher(TS_ENCODER/1000.0f, 0.15f, 0.0f, 0.0f);
FFPID pidElevationShooter(TS_ENCODER/1000.0f, 0.08f, 0.0f, 0.0f);


//======== Section Shared Data ========//
float elevationShooter = 0.0f;
bool launcherOn = false;
float positionCatcher = 0.0f;

// Mutex for protecting shared setpoint variables
SemaphoreHandle_t setpointMutex = NULL;

//======== Section PS3 Controller Variables ==================
String serialBuffer = "";
bool lastLeftPressed = false;
bool lastRightPressed = false;
bool lastTrianglePressed = false;
bool lastCirclePressed = false;
bool lastCrossPressed = false;
bool lastSquarePressed = false;
bool lastDpadUpPressed = false;
bool lastDpadDownPressed = false;

// PS3 Timeout Safety Variables
unsigned long lastPS3CommandTime = 0;
const unsigned long PS3_TIMEOUT_MS = 1000;  // 1 second timeout


//=========MAPPING=======================//

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//======== Section Hardware Init ========//
void hardware_init() {
    led.begin();
    led.setBrightness(35);
    
}

//======== Section PS3 Controller Functions ==================
void parsePS3Command(String data) {
    // Parse PS3 data format: select,l3,r3,start,up,right,down,left,l2,r2,l1,r1,triangle,circle,cross,square,ps,lx,ly,rx,ry,l2_analog,r2_analog
    
    float analogValues[6];
    bool digitalButtons[17];
    
    // Parse comma-separated values
    int valueIndex = 0;
    int startIndex = 0;

    for (int i = 0; i <= data.length(); i++) {
        if (i == data.length() || data.charAt(i) == ',') {
            if (valueIndex < 23) {
                String valueStr = data.substring(startIndex, i);
                
                if (valueIndex < 17) {
                    // Digital buttons (0 or 1)
                    digitalButtons[valueIndex] = (valueStr.toInt() == 1);
                } else {
                    // Analog values (floats)
                    analogValues[valueIndex - 17] = valueStr.toFloat();
                }
                valueIndex++;
            }
            startIndex = i + 1;
        }
    }
    
    // Extract specific values we need
    if (valueIndex >= 23) {
        // Record that we received a valid PS3 command
        lastPS3CommandTime = millis();
        
        bool selectBtn = digitalButtons[0];
        bool triangleBtn = digitalButtons[12];
        bool circleBtn = digitalButtons[13];
        bool crossBtn = digitalButtons[14];
        bool squareBtn = digitalButtons[15];
        bool dPadUpBtn = digitalButtons[4];
        bool dPadDownBtn = digitalButtons[6];
        
        // Update setpoints based on button presses (with edge detection)
        if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
            // Triangle button - position 4400
            if (triangleBtn && !lastTrianglePressed) {
                led_blink(BLUE);
                Serial.println("Triangle clicked!");
                positionCatcher = 5800.0f;
            }
            
            // Circle button - position 4000
            if (circleBtn && !lastCirclePressed) {
                led_blink(BLUE);
                Serial.println("Circle clicked!");
                positionCatcher = 5046.0f;
            }
            
            // Cross button - position 1000
            if (crossBtn && !lastCrossPressed) {
                led_blink(BLUE);
                Serial.println("Cross clicked!");
                positionCatcher = 1000.0f;
            }

            if (squareBtn && !lastSquarePressed) {
                launcherOn = true;
                Serial.println("Square pressed -> Launcher ON");
            }
            if (!squareBtn && lastSquarePressed) {
                launcherOn = false;
                Serial.println("Square released -> Launcher OFF");
            }

            // DPad Up - increase elevation
            if (dPadUpBtn && (millis() - lastDpadRepeatTime > DPAD_REPEAT_INTERVAL)) {
                led_blink(BLUE);
                Serial.println("DPad Up held!");
                elevationShooter += 50.0f;
                if (elevationShooter > 800.0f) elevationShooter = 800.0f;
                lastDpadRepeatTime = millis();
            }

            // DPad Down - decrease elevation
            if (dPadDownBtn && (millis() - lastDpadRepeatTime > DPAD_REPEAT_INTERVAL)) {
                led_blink(BLUE);
                Serial.println("DPad Down held!");
                elevationShooter -= 50.0f;
                if (elevationShooter < 0.0f) elevationShooter = 0.0f;
                lastDpadRepeatTime = millis();
            }
    
            // Reset with SELECT button
            if (selectBtn) {
                positionCatcher = 0.0f;
                // positionLauncher = 0.0f;
                elevationShooter = 0.0f;
            }
            
            xSemaphoreGive(setpointMutex);
        }
        
        // Update last button states
        lastTrianglePressed = triangleBtn;
        lastCirclePressed = circleBtn;
        lastCrossPressed = crossBtn;
        lastSquarePressed = squareBtn;
        lastDpadUpPressed = dPadUpBtn;
        lastDpadDownPressed = dPadDownBtn;
    }
}

void updateSetpointFromPS3() {
    // Read all available data from SerialPort
    while (SerialPort.available()) {
        char incomingByte = SerialPort.read();
        
        if (incomingByte == '\n') {
            // Complete message received, parse it
            if (serialBuffer.length() > 0) {
                parsePS3Command(serialBuffer);
                serialBuffer = "";  // Clear buffer for next message
            }
        } else {
            // Add character to buffer
            serialBuffer += incomingByte;
            
            // Prevent buffer overflow
            if (serialBuffer.length() > 500) {
                serialBuffer = "";  // Reset if buffer gets too long
            }
        }
    }
}

//======== Section Control Functions ========//

void controlShooter() {
    static float prevPos = 0.0f;
    static float speed = 0.0f;

    float currentPos = encoderShooter.getCount() * (2 * PI / 2400); // posisi (rad)

    if (millis() - millisLauncher > TS_ENCODER) {
        // Hitung kecepatan (rad/s)
        speed = (currentPos - prevPos) / (TS_ENCODER / 1000.0f);
        prevPos = currentPos;

        float targetSpeed = 0.0f;

        // Ambil target dari shared variable
        if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
            targetSpeed = targetShooterSpeed;
            xSemaphoreGive(setpointMutex);
        }

        // Hitung PWM dengan FFPID velocity control

        float pwm = pidShooter.getOutput(speed, targetSpeed);  // 12.0V max voltage

        shooter.speed(pwm);  // Normalisasi PWM di [-1, 1]

        // Debug print
        Serial.print("Speed = ");
        Serial.print(speed, 2);
        Serial.print(",Target = ");
        Serial.print(targetSpeed);
        Serial.print(",PWM = ");
        Serial.println(pwm, 3);



        millisLauncher = millis();
    }
}


void controlCatcher() {
    if (millis() - millisCatcher > TS_ENCODER) {
        float localTargetCatcher = 0.0f;
        
        // Get target position safely
        if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
            localTargetCatcher = positionCatcher;
            xSemaphoreGive(setpointMutex);
        }
        
        float pwm = pidCatcher.getOutput_miniPID(
            encoderCatcher.getCount() * (2 * PI / PPR),  // current position (rad)
            localTargetCatcher * (2 * PI / PPR),         // target converted from pulse to rad
            1.0f
        );
        
        motorCatcher.speed(pwm);
        
        // Serial.print("Catcher - pulse: ");
        // Serial.print(encoderCatcher.getCount());
        // Serial.print(" | current: ");
        // Serial.print(encoderCatcher.getCount() * (360.0f / PPR));
        // Serial.print(" | target: ");
        // Serial.print(localTargetCatcher);
        Serial.print(" | CATCHpwm: ");
        Serial.println(pwm);
        
        millisCatcher = millis();
    }
}


void controlElevation() {
    static float prevPos_elevation = 0.0f;
    static float speed = 0.0f;
    
    float currentPos_elevation = encoderElevation.getCount() * (2 * PI / PPR);
    
    if (millis() - millisElevation > TS_ENCODER) {
        speed = (prevPos_elevation - currentPos_elevation) / (TS_ENCODER/1000.0f);
        prevPos_elevation = currentPos_elevation;
        millisElevation = millis();
    }
    
    float localTargetElevation = 0.0f;
    
    // Get target position safely
    if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
        localTargetElevation = elevationShooter;
        xSemaphoreGive(setpointMutex);
    }
    
    float pwm = pidElevationShooter.getOutput_miniPID(
        currentPos_elevation,
        localTargetElevation * (PI / 180.0f), 
        1.0f
    );
    
    Serial.print("Elevation - current: ");
    Serial.print(currentPos_elevation * 180.0f / PI);
    Serial.print(" | target: ");
    Serial.print(localTargetElevation);
    Serial.print(" | pulses: ");
    Serial.print(encoderElevation.getCount());
    Serial.print(" | pwm: ");
    Serial.print(pwm);
    Serial.print(" | speed: ");
    Serial.println(speed);
    
    motorElevation.speed(pwm);
}


void controlLauncher() {
    float pwm = 0.0f;
    bool localLauncher = false;

    if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
        localLauncher = launcherOn;
        xSemaphoreGive(setpointMutex);
    }

    // Gunakan nilai boolean
    if (localLauncher) {
        pwm = 0.35f;
    } else {
        pwm = 0.0f;
    }

    if (millis() - millisLauncher > TS_ENCODER) {
        motorLauncher.speed(pwm);

        Serial.print("Launcher - pulse: ");
        Serial.print(encoderLauncher.getCount());
        Serial.print(" | current: ");
        Serial.print(-encoderLauncher.getCount() * (100 / 22));
        Serial.print(" | launcher ON: ");
        Serial.print(localLauncher);
        Serial.print(" | pwm: ");
        Serial.print(pwm);

        millisLauncher = millis();
    }
}



//======== Section LED Control Function ========//
void led_blink(LED_COLOR_STATE ledColorState) {
    if (millis() - millisLed >= TS_LEDBLINK) {
        millisLed = millis();
        ledState = !ledState;
        
        if (ledState) {
            if (ledColorState == LED_COLOR_STATE::RED) {
                led.setPixelColor(0, led.Color(255, 0, 0)); // Red
            } else if (ledColorState == LED_COLOR_STATE::GREEN) {
                led.setPixelColor(0, led.Color(0, 255, 0)); // Green
            } else if (ledColorState == LED_COLOR_STATE::BLUE) {
                led.setPixelColor(0, led.Color(0, 0, 255)); // Blue
            } else {
                led.setPixelColor(0, led.Color(255, 255, 0)); // Yellow
            }
        } else {
            led.setPixelColor(0, led.Color(0, 0, 0)); // Off
        }
    }
    led.show();
}

//======== Section Helper Functions ========//
String getJoy(const String &data, int index) {
    int commaCount = 0;
    int start = 0;
    int end = -1;
    
    for (int i = 0; i < data.length(); i++) {
        if (data.charAt(i) == ',') {
            commaCount++;
            if (commaCount == index + 1) {
                end = i;
                break;
            }
            start = i + 1;
        }
    }
    
    if (index == commaCount && end == -1) {
        end = data.length();
    }
    
    if (end > start) {
        return data.substring(start, end);
    } else {
        return "";
    }
}

void onSerialReceive_testingAll() {
    while (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        data.trim();  // penting: hapus spasi, \r, newline

        Serial.print("[DEBUG] Received raw: ");
        Serial.println(data);

        // Tampilkan hex dari setiap karakter untuk debug mendalam
        Serial.print("[HEX] Data: ");
        for (int i = 0; i < data.length(); i++) {
            Serial.print((uint8_t)data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        //=== Tambahkan handler "launcher on" dan "launcher off" ===//
        if (data.equalsIgnoreCase("launcher on")) {
            if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
                launcherOn = true;
                xSemaphoreGive(setpointMutex);
            }
            Serial.println("[INFO] Launcher turned ON");
            return;
        } else if (data.equalsIgnoreCase("launcher off")) {
            if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
                launcherOn = false;
                xSemaphoreGive(setpointMutex);
            }
            Serial.println("[INFO] Launcher turned OFF");
            return;
        }



        //=== Dukungan format lama: "catcher 75" atau "elevation 300" ===//
        int spaceIndex = data.indexOf(' ');
        if (spaceIndex != -1) {
            String name = data.substring(0, spaceIndex);
            String valueStr = data.substring(spaceIndex + 1);
            float value = valueStr.toFloat();

            if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
                if (name.equalsIgnoreCase("catcher")) {
                    float percent = constrain(value, 0.0f, 100.0f); // input persen
                    positionCatcher = percent * CATCHER_MAX_PULSE / 100.0f;
                    Serial.println("[INFO] Catcher set via Serial");
                }
                else if (name.equalsIgnoreCase("elevation")) {
                    if (value >= 31.0f && value <= 63.5f) {
                        elevationShooter = map(value, 31.0f, 63.5f, 1173.0f, 0.0f); // dibalik
                        Serial.println("[INFO] Elevation set via Serial");
                    } else {
                        Serial.println("[ERROR] Elevation must be between 31 and 63.5 degrees!");
                    }
                }

                else if (name.equalsIgnoreCase("shooter")) {
                    targetShooterSpeed = constrain(value, 0.0f, 300.0f);  // batas aman
                    Serial.print("[INFO] Shooter speed set to ");
                    Serial.print(targetShooterSpeed);
                    Serial.println(" rad/s via Serial");
                }
                xSemaphoreGive(setpointMutex);
            }
        } else {
            if (data.length() > 0) {
                Serial.println("[ERROR] Unknown or invalid command. Use: 'launcher on', 'elevation 500', etc.");
            }
        }
    }
}


//======== Section RTOS Tasks ==================

// --- Hardware Control Task (Core 1) ---
void hardwareControlTask(void *param) {
    while (1) {
        // controlCatcher();
        // controlElevation();
        // controlLauncher(); 
        controlShooter();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


// --- PS3 Controller Communication Task (Core 0) ---
void ps3Task(void *param) {
    lastPS3CommandTime = millis(); // Initialize timeout timer
    
    while (1) {
        // Handle Serial communication (PC commands)
        onSerialReceive_testingAll();
        
        // Handle PS3 communication via SerialPort
        updateSetpointFromPS3();
        
        // Check for PS3 command timeout
        // if (millis() - lastPS3CommandTime > PS3_TIMEOUT_MS) {
        //     // No PS3 commands received for too long - stop the robot
        //     if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
        //         positionCatcher = 0.0f;
        //         // positionLauncher = 0.0f;
        //         elevationShooter = 0.0f;
        //         xSemaphoreGive(setpointMutex);
        //     }
        // }
        
        // Print debug information
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint >= 500) { // Print every 500ms
            if (xSemaphoreTake(setpointMutex, portMAX_DELAY) == pdTRUE) {
                // Serial.print("Setpoints - Catcher: ");
                // Serial.print(positionCatcher * 100.0f / CATCHER_MAX_PULSE, 1);
                // Serial.print("% | Pulse: ");
                // Serial.print(positionCatcher, 1);
                // Serial.print(" | Pulse Catcher: ");
                // Serial.print(encoderCatcher.getCount());
                // Serial.print(", Launcher: "); Serial.print(launcherOn ? "ON" : "OFF");
                // Serial.print(", Elevation: "); Serial.println(elevationShooter, 1);
                xSemaphoreGive(setpointMutex);
            }
            lastPrint = millis();
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS); // 20Hz update rate
    }
}

//======== Section Setup ========//
void setup() {



    shooter.attach(FALCON_KIRI, FALCON_KANAN);


    // Initialize hardware
    hardware_init();
    
    // Initialize serial communications
    Serial.begin(115200);
    SerialPort.begin(115200, SERIAL_8N1, RXD2, TXD2);
    
    // PID Shooter Setup
    pidShooter.setMaxIOutput(1.5);
    pidShooter.setOutputLimits(1);

    // Initialize proximity sensor pins
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLauncher.attachFullQuad(ENCODER_LAUNCHER_A, ENCODER_LAUNCHER_B);
    encoderElevation.attachFullQuad(ENCODER_ELEVATION_B, ENCODER_ELEVATION_A);
    encoderCatcher.attachFullQuad(ENCODER_CATCHER_A, ENCODER_CATCHER_B);
    encoderShooter.attachFullQuad(ENCODER_SHOOTER_A, ENCODER_SHOOTER_B);

    // Create mutex for setpoint protection
    setpointMutex = xSemaphoreCreateMutex();
    if (setpointMutex == NULL) {
        Serial.println("Failed to create mutex!");
        while(1); // Stop execution
    }
    
    // Create RTOS tasks
    xTaskCreatePinnedToCore(
        hardwareControlTask,    // Task function
        "HardwareControl",      // Task name
        4096,                   // Stack size
        NULL,                   // Parameters
        2,                      // Priority (higher)
        NULL,                   // Task handle
        1                       // Core 1 (dedicated to hardware control)
    );
    
    xTaskCreatePinnedToCore(
        ps3Task,                // Task function
        "PS3Task",              // Task name
        4096,                   // Stack size
        NULL,                   // Parameters
        1,                      // Priority (lower)
        NULL,                   // Task handle
        0                       // Core 0 (communication tasks)
    );
    
    Serial.println("FreeRTOS tasks started successfully!");
}

//======== Section Main Loop ========//
void loop() {
    // Empty - all functionality moved to RTOS tasks
    delay(10);
}
//balik pin encoder, motor driver catcher
// setpoint ke 5800
