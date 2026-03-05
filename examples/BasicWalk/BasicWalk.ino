/**
 * BasicWalk.ino
 * 
 * Simple hexapod walking example using HexapodKinematics library.
 * Uses PCA9685 servo driver with default configuration.
 * 
 * Hardware:
 * - Arduino (Uno, Mega, ESP32, etc.)
 * - PCA9685 PWM driver board
 * - 18 servo motors connected to channels 0-17
 * 
 * Wiring:
 * - PCA9685 SDA -> Arduino SDA
 * - PCA9685 SCL -> Arduino SCL
 * - PCA9685 VCC -> 5V
 * - PCA9685 GND -> GND
 * - Servo power (V+) -> External 5-6V power supply
 * 
 * Required Libraries:
 * - Adafruit PWM Servo Driver Library
 */

#include <HexapodKinematics.h>

// ============================================================================
// DRIVER SELECTION
// Change this value to use different servo drivers:
//   0 = PCA9685 (Adafruit PWM driver)
//   1 = Pololu Maestro
//   2 = Direct PWM (Arduino Servo library)
// ============================================================================
const uint8_t DRIVER_TYPE = 0;

// Create hexapod with selected driver and default dimensions
Hexapod::Hexapod robot(DRIVER_TYPE);

// Timing
unsigned long lastStepTime = 0;
const unsigned long STEP_INTERVAL = 300;  // ms between steps

// State
bool isWalking = false;
uint8_t walkSpeed = 50;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println(F("HexapodKinematics - Basic Walk Example"));
    Serial.println(F("======================================"));
    Serial.println();
    Serial.println(F("Commands:"));
    Serial.println(F("  h - Home position"));
    Serial.println(F("  s - Sleep position"));
    Serial.println(F("  w - Toggle walking"));
    Serial.println(F("  r - Toggle rotation"));
    Serial.println(F("  + - Height up"));
    Serial.println(F("  - - Height down"));
    Serial.println(F("  z - Zero servos"));
    Serial.println();
    
    // Initialize robot
    robot.begin();
    
    // Verify initialization succeeded
    if (!robot.isInitialized()) {
        Serial.println(F("ERROR: Robot initialization failed!"));
        Serial.println(F("Check driver type and hardware connections."));
        while (1) delay(1000);  // Halt
    }
    
    // Set initial height (0-9, where 5 is medium)
    robot.setHeight(5);
    
    // Move to home position
    robot.goHome(walkSpeed);
    robot.update();
    
    Serial.println(F("Ready!"));
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }
    
    // Walking animation
    if (isWalking) {
        unsigned long now = millis();
        if (now - lastStepTime >= STEP_INTERVAL) {
            lastStepTime = now;
            robot.walkForward(walkSpeed);
            robot.update();
        }
    }
}

void handleCommand(char cmd) {
    switch (cmd) {
        case 'h':
        case 'H':
            Serial.println(F("Home position"));
            isWalking = false;
            robot.goHome(walkSpeed);
            robot.update();
            break;
            
        case 's':
        case 'S':
            Serial.println(F("Sleep position"));
            isWalking = false;
            robot.goSleep(walkSpeed);
            robot.update();
            break;
            
        case 'w':
        case 'W':
            isWalking = !isWalking;
            Serial.print(F("Walking: "));
            Serial.println(isWalking ? F("ON") : F("OFF"));
            if (!isWalking) {
                robot.goHome(walkSpeed);
                robot.update();
            }
            break;
            
        case 'r':
        case 'R':
            Serial.println(F("Rotate step"));
            robot.rotateClockwise(walkSpeed);
            robot.update();
            break;
            
        case '+':
        case '=':
            if (robot.getHeight() < 9) {
                robot.setHeight(robot.getHeight() + 1);
                Serial.print(F("Height: "));
                Serial.println(robot.getHeight());
                if (!isWalking) {
                    robot.goHome(walkSpeed);
                    robot.update();
                }
            }
            break;
            
        case '-':
        case '_':
            if (robot.getHeight() > 0) {
                robot.setHeight(robot.getHeight() - 1);
                Serial.print(F("Height: "));
                Serial.println(robot.getHeight());
                if (!isWalking) {
                    robot.goHome(walkSpeed);
                    robot.update();
                }
            }
            break;
            
        case 'z':
        case 'Z':
            Serial.println(F("Zero servos"));
            isWalking = false;
            robot.zeroAllServos(walkSpeed);
            robot.update();
            break;
            
        case 'i':
        case 'I':
            Serial.println(F("Identify legs"));
            robot.identifyAllLegs(walkSpeed);
            robot.update();
            break;
    }
}
