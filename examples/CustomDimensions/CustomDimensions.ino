/**
 * CustomDimensions.ino
 * 
 * Demonstrates how to configure a hexapod with custom dimensions
 * and servo channel mappings - all from your Arduino sketch!
 * 
 * This example shows how to customize:
 * - Leg segment lengths (thigh, foot, hip-to-knee)
 * - Body dimensions (leg anchor positions)
 * - Foot positions and stride settings
 * - Servo channel assignments and directions
 * - Driver-specific settings (I2C address, pins, etc.)
 */

#include <HexapodKinematics.h>

// ============================================================================
// CONFIGURATION - Customize everything here!
// ============================================================================

void setupConfig(Hexapod::HexapodConfig& config) {
    // ========================================================================
    // LEG SEGMENT LENGTHS (in millimeters)
    // Measure your robot's leg segments
    // ========================================================================
    
    config.thighLength = 84;       // From knee servo horn to foot servo horn
    config.footLength = 127;       // From foot servo horn to toe tip
    config.hipToKneeLength = 28;   // From hip servo horn to knee servo horn
    
    // ========================================================================
    // BODY DIMENSIONS - Leg Anchor Points
    // Where each hip joint connects to the body (mm from center)
    // Only need to set left side - right side is mirrored automatically
    // ========================================================================
    
    config.anchorLFrontX = 60;     // Left front hip X (forward)
    config.anchorLFrontY = 100;    // Left front hip Y (to the left)
    config.anchorLMiddleX = 0;     // Left middle hip X (at center)
    config.anchorLMiddleY = 120;   // Left middle hip Y (widest)
    config.anchorLBackX = -60;     // Left back hip X (backward)
    config.anchorLBackY = 100;     // Left back hip Y
    
    // ========================================================================
    // DEFAULT FOOT POSITIONS (in mm from body center)
    // Where feet rest in home position
    // ========================================================================
    
    config.footFBDistance = 150;   // Front/back feet X distance from center
    config.footWidthFB = 123;      // Front/back feet Y distance (sideways)
    config.footWidthM = 177;       // Middle feet Y distance (usually widest)
    
    // ========================================================================
    // HEIGHT AND WALKING SETTINGS
    // ========================================================================
    
    config.minHomeHeight = -60;    // Highest standing position (least negative)
    config.maxHomeHeight = -140;   // Lowest standing position (most negative)
    config.footUpOffset = 30;      // How high to lift feet when stepping
    config.footWalkX = 40;         // How far forward/back each step moves
    
    // ========================================================================
    // SERVO SETTINGS
    // ========================================================================
    
    config.minServoAngle = -90.0f;  // Minimum servo angle
    config.maxServoAngle = 90.0f;   // Maximum servo angle
    config.servoMinPulse = 500;     // Pulse width at min angle (microseconds)
    config.servoMaxPulse = 2500;    // Pulse width at max angle (microseconds)
    
    // ========================================================================
    // SERVO CHANNEL MAPPING
    // Assign each servo to a channel on your controller
    // Set reverse=true if servo rotates the wrong direction
    // ========================================================================
    
    // Left Front leg (channels 0, 1, 2)
    config.chLFFoot = 0;    config.revLFFoot = true;
    config.chLFKnee = 1;    config.revLFKnee = false;
    config.chLFHip = 2;     config.revLFHip = true;
    
    // Right Front leg (channels 3, 4, 5)
    config.chRFFoot = 3;    config.revRFFoot = false;
    config.chRFKnee = 4;    config.revRFKnee = true;
    config.chRFHip = 5;     config.revRFHip = true;
    
    // Left Middle leg (channels 6, 7, 8)
    config.chLMFoot = 6;    config.revLMFoot = true;
    config.chLMKnee = 7;    config.revLMKnee = false;
    config.chLMHip = 8;     config.revLMHip = true;
    
    // Right Middle leg (channels 9, 10, 11)
    config.chRMFoot = 9;    config.revRMFoot = false;
    config.chRMKnee = 10;   config.revRMKnee = true;
    config.chRMHip = 11;    config.revRMHip = true;
    
    // Left Back leg (channels 12, 13, 14)
    config.chLBFoot = 12;   config.revLBFoot = true;
    config.chLBKnee = 13;   config.revLBKnee = false;
    config.chLBHip = 14;    config.revLBHip = true;
    
    // Right Back leg (channels 15, 16, 17)
    config.chRBFoot = 15;   config.revRBFoot = false;
    config.chRBKnee = 16;   config.revRBKnee = true;
    config.chRBHip = 17;    config.revRBHip = true;
    
    // ========================================================================
    // DRIVER-SPECIFIC SETTINGS
    // ========================================================================
    
    // PCA9685 I2C address (default 0x40, change if using address jumpers)
    config.pca9685Address = 0x40;
    
    // Direct PWM pin mapping (only used with DRIVER_DIRECT_PWM)
    // Maps channel numbers to Arduino pins
    // Customize for your board (this example is for Arduino Mega)
    config.directPwmPins[0] = 2;
    config.directPwmPins[1] = 3;
    config.directPwmPins[2] = 4;
    config.directPwmPins[3] = 5;
    config.directPwmPins[4] = 6;
    config.directPwmPins[5] = 7;
    config.directPwmPins[6] = 8;
    config.directPwmPins[7] = 9;
    config.directPwmPins[8] = 10;
    config.directPwmPins[9] = 11;
    config.directPwmPins[10] = 12;
    config.directPwmPins[11] = 13;
    config.directPwmPins[12] = 22;
    config.directPwmPins[13] = 23;
    config.directPwmPins[14] = 24;
    config.directPwmPins[15] = 25;
    config.directPwmPins[16] = 26;
    config.directPwmPins[17] = 27;
}

// ============================================================================
// DRIVER SELECTION
// ============================================================================

// Choose your driver:
//   0 = PCA9685 (Adafruit PWM Servo Driver) - Most common
//   1 = Pololu Maestro (connect to Serial1)
//   2 = Direct PWM (Arduino Servo library)
const uint8_t DRIVER_TYPE = 0;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Hexapod::HexapodConfig myConfig;
Hexapod::Hexapod* robot = nullptr;

// For Pololu Maestro, uncomment this line:
// Hexapod::Hexapod* robot = new Hexapod::Hexapod(1, myConfig, Serial1);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println(F("HexapodKinematics - Custom Dimensions Example"));
    Serial.println(F("============================================="));
    Serial.println();
    
    // Apply our custom configuration
    setupConfig(myConfig);
    
    // Create robot with custom config
    robot = new Hexapod::Hexapod(DRIVER_TYPE, myConfig);
    
    // Check allocation succeeded
    if (robot == nullptr) {
        Serial.println(F("ERROR: Failed to allocate memory for robot!"));
        while (1) delay(1000);  // Halt
    }
    
    // Initialize
    robot->begin();
    
    // Verify initialization succeeded
    if (!robot->isInitialized()) {
        Serial.println(F("ERROR: Robot initialization failed!"));
        Serial.println(F("Check driver type and connections."));
        while (1) delay(1000);  // Halt
    }
    
    // Print configuration summary
    printConfig();
    
    // Start in home position
    robot->setHeight(5);
    robot->goHome(50);
    robot->update();
    
    Serial.println();
    Serial.println(F("Commands: h=home, s=sleep, w=walk, r=rotate, +/- height"));
    Serial.println(F("Ready!"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

unsigned long lastStep = 0;
bool walking = false;

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'h': robot->goHome(50); robot->update(); break;
            case 's': robot->goSleep(50); robot->update(); break;
            case 'w': walking = !walking; break;
            case 'r': robot->rotateClockwise(50); robot->update(); break;
            case '+': 
                if (robot->getHeight() < 9) {
                    robot->setHeight(robot->getHeight() + 1);
                    robot->goHome(50); 
                    robot->update();
                }
                break;
            case '-':
                if (robot->getHeight() > 0) {
                    robot->setHeight(robot->getHeight() - 1);
                    robot->goHome(50);
                    robot->update();
                }
                break;
        }
    }
    
    if (walking && millis() - lastStep > 300) {
        lastStep = millis();
        robot->walkForward(50);
        robot->update();
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void printConfig() {
    Serial.println(F("Configuration Summary:"));
    Serial.println(F("----------------------"));
    
    Serial.print(F("Leg lengths (mm): thigh="));
    Serial.print(myConfig.thighLength);
    Serial.print(F(", foot="));
    Serial.print(myConfig.footLength);
    Serial.print(F(", hip-knee="));
    Serial.println(myConfig.hipToKneeLength);
    
    Serial.print(F("Foot positions: FB="));
    Serial.print(myConfig.footFBDistance);
    Serial.print(F(", widthFB="));
    Serial.print(myConfig.footWidthFB);
    Serial.print(F(", widthM="));
    Serial.println(myConfig.footWidthM);
    
    Serial.print(F("Height range: "));
    Serial.print(myConfig.minHomeHeight);
    Serial.print(F(" to "));
    Serial.println(myConfig.maxHomeHeight);
    
    Serial.print(F("Driver: "));
    switch (DRIVER_TYPE) {
        case 0: Serial.print(F("PCA9685 @ 0x")); Serial.println(myConfig.pca9685Address, HEX); break;
        case 1: Serial.println(F("Pololu Maestro")); break;
        case 2: Serial.println(F("Direct PWM")); break;
    }
}
