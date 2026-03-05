/**
 * @file HexapodConfig.h
 * @brief Configuration structure for hexapod dimensions and settings
 * 
 * All dimensions are in millimeters. Users create a HexapodConfig struct,
 * modify any values they need, and pass it to the Hexapod constructor.
 */

#ifndef HEXAPOD_CONFIG_H
#define HEXAPOD_CONFIG_H

#include <Arduino.h>

namespace Hexapod {

/**
 * @brief Complete configuration for a hexapod robot
 * 
 * Create an instance, modify values as needed, then pass to Hexapod constructor.
 * All values have sensible defaults for a typical hexapod.
 * 
 * Example:
 * @code
 * HexapodConfig config;
 * config.thighLength = 80;      // My robot has shorter thighs
 * config.footLength = 130;      // And longer feet
 * config.footFBDistance = 140;  // Front/back feet closer together
 * 
 * Hexapod robot(DRIVER_PCA9685, config);
 * @endcode
 */
struct HexapodConfig {
    // ========================================================================
    // Leg Segment Lengths (in millimeters)
    // ========================================================================
    
    int16_t thighLength = 86;       ///< FEMUR: Distance from knee servo to foot servo
    int16_t footLength = 160;       ///< TIBIA: Distance from foot servo to toe tip
    int16_t hipToKneeLength = 38;   ///< COXA: Distance from hip servo to knee servo
    
    // ========================================================================
    // Body Dimensions - Leg Anchor Points (in mm from body center)
    // ========================================================================
    
    // Matching Hexapod_Arduino.ino leg origins:
    // RF: (100.94, -57.11), RM: (0, -105.72), RR: (-100.94, -57.11)
    // LR: (-100.94, 57.11), LM: (0, 105.72), LF: (100.94, 57.11)
    int16_t anchorLFrontX = 101;    ///< Left front hip X position
    int16_t anchorLFrontY = 57;     ///< Left front hip Y position
    int16_t anchorLMiddleX = 0;     ///< Left middle hip X position
    int16_t anchorLMiddleY = 106;   ///< Left middle hip Y position
    int16_t anchorLBackX = -101;    ///< Left back hip X position
    int16_t anchorLBackY = 57;      ///< Left back hip Y position
    // Right side mirrors left side (Y is negated automatically)
    
    // ========================================================================
    // Default Foot Positions (in mm from body center)
    // Calculated from FK with neutral angles to match original code behavior
    // ========================================================================
    
    int16_t footFBDistance = 180;   ///< X distance for front/back feet from center
    int16_t footWidthFB = 130;      ///< Y distance for front/back feet from center
    int16_t footWidthM = 210;       ///< Y distance for middle feet from center
    
    // ========================================================================
    // Height Settings (Z coordinates, negative = below body)
    // Matching Hexapod_Arduino.ino: DEFAULT_Z = -40, STEP_HEIGHT = 100
    // ========================================================================
    
    int16_t minHomeHeight = -40;    ///< Minimum standing height (highest position)
    int16_t maxHomeHeight = -120;   ///< Maximum standing height (lowest position)
    int16_t footUpOffset = 100;     ///< How high to lift feet when stepping (STEP_HEIGHT)
    int16_t footWalkX = 60;         ///< Forward/backward stride distance (STEP_LENGTH)
    
    // ========================================================================
    // Rotation Settings
    // ========================================================================
    
    float rotationAngleDeg = 15.0f; ///< Rotation angle per step in degrees
    
    // ========================================================================
    // Animation Settings
    // ========================================================================
    
    int16_t animLiftHeight = 40;    ///< Height to lift legs during animations
    int16_t animReachDistance = 60; ///< Forward reach distance for animations
    int16_t animWidthOffset = 50;   ///< Side offset for wave animations
    int16_t animExtraWidth = 20;    ///< Additional width offset for arm animations
    int16_t animExtraLift = 10;     ///< Additional lift for front leg animations
    
    // ========================================================================
    // Servo Settings
    // ========================================================================
    
    float minServoAngle = -90.0f;   ///< Minimum servo angle in degrees
    float maxServoAngle = 90.0f;    ///< Maximum servo angle in degrees
    uint16_t servoMinPulse = 500;   ///< Minimum pulse width in microseconds
    uint16_t servoMaxPulse = 2500;  ///< Maximum pulse width in microseconds
    
    // ========================================================================
    // Inverse Kinematics Calibration
    // ========================================================================
    
    /**
     * @brief Foot servo angle offset for calibration
     * 
     * This compensates for mechanical offset between servo horn position
     * and actual foot linkage. Adjust if foot doesn't point straight down
     * when at calculated "0" position.
     * 
     * Positive values rotate foot forward, negative rotates backward.
     */
    float footAngleOffset = -14.0f;
    
    // ========================================================================
    // Servo Channel Mapping
    // Each leg has 3 servos: foot, knee, hip
    // Default mapping for 18-channel controller
    // NOTE: Reverse flags are all false by default since the IK outputs
    // angles that match direct servo control. Adjust if your servos are mounted differently.
    // ========================================================================
    
    // Left Front (channels 0-2)
    uint8_t chLFFoot = 0;
    uint8_t chLFKnee = 1;
    uint8_t chLFHip = 2;
    bool revLFFoot = false;
    bool revLFKnee = false;
    bool revLFHip = false;
    
    // Right Front (channels 3-5)
    uint8_t chRFFoot = 3;
    uint8_t chRFKnee = 4;
    uint8_t chRFHip = 5;
    bool revRFFoot = false;
    bool revRFKnee = false;
    bool revRFHip = false;
    
    // Left Middle (channels 6-8)
    uint8_t chLMFoot = 6;
    uint8_t chLMKnee = 7;
    uint8_t chLMHip = 8;
    bool revLMFoot = false;
    bool revLMKnee = false;
    bool revLMHip = false;
    
    // Right Middle (channels 9-11)
    uint8_t chRMFoot = 9;
    uint8_t chRMKnee = 10;
    uint8_t chRMHip = 11;
    bool revRMFoot = false;
    bool revRMKnee = false;
    bool revRMHip = false;
    
    // Left Back (channels 12-14)
    uint8_t chLBFoot = 12;
    uint8_t chLBKnee = 13;
    uint8_t chLBHip = 14;
    bool revLBFoot = false;
    bool revLBKnee = false;
    bool revLBHip = false;
    
    // Right Back (channels 15-17)
    uint8_t chRBFoot = 15;
    uint8_t chRBKnee = 16;
    uint8_t chRBHip = 17;
    bool revRBFoot = false;
    bool revRBKnee = false;
    bool revRBHip = false;
    
    // ========================================================================
    // Driver-Specific Settings
    // ========================================================================
    
    // PCA9685 settings
    // For 18 servos, you need 2 PCA9685 boards (each has 16 channels)
    // Board 1 handles channels 0-15, Board 2 handles channels 16+
    // Set address jumpers on the second board to change its address
    uint8_t pca9685Address = 0x40;       ///< I2C address for first PCA9685
    uint8_t pca9685Address2 = 0x41;      ///< I2C address for second PCA9685 (channels 16+)
    bool useDualPCA9685 = true;          ///< Enable second PCA9685 for channels 16+
    
    // I2C Pin Configuration (ESP32 only - other boards use fixed pins)
    // Set to -1 to use default pins for your board
    // ESP32 defaults: SDA=21, SCL=22
    int8_t i2cSdaPin = -1;               ///< Custom SDA pin (-1 = use default)
    int8_t i2cSclPin = -1;               ///< Custom SCL pin (-1 = use default)
    
    // Pololu Maestro settings (uses Serial port specified in constructor)
    
    // Direct PWM pin mapping (for DRIVER_DIRECT_PWM)
    // Maps channel numbers to Arduino pins
    uint8_t directPwmPins[18] = {
        2, 3, 4, 5, 6, 7,           // Channels 0-5
        8, 9, 10, 11, 12, 13,       // Channels 6-11
        22, 23, 24, 25, 26, 27      // Channels 12-17 (for Mega)
    };
};

} // namespace Hexapod

#endif // HEXAPOD_CONFIG_H
