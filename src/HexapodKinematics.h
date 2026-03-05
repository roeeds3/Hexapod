/**
 * @file HexapodKinematics.h
 * @brief Main include file for the HexapodKinematics library
 * 
 * Easy-to-use hexapod library with built-in servo drivers.
 * All configuration is done through constructor parameters.
 * 
 * Supported drivers:
 *   0 = PCA9685 (Adafruit PWM Servo Driver)
 *   1 = Pololu Maestro
 *   2 = Direct PWM (Arduino Servo library)
 * 
 * Example:
 * @code
 * // Create hexapod with PCA9685 driver and default dimensions
 * Hexapod::Hexapod robot(0);
 * 
 * // Or with custom dimensions
 * Hexapod::HexapodConfig config;
 * config.thighLength = 80;
 * config.footLength = 120;
 * Hexapod::Hexapod robot(0, config);
 * @endcode
 */

#ifndef HEXAPOD_KINEMATICS_H
#define HEXAPOD_KINEMATICS_H

#include <Arduino.h>

// ============================================================================
// DRIVER SELECTION - Uncomment the drivers you want to use
// ============================================================================
// By default, all drivers are enabled. Comment out any you don't need
// to reduce code size.

#define HEXAPOD_ENABLE_PCA9685      // Requires: Adafruit PWM Servo Driver Library
//#define HEXAPOD_ENABLE_MAESTRO      // Requires: PololuMaestro library
//#define HEXAPOD_ENABLE_DIRECT_PWM   // Requires: Servo library (built-in)

// ============================================================================
// INCLUDES
// ============================================================================

// Core types (Position, KeyFrame, LegId, DriverType) - SINGLE source of truth
#include "HexapodTypes.h"

// Configuration structure
#include "HexapodConfig.h"

// Servo abstraction
#include "HexapodServo.h"

// Leg with inverse kinematics
#include "HexapodLeg.h"

// Predefined poses and gaits
#include "HexapodPoses.h"

// Main controller class
#include "Hexapod.h"

// NOTE: Position.h and KeyFrame.h are NOT included here because their
// types are already defined in HexapodTypes.h. Including them would
// cause duplicate definition errors.

#endif // HEXAPOD_KINEMATICS_H
