/**
 * @file HexapodTypes.h
 * @brief Common types used throughout the library
 * 
 * This is the SINGLE source of truth for Position and KeyFrame types.
 * Do not define these elsewhere to avoid redefinition errors.
 */

#ifndef HEXAPOD_TYPES_H
#define HEXAPOD_TYPES_H

#include <Arduino.h>

namespace Hexapod {

// ============================================================================
// Driver Type Enumeration
// ============================================================================

enum DriverType : uint8_t {
    DRIVER_PCA9685 = 0,      ///< Adafruit PCA9685 PWM driver
    DRIVER_MAESTRO = 1,      ///< Pololu Maestro servo controller
    DRIVER_DIRECT_PWM = 2    ///< Direct PWM using Arduino Servo library
};

// ============================================================================
// Leg Identifiers
// ============================================================================

enum class LegId : uint8_t {
    LFRONT = 0,   ///< Left front leg
    LMIDDLE = 1,  ///< Left middle leg
    LBACK = 2,    ///< Left back leg
    RFRONT = 3,   ///< Right front leg
    RMIDDLE = 4,  ///< Right middle leg
    RBACK = 5     ///< Right back leg
};

// ============================================================================
// Position Structure
// ============================================================================

/**
 * @brief Represents a 3D position in millimeters relative to body center
 * 
 * Coordinate system:
 * - X: Forward is positive, backward is negative
 * - Y: Left is positive, right is negative
 * - Z: Up is positive, down is negative (ground level is negative)
 */
struct Position {
    int16_t x;  ///< X coordinate in mm (forward/backward)
    int16_t y;  ///< Y coordinate in mm (left/right)
    int16_t z;  ///< Z coordinate in mm (up/down)
    
    Position() : x(0), y(0), z(0) {}
    Position(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
    
    Position operator+(const Position& other) const {
        return Position(x + other.x, y + other.y, z + other.z);
    }
    
    Position operator-(const Position& other) const {
        return Position(x - other.x, y - other.y, z - other.z);
    }
    
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
};

// ============================================================================
// KeyFrame Structure
// ============================================================================

/**
 * @brief Represents a complete pose with all six leg positions
 */
struct KeyFrame {
    Position LFront;
    Position LMiddle;
    Position LBack;
    Position RFront;
    Position RMiddle;
    Position RBack;
    
    KeyFrame() = default;
    
    KeyFrame(Position lf, Position lm, Position lb, 
             Position rf, Position rm, Position rb)
        : LFront(lf), LMiddle(lm), LBack(lb),
          RFront(rf), RMiddle(rm), RBack(rb) {}
    
    /**
     * @brief Interpolate between two keyframes
     * @param target Target keyframe
     * @param t Interpolation factor (0.0 to 1.0)
     */
    KeyFrame lerp(const KeyFrame& target, float t) const {
        t = constrain(t, 0.0f, 1.0f);
        KeyFrame result;
        // Helper lambda for proper rounding (works for both positive and negative)
        auto roundToInt16 = [](float v) -> int16_t {
            return (int16_t)(v >= 0 ? (v + 0.5f) : (v - 0.5f));
        };
        result.LFront = Position(
            roundToInt16(LFront.x + (target.LFront.x - LFront.x) * t),
            roundToInt16(LFront.y + (target.LFront.y - LFront.y) * t),
            roundToInt16(LFront.z + (target.LFront.z - LFront.z) * t));
        result.LMiddle = Position(
            roundToInt16(LMiddle.x + (target.LMiddle.x - LMiddle.x) * t),
            roundToInt16(LMiddle.y + (target.LMiddle.y - LMiddle.y) * t),
            roundToInt16(LMiddle.z + (target.LMiddle.z - LMiddle.z) * t));
        result.LBack = Position(
            roundToInt16(LBack.x + (target.LBack.x - LBack.x) * t),
            roundToInt16(LBack.y + (target.LBack.y - LBack.y) * t),
            roundToInt16(LBack.z + (target.LBack.z - LBack.z) * t));
        result.RFront = Position(
            roundToInt16(RFront.x + (target.RFront.x - RFront.x) * t),
            roundToInt16(RFront.y + (target.RFront.y - RFront.y) * t),
            roundToInt16(RFront.z + (target.RFront.z - RFront.z) * t));
        result.RMiddle = Position(
            roundToInt16(RMiddle.x + (target.RMiddle.x - RMiddle.x) * t),
            roundToInt16(RMiddle.y + (target.RMiddle.y - RMiddle.y) * t),
            roundToInt16(RMiddle.z + (target.RMiddle.z - RMiddle.z) * t));
        result.RBack = Position(
            roundToInt16(RBack.x + (target.RBack.x - RBack.x) * t),
            roundToInt16(RBack.y + (target.RBack.y - RBack.y) * t),
            roundToInt16(RBack.z + (target.RBack.z - RBack.z) * t));
        return result;
    }
};

} // namespace Hexapod

#endif // HEXAPOD_TYPES_H
