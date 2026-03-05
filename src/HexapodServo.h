/**
 * @file HexapodServo.h
 * @brief Individual servo control with optional smooth motion
 */

#ifndef HEXAPOD_SERVO_H
#define HEXAPOD_SERVO_H

#include <Arduino.h>
#include "HexapodTypes.h"
#include "HexapodConfig.h"

namespace Hexapod {

/**
 * @brief Represents a single servo motor with smooth motion support
 * 
 * Supports two modes:
 * - Instant: setAngle() + getTargetAngle() for immediate positioning
 * - Smooth: setAngle() + updateInterpolation() + getCurrentAngle() for smooth motion
 */
class HexapodServo {
public:
    HexapodServo();
    
    void initialize(const char* id, uint8_t channel, bool reverse,
                    float minAngle, float maxAngle,
                    uint16_t minPulse, uint16_t maxPulse);
    
    /**
     * @brief Set target angle
     * @param angle Target angle in degrees
     */
    void setAngle(float angle);
    
    /**
     * @brief Set movement speed for smooth motion
     * @param speed Speed 0-100 (0=slowest, 100=instant)
     */
    void setSpeed(uint8_t speed);
    
    /**
     * @brief Start a smooth move to target angle
     * 
     * Call this to begin interpolation. Then call updateInterpolation()
     * repeatedly until isMoving() returns false.
     * 
     * @param angle Target angle in degrees
     * @param speed Movement speed 0-100 (affects duration)
     * @param durationMs Optional: override calculated duration (0 = auto)
     */
    void startMove(float angle, uint8_t speed, uint16_t durationMs = 0);
    
    /**
     * @brief Update interpolation state
     * 
     * Call this frequently (e.g., every 20ms) for smooth motion.
     * Updates m_currentAngle based on elapsed time.
     * 
     * @return true if position changed, false if already at target
     */
    bool updateInterpolation();
    
    /**
     * @brief Check if servo is still moving to target
     * @return true if interpolation is in progress
     */
    bool isMoving() const;
    
    /**
     * @brief Immediately set current position to target (skip interpolation)
     */
    void snapToTarget();
    
    /**
     * @brief Sync current angle with target angle
     * 
     * Call this after using instant motion (setAngle + update) before
     * switching to smooth motion, so the servo knows where it actually is.
     */
    void syncCurrentToTarget() { m_currentAngle = m_targetAngle; }
    
    // Getters
    const char* getId() const { return m_id; }
    uint8_t getChannel() const { return m_channel; }
    float getTargetAngle() const { return m_targetAngle; }
    float getCurrentAngle() const { return m_currentAngle; }
    uint8_t getTargetSpeed() const { return m_speed; }
    bool isReverse() const { return m_reverse; }
    uint16_t getPulseWidth() const;
    
    /**
     * @brief Get current angle for output (accounting for reverse)
     * 
     * Use this for smooth motion - returns interpolated position.
     */
    float getCurrentOutputAngle() const { return m_reverse ? -m_currentAngle : m_currentAngle; }
    
    /**
     * @brief Get target angle for output (accounting for reverse)
     * 
     * Use this for instant motion - returns target position.
     */
    float getTargetOutputAngle() const { return m_reverse ? -m_targetAngle : m_targetAngle; }

private:
    // Basic servo properties
    const char* m_id;
    uint8_t m_channel;
    bool m_reverse;
    float m_minAngle;
    float m_maxAngle;
    uint16_t m_minPulse;
    uint16_t m_maxPulse;
    
    // Position state
    float m_targetAngle;     ///< Where we want to go
    float m_currentAngle;    ///< Where we are now (for interpolation)
    uint8_t m_speed;         ///< Movement speed setting
    
    // Interpolation state
    float m_startAngle;           ///< Angle at start of move
    unsigned long m_moveStartTime; ///< millis() when move started
    unsigned long m_moveDuration;  ///< Duration of move in ms (0 = not moving)
    
    /**
     * @brief Easing function for smooth acceleration/deceleration
     * @param t Progress 0.0 to 1.0
     * @return Eased progress 0.0 to 1.0
     */
    static float easeInOutQuad(float t);
};

} // namespace Hexapod

#endif // HEXAPOD_SERVO_H
