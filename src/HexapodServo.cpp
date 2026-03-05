/**
 * @file HexapodServo.cpp
 * @brief Implementation of HexapodServo class with smooth motion
 */

#include "HexapodServo.h"

namespace Hexapod {

HexapodServo::HexapodServo()
    : m_id("")
    , m_channel(0)
    , m_reverse(false)
    , m_minAngle(-90.0f)
    , m_maxAngle(90.0f)
    , m_minPulse(500)
    , m_maxPulse(2500)
    , m_targetAngle(0.0f)
    , m_currentAngle(0.0f)
    , m_speed(100)
    , m_startAngle(0.0f)
    , m_moveStartTime(0)
    , m_moveDuration(0)
{
}

void HexapodServo::initialize(const char* id, uint8_t channel, bool reverse,
                               float minAngle, float maxAngle,
                               uint16_t minPulse, uint16_t maxPulse) {
    m_id = id;
    m_channel = channel;
    m_reverse = reverse;
    m_minAngle = minAngle;
    m_maxAngle = maxAngle;
    m_minPulse = minPulse;
    m_maxPulse = maxPulse;
    m_targetAngle = 0.0f;
    m_currentAngle = 0.0f;
    m_speed = 100;
    m_startAngle = 0.0f;
    m_moveStartTime = 0;
    m_moveDuration = 0;
}

void HexapodServo::setAngle(float angle) {
    m_targetAngle = constrain(angle, m_minAngle, m_maxAngle);
}

void HexapodServo::setSpeed(uint8_t speed) {
    m_speed = constrain(speed, (uint8_t)0, (uint8_t)100);
}

void HexapodServo::startMove(float angle, uint8_t speed, uint16_t durationMs) {
    m_targetAngle = constrain(angle, m_minAngle, m_maxAngle);
    m_speed = constrain(speed, (uint8_t)0, (uint8_t)100);
    m_startAngle = m_currentAngle;
    m_moveStartTime = millis();
    
    if (durationMs > 0) {
        // Use provided duration
        m_moveDuration = durationMs;
    } else if (speed >= 100) {
        // Instant move
        m_moveDuration = 0;
        m_currentAngle = m_targetAngle;
    } else {
        // Calculate duration based on distance and speed
        float distance = m_targetAngle - m_startAngle;
        if (distance < 0) distance = -distance;  // abs
        
        // Speed mapping:
        // speed 0  -> 30 degrees/second (very slow)
        // speed 50 -> 180 degrees/second (medium)
        // speed 99 -> 500 degrees/second (fast)
        float degreesPerSec;
        if (speed < 50) {
            degreesPerSec = 30.0f + (speed / 50.0f) * 150.0f;  // 30 to 180
        } else {
            degreesPerSec = 180.0f + ((speed - 50) / 49.0f) * 320.0f;  // 180 to 500
        }
        
        m_moveDuration = (unsigned long)(distance / degreesPerSec * 1000.0f);
        
        // Minimum duration to avoid jitter
        if (m_moveDuration < 20) {
            m_moveDuration = 0;  // Just snap to position
            m_currentAngle = m_targetAngle;
        }
    }
}

bool HexapodServo::updateInterpolation() {
    // Already at target?
    if (m_moveDuration == 0) {
        return false;
    }
    
    unsigned long elapsed = millis() - m_moveStartTime;
    
    // Move complete?
    if (elapsed >= m_moveDuration) {
        m_currentAngle = m_targetAngle;
        m_moveDuration = 0;  // Mark as complete
        return true;  // Position changed (final update)
    }
    
    // Calculate progress (0.0 to 1.0)
    float t = (float)elapsed / (float)m_moveDuration;
    
    // Apply easing for smooth acceleration/deceleration
    t = easeInOutQuad(t);
    
    // Interpolate
    m_currentAngle = m_startAngle + (m_targetAngle - m_startAngle) * t;
    
    return true;  // Position changed
}

bool HexapodServo::isMoving() const {
    return m_moveDuration > 0;
}

void HexapodServo::snapToTarget() {
    m_currentAngle = m_targetAngle;
    m_moveDuration = 0;
}

uint16_t HexapodServo::getPulseWidth() const {
    float angle = m_reverse ? -m_currentAngle : m_currentAngle;
    return map(angle * 100, m_minAngle * 100, m_maxAngle * 100, m_minPulse, m_maxPulse);
}

float HexapodServo::easeInOutQuad(float t) {
    // Attempt to avoid calculation for exact endpoints
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    
    // Quadratic ease in-out: smooth start and stop
    if (t < 0.5f) {
        return 2.0f * t * t;
    } else {
        float t2 = -2.0f * t + 2.0f;
        return 1.0f - (t2 * t2) / 2.0f;
    }
}

} // namespace Hexapod
