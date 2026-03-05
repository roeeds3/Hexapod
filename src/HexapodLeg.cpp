/**
 * @file HexapodLeg.cpp
 * @brief Implementation of HexapodLeg with inverse kinematics
 */

#include "HexapodLeg.h"
#include <math.h>

namespace Hexapod {

static const float RAD2DEG = 180.0f / M_PI;

HexapodLeg::HexapodLeg()
    : m_legId(LegId::LFRONT)
    , m_anchorX(0)
    , m_anchorY(0)
    , m_anchorZ(0)
    , m_anchorAngleDeg(0)
    , m_thighLength(84)
    , m_footLength(127)
    , m_hipToKneeLength(28)
    , m_footAngleOffset(-14.0f)
    , m_footPosX(0)
    , m_footPosY(0)
    , m_footPosZ(0)
    , m_initialized(false)
{
}

void HexapodLeg::initialize(LegId id, int16_t anchorX, int16_t anchorY, const HexapodConfig& config) {
    m_legId = id;
    m_anchorX = anchorX;
    m_anchorY = anchorY;
    m_anchorZ = 0;
    m_anchorAngleDeg = atan2(anchorY, anchorX) * RAD2DEG;
    
    // Store leg dimensions from config
    m_thighLength = config.thighLength;
    m_footLength = config.footLength;
    m_hipToKneeLength = config.hipToKneeLength;
    m_footAngleOffset = config.footAngleOffset;  // Use configurable offset
    
    // Initialize servos based on leg ID
    switch (id) {
        case LegId::LFRONT:
            m_footServo.initialize("LFFoot", config.chLFFoot, config.revLFFoot,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_kneeServo.initialize("LFKnee", config.chLFKnee, config.revLFKnee,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_hipServo.initialize("LFHip", config.chLFHip, config.revLFHip,
                                  config.minServoAngle, config.maxServoAngle,
                                  config.servoMinPulse, config.servoMaxPulse);
            break;
            
        case LegId::LMIDDLE:
            m_footServo.initialize("LMFoot", config.chLMFoot, config.revLMFoot,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_kneeServo.initialize("LMKnee", config.chLMKnee, config.revLMKnee,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_hipServo.initialize("LMHip", config.chLMHip, config.revLMHip,
                                  config.minServoAngle, config.maxServoAngle,
                                  config.servoMinPulse, config.servoMaxPulse);
            break;
            
        case LegId::LBACK:
            m_footServo.initialize("LBFoot", config.chLBFoot, config.revLBFoot,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_kneeServo.initialize("LBKnee", config.chLBKnee, config.revLBKnee,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_hipServo.initialize("LBHip", config.chLBHip, config.revLBHip,
                                  config.minServoAngle, config.maxServoAngle,
                                  config.servoMinPulse, config.servoMaxPulse);
            break;
            
        case LegId::RFRONT:
            m_footServo.initialize("RFFoot", config.chRFFoot, config.revRFFoot,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_kneeServo.initialize("RFKnee", config.chRFKnee, config.revRFKnee,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_hipServo.initialize("RFHip", config.chRFHip, config.revRFHip,
                                  config.minServoAngle, config.maxServoAngle,
                                  config.servoMinPulse, config.servoMaxPulse);
            break;
            
        case LegId::RMIDDLE:
            m_footServo.initialize("RMFoot", config.chRMFoot, config.revRMFoot,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_kneeServo.initialize("RMKnee", config.chRMKnee, config.revRMKnee,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_hipServo.initialize("RMHip", config.chRMHip, config.revRMHip,
                                  config.minServoAngle, config.maxServoAngle,
                                  config.servoMinPulse, config.servoMaxPulse);
            break;
            
        case LegId::RBACK:
            m_footServo.initialize("RBFoot", config.chRBFoot, config.revRBFoot,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_kneeServo.initialize("RBKnee", config.chRBKnee, config.revRBKnee,
                                   config.minServoAngle, config.maxServoAngle,
                                   config.servoMinPulse, config.servoMaxPulse);
            m_hipServo.initialize("RBHip", config.chRBHip, config.revRBHip,
                                  config.minServoAngle, config.maxServoAngle,
                                  config.servoMinPulse, config.servoMaxPulse);
            break;
    }
    
    m_initialized = true;
}

bool HexapodLeg::setFootPosition(const Position& footPosition, uint8_t speed) {
    if (!m_initialized) return false;
    
    float hipAngleDeg, kneeAngleDeg, footAngleDeg;
    if (!calculateIK(footPosition, hipAngleDeg, kneeAngleDeg, footAngleDeg)) {
        return false;
    }
    
    // Apply angles
    m_hipServo.setAngle(hipAngleDeg);
    m_kneeServo.setAngle(kneeAngleDeg);
    m_footServo.setAngle(footAngleDeg);
    
    m_hipServo.setSpeed(speed);
    m_kneeServo.setSpeed(speed);
    m_footServo.setSpeed(speed);
    
    // Store position
    m_footPosX = footPosition.x;
    m_footPosY = footPosition.y;
    m_footPosZ = footPosition.z;
    
    return true;
}

bool HexapodLeg::setFootPositionSmooth(const Position& footPosition, uint8_t speed, uint16_t durationMs) {
    if (!m_initialized) return false;
    
    float hipAngleDeg, kneeAngleDeg, footAngleDeg;
    if (!calculateIK(footPosition, hipAngleDeg, kneeAngleDeg, footAngleDeg)) {
        return false;
    }
    
    // Start smooth moves on all servos
    m_hipServo.startMove(hipAngleDeg, speed, durationMs);
    m_kneeServo.startMove(kneeAngleDeg, speed, durationMs);
    m_footServo.startMove(footAngleDeg, speed, durationMs);
    
    // Store position
    m_footPosX = footPosition.x;
    m_footPosY = footPosition.y;
    m_footPosZ = footPosition.z;
    
    return true;
}

bool HexapodLeg::calculateIK(const Position& footPosition, float& hipAngleDeg, float& kneeAngleDeg, float& footAngleDeg) {
    // Calculate foot position relative to leg origin (hip joint)
    float relX = footPosition.x - m_anchorX;
    float relY = footPosition.y - m_anchorY;
    float relZ = footPosition.z;  // Z is absolute, not relative to anchor
    
    // Rotate relative position by negative anchor angle to get leg-local coordinates
    float anchorAngleRad = m_anchorAngleDeg / RAD2DEG;
    float c = cos(-anchorAngleRad);
    float s = sin(-anchorAngleRad);
    
    float localX = relX * c - relY * s;
    float localY = relX * s + relY * c;
    
    // Calculate horizontal distance from hip to foot
    float l_xy = sqrt(sq(localX) + sq(localY));
    
    // Distance from coxa (hip) joint to foot projection on XY plane
    float l_forward = l_xy - m_hipToKneeLength;  // Subtract COXA length
    
    // Total distance from femur joint to foot
    float D = sqrt(sq(l_forward) + sq(relZ));
    
    // Clamp D to reachable range
    float maxReach = m_thighLength + m_footLength;
    float minReach = fabs(m_thighLength - m_footLength);
    
    if (D > maxReach) {
        D = maxReach - 0.001f;
    }
    if (D < minReach) {
        D = minReach + 0.001f;
    }
    
    // === FOOT (TIBIA) ANGLE ===
    // Original calculates: res.z = RAD_TO_DEG(acos(...) - PI)
    // This gives the angle between femur and tibia, minus 180°
    // At neutral position, this is -90° (tibia perpendicular to femur)
    // Original then writes: servo.write(fabs(angles.z)) = servo.write(90)
    //
    // The key insight: the original's fabs(-90) = 90 goes to servo center,
    // but that's because their servo is mounted so center = bent position.
    //
    // For our library with -90 to +90 mapping:
    // We want -90 to mean fully bent (foot pointing back)
    // We want 0 to mean straight (foot in line with femur)
    // We want +90 to mean bent forward (not physically possible usually)
    //
    // The raw tibiaAngle from the formula is what we want directly!
    
    float cosTibiaAngle = (sq(m_thighLength) + sq(m_footLength) - sq(D)) / 
                          (2.0f * m_thighLength * m_footLength);
    cosTibiaAngle = constrain(cosTibiaAngle, -1.0f, 1.0f);
    // This gives us the angle we need: -90 at neutral, closer to 0 when extended
    footAngleDeg = acos(cosTibiaAngle) * RAD2DEG - 180.0f;
    
    // === KNEE (FEMUR) ANGLE ===
    // Original: res.y = RAD_TO_DEG(beta1 + beta2)
    // Then writes: servo.write(90 + angles.y)
    // At neutral (45°): servo = 90 + 45 = 135
    // Our angle = 135 - 90 = 45
    // So our angle = original angle (no change needed)
    
    float beta1 = atan2(relZ, l_forward);
    float cosBeta2 = (sq(m_thighLength) + sq(D) - sq(m_footLength)) / 
                     (2.0f * m_thighLength * D);
    cosBeta2 = constrain(cosBeta2, -1.0f, 1.0f);
    float beta2 = acos(cosBeta2);
    kneeAngleDeg = (beta1 + beta2) * RAD2DEG;
    
    // === HIP (COXA) ANGLE ===
    // Original: res.x = RAD_TO_DEG(atan2(localY, localX))
    // Then writes: servo.write(90 + angles.x)
    // At neutral (0°): servo = 90
    // Our angle = 90 - 90 = 0
    // So our angle = original angle (no change needed)
    
    hipAngleDeg = atan2(localY, localX) * RAD2DEG;
    
    // Validate angles
    if (isnan(hipAngleDeg) || isnan(kneeAngleDeg) || isnan(footAngleDeg)) {
        return false;
    }
    
    return true;
}

void HexapodLeg::zeroAngles(uint8_t speed) {
    m_footServo.setAngle(0);
    m_kneeServo.setAngle(0);
    m_hipServo.setAngle(0);
    m_footServo.setSpeed(speed);
    m_kneeServo.setSpeed(speed);
    m_hipServo.setSpeed(speed);
}

void HexapodLeg::identifyLeg(uint8_t speed) {
    m_footServo.setAngle(10);
    m_kneeServo.setAngle(10);
    m_hipServo.setAngle(10);
    m_footServo.setSpeed(speed);
    m_kneeServo.setSpeed(speed);
    m_hipServo.setSpeed(speed);
}

} // namespace Hexapod
