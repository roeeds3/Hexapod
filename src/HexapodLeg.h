/**
 * @file HexapodLeg.h
 * @brief Individual leg control with inverse kinematics
 */

#ifndef HEXAPOD_LEG_H
#define HEXAPOD_LEG_H

#include <Arduino.h>
#include "HexapodTypes.h"
#include "HexapodConfig.h"
#include "HexapodServo.h"

namespace Hexapod {

/**
 * @brief Represents a single 3-DOF hexapod leg with inverse kinematics
 */
class HexapodLeg {
public:
    HexapodLeg();
    
    /**
     * @brief Initialize the leg with configuration
     * @param id Leg identifier
     * @param anchorX Hip joint X position from body center
     * @param anchorY Hip joint Y position from body center
     * @param config Full hexapod configuration
     */
    void initialize(LegId id, int16_t anchorX, int16_t anchorY, const HexapodConfig& config);
    
    /**
     * @brief Set foot position using inverse kinematics
     * @param footPosition Target position relative to body center (mm)
     * @param speed Movement speed (0-100)
     * @return true if position is reachable
     */
    bool setFootPosition(const Position& footPosition, uint8_t speed);
    
    /**
     * @brief Set foot position with smooth motion
     * 
     * Like setFootPosition(), but initiates smooth interpolation on all servos.
     * Call updateInterpolation() on each servo, or use Hexapod::updateSmooth().
     * 
     * @param footPosition Target position relative to body center (mm)
     * @param speed Movement speed (0-100)
     * @param durationMs Optional: override calculated duration (0 = auto)
     * @return true if position is reachable
     */
    bool setFootPositionSmooth(const Position& footPosition, uint8_t speed, uint16_t durationMs = 0);
    
    /**
     * @brief Set all servos to zero angle
     */
    void zeroAngles(uint8_t speed);
    
    /**
     * @brief Small offset for leg identification
     */
    void identifyLeg(uint8_t speed);
    
    // Getters
    LegId getId() const { return m_legId; }
    Position getFootPosition() const { return Position(m_footPosX, m_footPosY, m_footPosZ); }
    
    HexapodServo& getFootServo() { return m_footServo; }
    HexapodServo& getKneeServo() { return m_kneeServo; }
    HexapodServo& getHipServo() { return m_hipServo; }
    
    const HexapodServo& getFootServo() const { return m_footServo; }
    const HexapodServo& getKneeServo() const { return m_kneeServo; }
    const HexapodServo& getHipServo() const { return m_hipServo; }

private:
    LegId m_legId;
    int16_t m_anchorX;
    int16_t m_anchorY;
    int16_t m_anchorZ;
    float m_anchorAngleDeg;
    
    int16_t m_thighLength;
    int16_t m_footLength;
    int16_t m_hipToKneeLength;
    float m_footAngleOffset;  // Configurable calibration offset
    
    HexapodServo m_footServo;
    HexapodServo m_kneeServo;
    HexapodServo m_hipServo;
    
    int16_t m_footPosX;
    int16_t m_footPosY;
    int16_t m_footPosZ;
    
    bool m_initialized;
    
    /**
     * @brief Calculate servo angles from foot position using inverse kinematics
     * @param footPosition Target position
     * @param hipAngle Output hip angle
     * @param kneeAngle Output knee angle
     * @param footAngle Output foot angle
     * @return true if position is reachable and angles are valid
     */
    bool calculateIK(const Position& footPosition, float& hipAngle, float& kneeAngle, float& footAngle);
};

} // namespace Hexapod

#endif // HEXAPOD_LEG_H
