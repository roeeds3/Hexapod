/**
 * @file HexapodPoses.h
 * @brief Predefined poses and walking gaits
 * 
 * All poses are fully derived from configuration values,
 * making them work correctly with custom hexapod dimensions.
 */

#ifndef HEXAPOD_POSES_H
#define HEXAPOD_POSES_H

#include <Arduino.h>
#include <math.h>
#include "HexapodTypes.h"
#include "HexapodConfig.h"

namespace Hexapod {

/**
 * @brief Generates poses based on hexapod configuration
 * 
 * All pose methods take the config to use correct dimensions
 * and a height parameter (0-9) for standing height.
 */
class Poses {
public:

    
    static const uint8_t WALK_PHASE_COUNT = 8;
    static const uint8_t ROTATE_PHASE_COUNT = 8;
    static const uint8_t MOVE_PHASE_COUNT = 8;
    
    static int16_t calcHeight(uint8_t height, const HexapodConfig& cfg) {
        return map(height, 0, 9, cfg.minHomeHeight, cfg.maxHomeHeight);
    }
    
    static Position calcLegHome(int16_t anchorX, int16_t anchorY, int16_t z, const HexapodConfig& cfg) {
        float anchorAngle = atan2((float)anchorY, (float)anchorX);
        float horizontalReach = cfg.hipToKneeLength + 
                                (cfg.thighLength + cfg.footLength) * 0.707f;
        float footX = anchorX + horizontalReach * cos(anchorAngle);
        float footY = anchorY + horizontalReach * sin(anchorAngle);
        return Position((int16_t)footX, (int16_t)footY, z);
    }
    
    static KeyFrame home(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        return KeyFrame(
            calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg),
            calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg),
            calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg),
            calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg),
            calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg),
            calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg)
        );
    }
    
    static KeyFrame sleep(const HexapodConfig& cfg) {
        return KeyFrame(
            calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, 0, cfg),
            calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, 0, cfg),
            calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, 0, cfg),
            calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, 0, cfg),
            calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, 0, cfg),
            calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, 0, cfg)
        );
    }
    
    static Position offsetPosition(const Position& pos, int16_t dx, int16_t dy, int16_t dz) {
        return Position(pos.x + dx, pos.y + dy, pos.z + dz);
    }
    
    // ========================================================================
    //  Tripod A: LFront, LBack, RMiddle
    //  Tripod B: RFront, RBack, LMiddle
    // ========================================================================
    // Tripod Walking Gait — 8-Phase
    // ========================================================================

    // Phase 0: Lift A at home. B on ground at home.
    static KeyFrame walkPhase0(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t lift = cfg.footUpOffset;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            offsetPosition(lf, 0, 0, lift),    // A — UP
            lm,                                 // B — ground
            offsetPosition(lb, 0, 0, lift),    // A — UP
            rf,                                 // B — ground
            offsetPosition(rm, 0, 0, lift),    // A — UP
            rb                                  // B — ground
        );
    }
    
    // Phase 1: A swings to +dx in air. B stays at home on ground.
    static KeyFrame walkPhase1(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = cfg.footWalkX;
        int16_t lift = cfg.footUpOffset;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            offsetPosition(lf, dx, 0, lift),   // A — UP + forward
            lm,                                 // B — ground at home
            offsetPosition(lb, dx, 0, lift),   // A — UP + forward
            rf,                                 // B — ground at home
            offsetPosition(rm, dx, 0, lift),   // A — UP + forward
            rb                                  // B — ground at home
        );
    }
    
    // Phase 2: A plants at +dx. B still at home. ALL 6 ON GROUND.
    static KeyFrame walkPhase2(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = cfg.footWalkX;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            offsetPosition(lf, dx, 0, 0),      // A — ground at +dx
            lm,                                 // B — ground at home
            offsetPosition(lb, dx, 0, 0),      // A — ground at +dx
            rf,                                 // B — ground at home
            offsetPosition(rm, dx, 0, 0),      // A — ground at +dx
            rb                                  // B — ground at home
        );
    }
    
    // Phase 3: BODY SHIFT. All 6 on ground, everything shifts -dx.
    // A goes from +dx to home. B goes from home to -dx.
    static KeyFrame walkPhase3(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = cfg.footWalkX;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            lf,                                 // A — ground at home (was +dx, shifted -dx)
            offsetPosition(lm, -dx, 0, 0),     // B — ground at -dx (was home, shifted -dx)
            lb,                                 // A — ground at home
            offsetPosition(rf, -dx, 0, 0),     // B — ground at -dx
            rm,                                 // A — ground at home
            offsetPosition(rb, -dx, 0, 0)      // B — ground at -dx
        );
    }
    
    // Phase 4: B lifts from -dx. A on ground at home.
    static KeyFrame walkPhase4(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = cfg.footWalkX;
        int16_t lift = cfg.footUpOffset;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            lf,                                     // A — ground at home (supporting)
            offsetPosition(lm, -dx, 0, lift),      // B — UP at -dx
            lb,                                     // A — ground at home (supporting)
            offsetPosition(rf, -dx, 0, lift),      // B — UP at -dx
            rm,                                     // A — ground at home (supporting)
            offsetPosition(rb, -dx, 0, lift)       // B — UP at -dx
        );
    }
    
    // Phase 5: B swings to +dx in air. A stays at home on ground.
    static KeyFrame walkPhase5(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = cfg.footWalkX;
        int16_t lift = cfg.footUpOffset;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            lf,                                 // A — ground at home (supporting)
            offsetPosition(lm, dx, 0, lift),   // B — UP + forward
            lb,                                 // A — ground at home (supporting)
            offsetPosition(rf, dx, 0, lift),   // B — UP + forward
            rm,                                 // A — ground at home (supporting)
            offsetPosition(rb, dx, 0, lift)    // B — UP + forward
        );
    }
    
    // Phase 6: B plants at +dx. A still at home. ALL 6 ON GROUND.
    static KeyFrame walkPhase6(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = cfg.footWalkX;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            lf,                                 // A — ground at home
            offsetPosition(lm, dx, 0, 0),      // B — ground at +dx
            lb,                                 // A — ground at home
            offsetPosition(rf, dx, 0, 0),      // B — ground at +dx
            rm,                                 // A — ground at home
            offsetPosition(rb, dx, 0, 0)       // B — ground at +dx
        );
    }
    
    // Phase 7: BODY SHIFT. All 6 on ground, everything shifts -dx.
    // A goes from home to -dx. B goes from +dx to home.
    static KeyFrame walkPhase7(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = cfg.footWalkX;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            offsetPosition(lf, -dx, 0, 0),     // A — ground at -dx
            lm,                                 // B — ground at home
            offsetPosition(lb, -dx, 0, 0),     // A — ground at -dx
            rf,                                 // B — ground at home
            offsetPosition(rm, -dx, 0, 0),     // A — ground at -dx
            rb                                  // B — ground at home
        );
    }
    
    // ========================================================================
    // Rotation Gait — 8-Phase
    // ========================================================================
    
    static Position rotatePosition(const Position& pos, float angleDeg) {
        float angleRad = angleDeg * M_PI / 180.0f;
        float cosA = cos(angleRad);
        float sinA = sin(angleRad);
        float newX = pos.x * cosA - pos.y * sinA;
        float newY = pos.x * sinA + pos.y * cosA;
        return Position(
            (int16_t)(newX >= 0 ? (newX + 0.5f) : (newX - 0.5f)),
            (int16_t)(newY >= 0 ? (newY + 0.5f) : (newY - 0.5f)),
            pos.z
        );
    }
    
    static KeyFrame rotatePhase0(uint8_t height, const HexapodConfig& cfg,bool clockwise) {
        return walkPhase0(height, cfg);
    }
    
    static KeyFrame rotatePhase1(uint8_t height, const HexapodConfig& cfg, bool clockwise) {
        int16_t z = calcHeight(height, cfg);
        int16_t lift = cfg.footUpOffset;
        float angle = clockwise ? cfg.rotationAngleDeg : -cfg.rotationAngleDeg;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        Position lfRot = rotatePosition(lf, angle);  lfRot.z = z + lift;
        Position lbRot = rotatePosition(lb, angle);  lbRot.z = z + lift;
        Position rmRot = rotatePosition(rm, angle);  rmRot.z = z + lift;
        
        return KeyFrame(lfRot, lm, lbRot, rf, rmRot, rb);
    }
    
    static KeyFrame rotatePhase2(uint8_t height, const HexapodConfig& cfg, bool clockwise) {
        int16_t z = calcHeight(height, cfg);
        float angle = clockwise ? cfg.rotationAngleDeg : -cfg.rotationAngleDeg;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            rotatePosition(lf, angle), lm, rotatePosition(lb, angle),
            rf, rotatePosition(rm, angle), rb
        );
    }
    
    static KeyFrame rotatePhase3(uint8_t height, const HexapodConfig& cfg, bool clockwise) {
        int16_t z = calcHeight(height, cfg);
        float angle = clockwise ? cfg.rotationAngleDeg : -cfg.rotationAngleDeg;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            lf, rotatePosition(lm, -angle), lb,
            rotatePosition(rf, -angle), rm, rotatePosition(rb, -angle)
        );
    }
    
    static KeyFrame rotatePhase4(uint8_t height, const HexapodConfig& cfg, bool clockwise) {
        int16_t z = calcHeight(height, cfg);
        int16_t lift = cfg.footUpOffset;
        float angle = clockwise ? cfg.rotationAngleDeg : -cfg.rotationAngleDeg;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        Position lmUp = rotatePosition(lm, -angle);  lmUp.z = z + lift;
        Position rfUp = rotatePosition(rf, -angle);   rfUp.z = z + lift;
        Position rbUp = rotatePosition(rb, -angle);   rbUp.z = z + lift;
        
        return KeyFrame(lf, lmUp, lb, rfUp, rm, rbUp);
    }
    
    static KeyFrame rotatePhase5(uint8_t height, const HexapodConfig& cfg, bool clockwise) {
        int16_t z = calcHeight(height, cfg);
        int16_t lift = cfg.footUpOffset;
        float angle = clockwise ? cfg.rotationAngleDeg : -cfg.rotationAngleDeg;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        Position lmRot = rotatePosition(lm, angle);  lmRot.z = z + lift;
        Position rfRot = rotatePosition(rf, angle);   rfRot.z = z + lift;
        Position rbRot = rotatePosition(rb, angle);   rbRot.z = z + lift;
        
        return KeyFrame(lf, lmRot, lb, rfRot, rm, rbRot);
    }
    
    static KeyFrame rotatePhase6(uint8_t height, const HexapodConfig& cfg, bool clockwise) {
        int16_t z = calcHeight(height, cfg);
        float angle = clockwise ? cfg.rotationAngleDeg : -cfg.rotationAngleDeg;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            lf, rotatePosition(lm, angle), lb,
            rotatePosition(rf, angle), rm, rotatePosition(rb, angle)
        );
    }
    
    static KeyFrame rotatePhase7(uint8_t height, const HexapodConfig& cfg, bool clockwise) {
        int16_t z = calcHeight(height, cfg);
        float angle = clockwise ? cfg.rotationAngleDeg : -cfg.rotationAngleDeg;
        
        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);
        
        return KeyFrame(
            rotatePosition(lf, -angle), lm, rotatePosition(lb, -angle),
            rf, rotatePosition(rm, -angle), rb
        );
    }

    // ========================================================================
    // OmniDirectional Walking Poses
    // ========================================================================

    static KeyFrame movePhase0(uint8_t height, const HexapodConfig& cfg, Vector v) {
        int16_t z = calcHeight(height, cfg);
        int16_t lift = cfg.footUpOffset;

        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);

        return KeyFrame(
            offsetPosition(lf, 0, 0, lift),    // A — UP
            lm,                                 // B — ground
            offsetPosition(lb, 0, 0, lift),    // A — UP
            rf,                                 // B — ground
            offsetPosition(rm, 0, 0, lift),    // A — UP
            rb                                  // B — ground
        );
    }

    // Phase 1: A swings to +dx,+dy in air. B stays at home on ground.
    static KeyFrame movePhase1(uint8_t height, const HexapodConfig& cfg, Vector v) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = static_cast<int>(cfg.footWalkX * v.x);
        int16_t dy = static_cast<int>(cfg.footWalkX * v.y);
        int16_t lift = cfg.footUpOffset;

        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);

        return KeyFrame(
            offsetPosition(lf, dx, dy, lift),   // A — UP + forward
            lm,                                 // B — ground at home
            offsetPosition(lb, dx, dy, lift),   // A — UP + forward
            rf,                                 // B — ground at home
            offsetPosition(rm, dx, dy, lift),   // A — UP + forward
            rb                                  // B — ground at home
        );
    }

    // Phase 2: A plants at +dx,+dy. B still at home. ALL 6 ON GROUND.
    static KeyFrame movePhase2(uint8_t height, const HexapodConfig& cfg, Vector v) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = static_cast<int>(cfg.footWalkX * v.x);
        int16_t dy = static_cast<int>(cfg.footWalkX * v.y);

        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);

        return KeyFrame(
            offsetPosition(lf, dx, dy, 0),      // A — ground at +dx,+dy
            lm,                                 // B — ground at home
            offsetPosition(lb, dx, dy, 0),      // A — ground at +dx,+dy
            rf,                                 // B — ground at home
            offsetPosition(rm, dx, dy, 0),      // A — ground at +dx,+dy
            rb                                  // B — ground at home
        );
    }

    // Phase 3: BODY SHIFT. All 6 on ground, everything shifts -dx,+dy.
    // A goes from +dx,+dy to home. B goes from home to -dx,-dy.
    static KeyFrame movePhase3(uint8_t height, const HexapodConfig& cfg, Vector v) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = static_cast<int>(cfg.footWalkX * v.x);
        int16_t dy = static_cast<int>(cfg.footWalkX * v.y);

        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);

        return KeyFrame(
            lf,                                 // A — ground at home (was +dx,+dy, shifted -dx,-dy)
            offsetPosition(lm, -dx, -dy, 0),     // B — ground at -dx,-dy (was home, shifted -dx,-dy)
            lb,                                 // A — ground at home
            offsetPosition(rf, -dx, -dy, 0),     // B — ground at -dx,-dy
            rm,                                 // A — ground at home
            offsetPosition(rb, -dx, -dy, 0)      // B — ground at -dx,-dy
        );
    }

    // Phase 4: B lifts from -dx,-dy. A on ground at home.
    static KeyFrame movePhase4(uint8_t height, const HexapodConfig& cfg, Vector v) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = static_cast<int>(cfg.footWalkX * v.x);
        int16_t dy = static_cast<int>(cfg.footWalkX * v.y);
        int16_t lift = cfg.footUpOffset;

        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);

        return KeyFrame(
            lf,                                     // A — ground at home (supporting)
            offsetPosition(lm, -dx, -dy, lift),      // B — UP at -dx,-dy
            lb,                                     // A — ground at home (supporting)
            offsetPosition(rf, -dx, -dy, lift),      // B — UP at -dx,-dy
            rm,                                     // A — ground at home (supporting)
            offsetPosition(rb, -dx, -dy, lift)       // B — UP at -dx,-dy
        );
    }

    // Phase 5: B swings to +dx,+dy in air. A stays at home on ground.
    static KeyFrame movePhase5(uint8_t height, const HexapodConfig& cfg, Vector v) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = static_cast<int>(cfg.footWalkX * v.x);
        int16_t dy = static_cast<int>(cfg.footWalkX * v.y);
        int16_t lift = cfg.footUpOffset;

        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);

        return KeyFrame(
            lf,                                 // A — ground at home (supporting)
            offsetPosition(lm, dx, dy, lift),   // B — UP + forward
            lb,                                 // A — ground at home (supporting)
            offsetPosition(rf, dx, dy, lift),   // B — UP + forward
            rm,                                 // A — ground at home (supporting)
            offsetPosition(rb, dx, dy, lift)    // B — UP + forward
        );
    }

    // Phase 6: B plants at +dx,+dy. A still at home. ALL 6 ON GROUND.
    static KeyFrame movePhase6(uint8_t height, const HexapodConfig& cfg, Vector v) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = static_cast<int>(cfg.footWalkX * v.x);
        int16_t dy = static_cast<int>(cfg.footWalkX * v.y);

        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);

        return KeyFrame(
            lf,                                 // A — ground at home
            offsetPosition(lm, dx, dy, 0),      // B — ground at +dx,+dy
            lb,                                 // A — ground at home
            offsetPosition(rf, dx, dy, 0),      // B — ground at +dx,+dy
            rm,                                 // A — ground at home
            offsetPosition(rb, dx, dy, 0)       // B — ground at +dx,+dy
        );
    }

    // Phase 7: BODY SHIFT. All 6 on ground, everything shifts -dx,-dy.
    // A goes from home to -dx,-dy. B goes from +dx,+dy to home.
    static KeyFrame movePhase7(uint8_t height, const HexapodConfig& cfg, Vector v) {
        int16_t z = calcHeight(height, cfg);
        int16_t dx = static_cast<int>(cfg.footWalkX * v.x);
        int16_t dy = static_cast<int>(cfg.footWalkX * v.y);

        Position lf = calcLegHome(cfg.anchorLFrontX, cfg.anchorLFrontY, z, cfg);
        Position lm = calcLegHome(cfg.anchorLMiddleX, cfg.anchorLMiddleY, z, cfg);
        Position lb = calcLegHome(cfg.anchorLBackX, cfg.anchorLBackY, z, cfg);
        Position rf = calcLegHome(cfg.anchorLFrontX, -cfg.anchorLFrontY, z, cfg);
        Position rm = calcLegHome(cfg.anchorLMiddleX, -cfg.anchorLMiddleY, z, cfg);
        Position rb = calcLegHome(cfg.anchorLBackX, -cfg.anchorLBackY, z, cfg);

        return KeyFrame(
            offsetPosition(lf, -dx, -dy, 0),     // A — ground at -dx
            lm,                                 // B — ground at home
            offsetPosition(lb, -dx, -dy, 0),     // A — ground at -dx
            rf,                                 // B — ground at home
            offsetPosition(rm, -dx, -dy, 0),     // A — ground at -dx
            rb                                  // B — ground at home
        );
    }
    
    // ========================================================================
    // Animation Poses
    // ========================================================================
    
    static KeyFrame anim1Phase1(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t reach = cfg.animReachDistance;
        int16_t lift = cfg.animLiftHeight;
        return KeyFrame(
            Position(cfg.footFBDistance, cfg.footWidthFB, z),
            Position(reach, cfg.footWidthM, z + lift),
            Position(-cfg.footFBDistance, cfg.footWidthFB, z),
            Position(cfg.footFBDistance, -cfg.footWidthFB, z),
            Position(reach, -cfg.footWidthM, z + lift),
            Position(-cfg.footFBDistance, -cfg.footWidthFB, z)
        );
    }
    
    static KeyFrame anim1Phase2(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t reach = cfg.animReachDistance;
        return KeyFrame(
            Position(cfg.footFBDistance, cfg.footWidthFB, z),
            Position(reach, cfg.footWidthM, z),
            Position(-cfg.footFBDistance, cfg.footWidthFB, z),
            Position(cfg.footFBDistance, -cfg.footWidthFB, z),
            Position(reach, -cfg.footWidthM, z),
            Position(-cfg.footFBDistance, -cfg.footWidthFB, z)
        );
    }
    
    static KeyFrame anim1Phase3(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t reach = cfg.animReachDistance;
        int16_t lift = cfg.animLiftHeight;
        int16_t widthOff = cfg.animWidthOffset + cfg.animExtraWidth;
        int16_t extraLift = cfg.animExtraLift;
        return KeyFrame(
            Position(cfg.footFBDistance + lift, cfg.footWidthFB - widthOff, z + lift + extraLift),
            Position(reach, cfg.footWidthM, z - cfg.footUpOffset),
            Position(-cfg.footFBDistance, cfg.footWidthFB, z),
            Position(cfg.footFBDistance + lift, -cfg.footWidthFB + widthOff, z + lift + extraLift),
            Position(reach, -cfg.footWidthM, z - cfg.footUpOffset),
            Position(-cfg.footFBDistance, -cfg.footWidthFB, z)
        );
    }
    
    static KeyFrame anim1Phase4(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t reach = cfg.animReachDistance;
        int16_t lift = cfg.animLiftHeight;
        int16_t widthOff = cfg.animWidthOffset + cfg.animExtraWidth;
        return KeyFrame(
            Position(cfg.footFBDistance + lift, cfg.footWidthFB - widthOff, z - lift),
            Position(reach, cfg.footWidthM, z - cfg.footUpOffset),
            Position(-cfg.footFBDistance, cfg.footWidthFB, z),
            Position(cfg.footFBDistance + lift, -cfg.footWidthFB + widthOff, z - lift),
            Position(reach, -cfg.footWidthM, z - cfg.footUpOffset),
            Position(-cfg.footFBDistance, -cfg.footWidthFB, z)
        );
    }
    
    static KeyFrame anim2Phase1(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t widthOff = cfg.animWidthOffset;
        int16_t lift = cfg.animLiftHeight + cfg.animExtraLift;
        return KeyFrame(
            Position(cfg.footFBDistance, cfg.footWidthFB, z),
            Position(0, cfg.footWidthM + widthOff, z + lift),
            Position(-cfg.footFBDistance, cfg.footWidthFB, z),
            Position(cfg.footFBDistance, -cfg.footWidthFB, z),
            Position(0, -cfg.footWidthM, z),
            Position(-cfg.footFBDistance, -cfg.footWidthFB, z)
        );
    }
    
    static KeyFrame anim2Phase2(uint8_t height, const HexapodConfig& cfg) {
        int16_t z = calcHeight(height, cfg);
        int16_t widthOff = cfg.animWidthOffset;
        return KeyFrame(
            Position(cfg.footFBDistance, cfg.footWidthFB, z),
            Position(0, cfg.footWidthM + widthOff, z),
            Position(-cfg.footFBDistance, cfg.footWidthFB, z),
            Position(cfg.footFBDistance, -cfg.footWidthFB, z),
            Position(0, -cfg.footWidthM, z),
            Position(-cfg.footFBDistance, -cfg.footWidthFB, z)
        );
    }
};

} // namespace Hexapod

#endif // HEXAPOD_POSES_H
