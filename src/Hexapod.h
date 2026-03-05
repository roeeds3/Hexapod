/**
 * @file Hexapod.h
 * @brief Main hexapod controller with built-in servo drivers
 */

#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <Arduino.h>
#include "HexapodTypes.h"
#include "HexapodConfig.h"
#include "HexapodLeg.h"
#include "HexapodPoses.h"

// Include driver libraries based on defines
#ifdef HEXAPOD_ENABLE_PCA9685
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#endif

#ifdef HEXAPOD_ENABLE_MAESTRO
#include <PololuMaestro.h>
#endif

#ifdef HEXAPOD_ENABLE_DIRECT_PWM
#include <Servo.h>
#endif

namespace Hexapod {

/**
 * @brief Main hexapod robot controller
 * 
 * Supports three driver types:
 *   0 (DRIVER_PCA9685)    - Adafruit PCA9685 PWM driver
 *   1 (DRIVER_MAESTRO)    - Pololu Maestro servo controller
 *   2 (DRIVER_DIRECT_PWM) - Arduino Servo library (direct PWM)
 * 
 * Example with default configuration:
 * @code
 * Hexapod::Hexapod robot(0);  // PCA9685 with defaults
 * 
 * void setup() {
 *     robot.begin();
 *     robot.goHome(50);
 *     robot.update();
 * }
 * @endcode
 * 
 * Example with custom configuration:
 * @code
 * Hexapod::HexapodConfig config;
 * config.thighLength = 80;
 * config.footLength = 120;
 * config.pca9685Address = 0x41;
 * 
 * Hexapod::Hexapod robot(0, config);
 * @endcode
 * 
 * Example with Pololu Maestro:
 * @code
 * Hexapod::Hexapod robot(1, Hexapod::HexapodConfig(), Serial1);
 * @endcode
 */
class Hexapod {
public:
    /**
     * @brief Construct hexapod with default configuration
     * @param driverType 0=PCA9685, 1=Maestro, 2=DirectPWM
     */
    explicit Hexapod(uint8_t driverType);
    
    /**
     * @brief Construct hexapod with custom configuration
     * @param driverType 0=PCA9685, 1=Maestro, 2=DirectPWM
     * @param config Custom configuration struct
     */
    Hexapod(uint8_t driverType, const HexapodConfig& config);
    
    /**
     * @brief Construct hexapod with Maestro driver using specified serial
     * @param driverType Should be 1 (DRIVER_MAESTRO)
     * @param config Custom configuration struct
     * @param serial Serial port for Maestro communication
     */
    Hexapod(uint8_t driverType, const HexapodConfig& config, Stream& serial);
    
    /**
     * @brief Destructor - cleans up dynamically allocated driver objects
     */
    ~Hexapod();
    
    // Prevent copying (due to dynamic allocation)
    Hexapod(const Hexapod&) = delete;
    Hexapod& operator=(const Hexapod&) = delete;
    
    /**
     * @brief Initialize the hexapod and servo driver
     */
    void begin();
    
    /**
     * @brief Send all servo positions to hardware
     * Call this after changing positions
     */
    void update();
    
    /**
     * @brief Set all legs to a pose
     * @param frame Target positions for all legs
     * @param speed Movement speed (0-100)
     * @return true if all positions are reachable
     */
    bool setPose(const KeyFrame& frame, uint8_t speed);
    
    /**
     * @brief Set a single leg position
     * @param legId Which leg
     * @param position Target position
     * @param speed Movement speed
     * @return true if reachable
     */
    bool setLegPosition(LegId legId, const Position& position, uint8_t speed);
    
    /**
     * @brief Move to standing position
     * @param speed Movement speed (0-100)
     */
    void goHome(uint8_t speed);
    
    /**
     * @brief Lower body to ground
     * @param speed Movement speed (0-100)
     */
    void goSleep(uint8_t speed);
    
    /**
     * @brief Center all servos
     * @param speed Movement speed (0-100)
     */
    void zeroAllServos(uint8_t speed);
    
    /**
     * @brief Small movement for leg identification
     * @param speed Movement speed (0-100)
     */
    void identifyAllLegs(uint8_t speed);
    
    /**
     * @brief Execute one forward walk step
     * Call repeatedly for continuous walking
     * @param speed Movement speed (0-100)
     */
    void walkForward(uint8_t speed);
    
    /**
     * @brief Execute one backward walk step
     * @param speed Movement speed (0-100)
     */
    void walkBackward(uint8_t speed);
    
    /**
     * @brief Execute one clockwise rotation step
     * @param speed Movement speed (0-100)
     */
    void rotateClockwise(uint8_t speed);
    
    /**
     * @brief Execute one counter-clockwise rotation step
     * @param speed Movement speed (0-100)
     */
    void rotateCounterClockwise(uint8_t speed);
    
    // ========================================================================
    // Smooth Motion Control
    // ========================================================================
    
    /**
     * @brief Update interpolation and send current positions to servos
     * 
     * Call this frequently (every 10-20ms) in your loop for smooth motion.
     * This updates the interpolation state and sends the current (interpolated)
     * positions to the servo driver.
     * 
     * Example:
     * @code
     * robot.goHome(50);  // Start moving to home
     * while (robot.isMoving()) {
     *     robot.updateSmooth();
     *     delay(20);
     * }
     * @endcode
     * 
     * @return true if any servo position changed
     */
    bool updateSmooth();
    
    /**
     * @brief Check if any servo is still moving to its target
     * @return true if interpolation is in progress on any servo
     */
    bool isMoving() const;
    
    /**
     * @brief Immediately snap all servos to their target positions
     * 
     * Cancels any in-progress interpolation and sets current = target
     * for all servos. Call update() after this to send positions.
     */
    void snapToTargets();
    
    /**
     * @brief Set pose with smooth motion
     * 
     * Like setPose(), but initiates smooth interpolation.
     * Call updateSmooth() repeatedly until isMoving() returns false.
     * 
     * @param frame Target positions for all legs
     * @param speed Movement speed (0-100)
     * @param durationMs Optional: override calculated duration (0 = auto)
     * @return true if all positions are reachable
     */
    bool setPoseSmooth(const KeyFrame& frame, uint8_t speed, uint16_t durationMs = 0);
    
    /**
     * @brief Move to home position with smooth motion
     * @param speed Movement speed (0-100)
     */
    void goHomeSmooth(uint8_t speed);
    
    /**
     * @brief Lower body to ground with smooth motion
     * @param speed Movement speed (0-100)
     */
    void goSleepSmooth(uint8_t speed);
    
    // ========================================================================
    // Height and Configuration
    // ========================================================================
    
    // Height control (0-9)
    void setHeight(uint8_t height) { m_height = constrain(height, 0, 9); }
    uint8_t getHeight() const { return m_height; }
    
    // Check if robot was initialized successfully
    bool isInitialized() const { return m_initialized; }
    
    // Access to legs for advanced use
    HexapodLeg& getLeg(LegId id);
    const HexapodLeg& getLeg(LegId id) const;
    const HexapodConfig& getConfig() const { return m_config; }

private:
    void initializeLegs();
    void sendServoCommand(uint8_t channel, float angle, bool reverse, uint8_t speed);
    void cleanup();  // Helper for destructor
    
    DriverType m_driverType;
    HexapodConfig m_config;
    
    HexapodLeg m_legs[6];
    
    uint8_t m_height;
    uint8_t m_walkPhase;
    uint8_t m_rotatePhase;
    
    bool m_initialized;
    
    // Driver-specific members
#ifdef HEXAPOD_ENABLE_PCA9685
    Adafruit_PWMServoDriver* m_pca9685;
    Adafruit_PWMServoDriver* m_pca9685_2;  // Second board for channels 16+
#endif

#ifdef HEXAPOD_ENABLE_MAESTRO
    MiniMaestro* m_maestro;
    Stream* m_maestroSerial;
#endif

#ifdef HEXAPOD_ENABLE_DIRECT_PWM
    Servo m_servos[18];
#endif
};

} // namespace Hexapod

#endif // HEXAPOD_H
