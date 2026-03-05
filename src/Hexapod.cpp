/**
 * @file Hexapod.cpp
 * @brief Implementation of main Hexapod controller with built-in drivers
 */

// IMPORTANT: Include HexapodKinematics.h (not just Hexapod.h) to get the
// HEXAPOD_ENABLE_* defines that control which drivers are compiled in.
#include "HexapodKinematics.h"

namespace Hexapod {

// ============================================================================
// Constructors & Destructor
// ============================================================================

Hexapod::Hexapod(uint8_t driverType)
    : m_driverType(static_cast<DriverType>(driverType))
    , m_config()
    , m_height(5)
    , m_walkPhase(0)
    , m_rotatePhase(0)
    , m_initialized(false)
#ifdef HEXAPOD_ENABLE_PCA9685
    , m_pca9685(nullptr)
    , m_pca9685_2(nullptr)
#endif
#ifdef HEXAPOD_ENABLE_MAESTRO
    , m_maestro(nullptr)
    , m_maestroSerial(nullptr)
#endif
{
    initializeLegs();
}

Hexapod::Hexapod(uint8_t driverType, const HexapodConfig& config)
    : m_driverType(static_cast<DriverType>(driverType))
    , m_config(config)
    , m_height(5)
    , m_walkPhase(0)
    , m_rotatePhase(0)
    , m_initialized(false)
#ifdef HEXAPOD_ENABLE_PCA9685
    , m_pca9685(nullptr)
    , m_pca9685_2(nullptr)
#endif
#ifdef HEXAPOD_ENABLE_MAESTRO
    , m_maestro(nullptr)
    , m_maestroSerial(nullptr)
#endif
{
    initializeLegs();
}

Hexapod::Hexapod(uint8_t driverType, const HexapodConfig& config, Stream& serial)
    : m_driverType(static_cast<DriverType>(driverType))
    , m_config(config)
    , m_height(5)
    , m_walkPhase(0)
    , m_rotatePhase(0)
    , m_initialized(false)
#ifdef HEXAPOD_ENABLE_PCA9685
    , m_pca9685(nullptr)
    , m_pca9685_2(nullptr)
#endif
#ifdef HEXAPOD_ENABLE_MAESTRO
    , m_maestro(nullptr)
    , m_maestroSerial(&serial)
#endif
{
    initializeLegs();
}

Hexapod::~Hexapod() {
    cleanup();
}

void Hexapod::cleanup() {
#ifdef HEXAPOD_ENABLE_PCA9685
    if (m_pca9685 != nullptr) {
        delete m_pca9685;
        m_pca9685 = nullptr;
    }
    if (m_pca9685_2 != nullptr) {
        delete m_pca9685_2;
        m_pca9685_2 = nullptr;
    }
#endif

#ifdef HEXAPOD_ENABLE_MAESTRO
    if (m_maestro != nullptr) {
        delete m_maestro;
        m_maestro = nullptr;
    }
    // Note: m_maestroSerial is not owned by us, don't delete it
#endif

#ifdef HEXAPOD_ENABLE_DIRECT_PWM
    // Detach all servos
    if (m_initialized) {
        for (int i = 0; i < 18; i++) {
            m_servos[i].detach();
        }
    }
#endif

    m_initialized = false;
}

// ============================================================================
// Initialization
// ============================================================================

void Hexapod::initializeLegs() {
    // Initialize each leg with its anchor position from config
    m_legs[0].initialize(LegId::LFRONT, m_config.anchorLFrontX, m_config.anchorLFrontY, m_config);
    m_legs[1].initialize(LegId::LMIDDLE, m_config.anchorLMiddleX, m_config.anchorLMiddleY, m_config);
    m_legs[2].initialize(LegId::LBACK, m_config.anchorLBackX, m_config.anchorLBackY, m_config);
    m_legs[3].initialize(LegId::RFRONT, m_config.anchorLFrontX, -m_config.anchorLFrontY, m_config);
    m_legs[4].initialize(LegId::RMIDDLE, m_config.anchorLMiddleX, -m_config.anchorLMiddleY, m_config);
    m_legs[5].initialize(LegId::RBACK, m_config.anchorLBackX, -m_config.anchorLBackY, m_config);
}

void Hexapod::begin() {
    // Clean up any previous initialization
    if (m_initialized) {
        cleanup();
    }
    
    switch (m_driverType) {
#ifdef HEXAPOD_ENABLE_PCA9685
        case DRIVER_PCA9685:
            // Initialize I2C with custom pins if specified (ESP32 support)
            if (m_config.i2cSdaPin >= 0 && m_config.i2cSclPin >= 0) {
                Wire.begin(m_config.i2cSdaPin, m_config.i2cSclPin);
            } else {
                Wire.begin();
            }
            
            m_pca9685 = new Adafruit_PWMServoDriver(m_config.pca9685Address);
            if (m_pca9685 != nullptr) {
                m_pca9685->begin();
                m_pca9685->setOscillatorFrequency(27000000);
                m_pca9685->setPWMFreq(50);
                
                // Initialize second PCA9685 if enabled and we have channels > 15
                if (m_config.useDualPCA9685) {
                    m_pca9685_2 = new Adafruit_PWMServoDriver(m_config.pca9685Address2);
                    if (m_pca9685_2 != nullptr) {
                        m_pca9685_2->begin();
                        m_pca9685_2->setOscillatorFrequency(27000000);
                        m_pca9685_2->setPWMFreq(50);
                    }
                    // Note: We still set initialized=true even if board 2 fails,
                    // as board 1 can still control channels 0-15
                }
                
                delay(10);
                m_initialized = true;
            }
            // If allocation failed, m_initialized stays false
            break;
#endif

#ifdef HEXAPOD_ENABLE_MAESTRO
        case DRIVER_MAESTRO:
            if (m_maestroSerial != nullptr) {
                m_maestro = new MiniMaestro(*m_maestroSerial);
                if (m_maestro != nullptr) {
                    m_initialized = true;
                }
                // If allocation failed, m_initialized stays false
            }
            // If m_maestroSerial is null, m_initialized stays false
            break;
#endif

#ifdef HEXAPOD_ENABLE_DIRECT_PWM
        case DRIVER_DIRECT_PWM:
            for (int i = 0; i < 18; i++) {
                m_servos[i].attach(m_config.directPwmPins[i]);
            }
            m_initialized = true;
            break;
#endif

        default:
            // Unknown driver type - m_initialized stays false
            break;
    }
}

// ============================================================================
// Servo Control
// ============================================================================

void Hexapod::sendServoCommand(uint8_t channel, float angle, bool reverse, uint8_t speed) {
    if (!m_initialized) return;
    
    if (reverse) angle = -angle;
    
    switch (m_driverType) {
#ifdef HEXAPOD_ENABLE_PCA9685
        case DRIVER_PCA9685: {
            // Determine which board to use based on channel number
            Adafruit_PWMServoDriver* board = m_pca9685;
            uint8_t boardChannel = channel;
            
            if (channel >= 16) {
                // Use second board for channels 16+
                if (m_pca9685_2 == nullptr) return;  // Second board not available
                board = m_pca9685_2;
                boardChannel = channel - 16;  // Remap to 0-15 on second board
            } else {
                if (m_pca9685 == nullptr) return;
            }
            
            // Map angle to pulse width, then to PCA9685 value
            uint16_t pulseUs = map(angle * 100, 
                                   m_config.minServoAngle * 100, 
                                   m_config.maxServoAngle * 100,
                                   m_config.servoMinPulse, 
                                   m_config.servoMaxPulse);
            
            // PCA9685 operates at 50Hz, so 20ms period = 4096 ticks
            uint16_t pulse = map(pulseUs, 0, 20000, 0, 4096);
            board->setPWM(boardChannel, 0, pulse);
            break;
        }
#endif

#ifdef HEXAPOD_ENABLE_MAESTRO
        case DRIVER_MAESTRO: {
            if (m_maestro == nullptr) return;
            
            // Maestro uses quarter-microseconds
            uint16_t pulseUs = map(angle * 100,
                                   m_config.minServoAngle * 100,
                                   m_config.maxServoAngle * 100,
                                   m_config.servoMinPulse,
                                   m_config.servoMaxPulse);
            uint16_t target = pulseUs * 4;
            m_maestro->setTarget(channel, target);
            
            // Set speed if supported (0 = unlimited, higher = slower)
            if (speed < 100) {
                uint16_t maestroSpeed = map(speed, 0, 100, 200, 0);
                m_maestro->setSpeed(channel, maestroSpeed);
            }
            break;
        }
#endif

#ifdef HEXAPOD_ENABLE_DIRECT_PWM
        case DRIVER_DIRECT_PWM: {
            if (channel >= 18) return;
            
            // Arduino Servo library uses 0-180 degrees
            int pos = map(angle * 100,
                          m_config.minServoAngle * 100,
                          m_config.maxServoAngle * 100,
                          0, 180);
            pos = constrain(pos, 0, 180);  // Ensure valid range
            m_servos[channel].write(pos);
            break;
        }
#endif

        default:
            break;
    }
}

void Hexapod::update() {
    if (!m_initialized) return;
    
    // Send commands for all servos on all legs
    for (int i = 0; i < 6; i++) {
        HexapodServo& foot = m_legs[i].getFootServo();
        HexapodServo& knee = m_legs[i].getKneeServo();
        HexapodServo& hip = m_legs[i].getHipServo();
        
        sendServoCommand(foot.getChannel(), foot.getTargetAngle(), foot.isReverse(), foot.getTargetSpeed());
        sendServoCommand(knee.getChannel(), knee.getTargetAngle(), knee.isReverse(), knee.getTargetSpeed());
        sendServoCommand(hip.getChannel(), hip.getTargetAngle(), hip.isReverse(), hip.getTargetSpeed());
        
        // Sync current angles so smooth motion knows where servos actually are
        foot.syncCurrentToTarget();
        knee.syncCurrentToTarget();
        hip.syncCurrentToTarget();
    }
}

// ============================================================================
// Pose Control
// ============================================================================

bool Hexapod::setPose(const KeyFrame& frame, uint8_t speed) {
    bool success = true;
    success &= m_legs[0].setFootPosition(frame.LFront, speed);
    success &= m_legs[1].setFootPosition(frame.LMiddle, speed);
    success &= m_legs[2].setFootPosition(frame.LBack, speed);
    success &= m_legs[3].setFootPosition(frame.RFront, speed);
    success &= m_legs[4].setFootPosition(frame.RMiddle, speed);
    success &= m_legs[5].setFootPosition(frame.RBack, speed);
    return success;
}

bool Hexapod::setLegPosition(LegId legId, const Position& position, uint8_t speed) {
    return getLeg(legId).setFootPosition(position, speed);
}

void Hexapod::goHome(uint8_t speed) {
    setPose(Poses::home(m_height, m_config), speed);
}

void Hexapod::goSleep(uint8_t speed) {
    setPose(Poses::sleep(m_config), speed);
}

void Hexapod::zeroAllServos(uint8_t speed) {
    for (int i = 0; i < 6; i++) {
        m_legs[i].zeroAngles(speed);
    }
}

void Hexapod::identifyAllLegs(uint8_t speed) {
    for (int i = 0; i < 6; i++) {
        m_legs[i].identifyLeg(speed);
    }
}

// ============================================================================
// Walking
// ============================================================================

void Hexapod::walkForward(uint8_t speed) {
    switch (m_walkPhase) {
        case 0: setPose(Poses::walkPhase0(m_height, m_config), speed); break;
        case 1: setPose(Poses::walkPhase1(m_height, m_config), speed); break;
        case 2: setPose(Poses::walkPhase2(m_height, m_config), speed); break;
        case 3: setPose(Poses::walkPhase3(m_height, m_config), speed); break;
        case 4: setPose(Poses::walkPhase4(m_height, m_config), speed); break;
        case 5: setPose(Poses::walkPhase5(m_height, m_config), speed); break;
        case 6: setPose(Poses::walkPhase6(m_height, m_config), speed); break;
        case 7: setPose(Poses::walkPhase7(m_height, m_config), speed); break;
    }
    m_walkPhase = (m_walkPhase + 1) % Poses::WALK_PHASE_COUNT;
}

void Hexapod::walkBackward(uint8_t speed) {
    switch (m_walkPhase) {
        case 0: setPose(Poses::walkPhase4(m_height, m_config), speed); break;
        case 1: setPose(Poses::walkPhase5(m_height, m_config), speed); break;
        case 2: setPose(Poses::walkPhase6(m_height, m_config), speed); break;
        case 3: setPose(Poses::walkPhase7(m_height, m_config), speed); break;
        case 4: setPose(Poses::walkPhase0(m_height, m_config), speed); break;
        case 5: setPose(Poses::walkPhase1(m_height, m_config), speed); break;
        case 6: setPose(Poses::walkPhase2(m_height, m_config), speed); break;
        case 7: setPose(Poses::walkPhase3(m_height, m_config), speed); break;
    }
    m_walkPhase = (m_walkPhase + 1) % Poses::WALK_PHASE_COUNT;
}

void Hexapod::rotateClockwise(uint8_t speed) {
    switch (m_rotatePhase) {
        case 0: setPose(Poses::rotatePhase0(m_height, m_config), speed); break;
        case 1: setPose(Poses::rotatePhase1(m_height, m_config), speed); break;
        case 2: setPose(Poses::rotatePhase2(m_height, m_config), speed); break;
        case 3: setPose(Poses::rotatePhase3(m_height, m_config), speed); break;
        case 4: setPose(Poses::rotatePhase4(m_height, m_config), speed); break;
        case 5: setPose(Poses::rotatePhase5(m_height, m_config), speed); break;
        case 6: setPose(Poses::rotatePhase6(m_height, m_config), speed); break;
        case 7: setPose(Poses::rotatePhase7(m_height, m_config), speed); break;
    }
    m_rotatePhase = (m_rotatePhase + 1) % Poses::ROTATE_PHASE_COUNT;
}

void Hexapod::rotateCounterClockwise(uint8_t speed) {
    switch (m_rotatePhase) {
        case 0: setPose(Poses::rotatePhase4(m_height, m_config), speed); break;
        case 1: setPose(Poses::rotatePhase5(m_height, m_config), speed); break;
        case 2: setPose(Poses::rotatePhase6(m_height, m_config), speed); break;
        case 3: setPose(Poses::rotatePhase7(m_height, m_config), speed); break;
        case 4: setPose(Poses::rotatePhase0(m_height, m_config), speed); break;
        case 5: setPose(Poses::rotatePhase1(m_height, m_config), speed); break;
        case 6: setPose(Poses::rotatePhase2(m_height, m_config), speed); break;
        case 7: setPose(Poses::rotatePhase3(m_height, m_config), speed); break;
    }
    m_rotatePhase = (m_rotatePhase + 1) % Poses::ROTATE_PHASE_COUNT;
}

// ============================================================================
// Accessors
// ============================================================================

HexapodLeg& Hexapod::getLeg(LegId id) {
    uint8_t index = static_cast<uint8_t>(id);
    if (index > 5) index = 0;
    return m_legs[index];
}

const HexapodLeg& Hexapod::getLeg(LegId id) const {
    uint8_t index = static_cast<uint8_t>(id);
    if (index > 5) index = 0;
    return m_legs[index];
}

// ============================================================================
// Smooth Motion
// ============================================================================

bool Hexapod::updateSmooth() {
    if (!m_initialized) return false;
    
    bool anyChanged = false;
    
    // Update interpolation and send commands for all servos
    for (int i = 0; i < 6; i++) {
        HexapodServo& foot = m_legs[i].getFootServo();
        HexapodServo& knee = m_legs[i].getKneeServo();
        HexapodServo& hip = m_legs[i].getHipServo();
        
        // Update interpolation state
        if (foot.updateInterpolation()) anyChanged = true;
        if (knee.updateInterpolation()) anyChanged = true;
        if (hip.updateInterpolation()) anyChanged = true;
        
        // Send CURRENT (interpolated) positions to hardware
        sendServoCommand(foot.getChannel(), foot.getCurrentAngle(), foot.isReverse(), foot.getTargetSpeed());
        sendServoCommand(knee.getChannel(), knee.getCurrentAngle(), knee.isReverse(), knee.getTargetSpeed());
        sendServoCommand(hip.getChannel(), hip.getCurrentAngle(), hip.isReverse(), hip.getTargetSpeed());
    }
    
    return anyChanged;
}

bool Hexapod::isMoving() const {
    for (int i = 0; i < 6; i++) {
        if (m_legs[i].getFootServo().isMoving() ||
            m_legs[i].getKneeServo().isMoving() ||
            m_legs[i].getHipServo().isMoving()) {
            return true;
        }
    }
    return false;
}

void Hexapod::snapToTargets() {
    for (int i = 0; i < 6; i++) {
        m_legs[i].getFootServo().snapToTarget();
        m_legs[i].getKneeServo().snapToTarget();
        m_legs[i].getHipServo().snapToTarget();
    }
}

bool Hexapod::setPoseSmooth(const KeyFrame& frame, uint8_t speed, uint16_t durationMs) {
    bool success = true;
    success &= m_legs[0].setFootPositionSmooth(frame.LFront, speed, durationMs);
    success &= m_legs[1].setFootPositionSmooth(frame.LMiddle, speed, durationMs);
    success &= m_legs[2].setFootPositionSmooth(frame.LBack, speed, durationMs);
    success &= m_legs[3].setFootPositionSmooth(frame.RFront, speed, durationMs);
    success &= m_legs[4].setFootPositionSmooth(frame.RMiddle, speed, durationMs);
    success &= m_legs[5].setFootPositionSmooth(frame.RBack, speed, durationMs);
    return success;
}

void Hexapod::goHomeSmooth(uint8_t speed) {
    setPoseSmooth(Poses::home(m_height, m_config), speed);
}

void Hexapod::goSleepSmooth(uint8_t speed) {
    setPoseSmooth(Poses::sleep(m_config), speed);
}

} // namespace Hexapod
