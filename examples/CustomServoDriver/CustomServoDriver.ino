/**
 * CustomServoDriver.ino
 * 
 * Demonstrates advanced usage of the HexapodKinematics library:
 * - Accessing individual servo objects for direct control
 * - Reading servo angles and positions
 * - Debug output for servo movements
 * 
 * This example is useful for:
 * - Debugging servo wiring and direction issues
 * - Understanding the internal servo angle calculations
 * - Building custom control interfaces
 * 
 * Note: The library has built-in support for PCA9685, Maestro, and Direct PWM.
 * For unsupported hardware, you can access servo angles and implement your
 * own driver by reading the calculated angles from HexapodServo objects.
 */

#include <HexapodKinematics.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Choose your driver (0=PCA9685, 1=Maestro, 2=DirectPWM)
const uint8_t DRIVER_TYPE = 0;

// Enable debug output
bool verboseMode = true;

// Create hexapod with default config
Hexapod::Hexapod robot(DRIVER_TYPE);

// ============================================================================
// DEBUG HELPERS
// ============================================================================

/**
 * Print all servo angles for debugging
 */
void printAllServoAngles() {
    Serial.println(F("\n--- Current Servo Angles ---"));
    
    const char* legNames[] = {"LFront", "LMiddle", "LBack", "RFront", "RMiddle", "RBack"};
    
    for (int i = 0; i < 6; i++) {
        Hexapod::LegId legId = static_cast<Hexapod::LegId>(i);
        Hexapod::HexapodLeg& leg = robot.getLeg(legId);
        
        Serial.print(legNames[i]);
        Serial.print(F(": Hip="));
        Serial.print(leg.getHipServo().getTargetAngle(), 1);
        Serial.print(F("° Knee="));
        Serial.print(leg.getKneeServo().getTargetAngle(), 1);
        Serial.print(F("° Foot="));
        Serial.print(leg.getFootServo().getTargetAngle(), 1);
        Serial.println(F("°"));
    }
    Serial.println(F("----------------------------\n"));
}

/**
 * Print servo channel mapping
 */
void printServoMapping() {
    Serial.println(F("\n--- Servo Channel Mapping ---"));
    
    const char* legNames[] = {"LFront", "LMiddle", "LBack", "RFront", "RMiddle", "RBack"};
    
    for (int i = 0; i < 6; i++) {
        Hexapod::LegId legId = static_cast<Hexapod::LegId>(i);
        Hexapod::HexapodLeg& leg = robot.getLeg(legId);
        
        Serial.print(legNames[i]);
        Serial.print(F(": Hip=Ch"));
        Serial.print(leg.getHipServo().getChannel());
        Serial.print(leg.getHipServo().isReverse() ? F("(R)") : F(""));
        Serial.print(F(" Knee=Ch"));
        Serial.print(leg.getKneeServo().getChannel());
        Serial.print(leg.getKneeServo().isReverse() ? F("(R)") : F(""));
        Serial.print(F(" Foot=Ch"));
        Serial.print(leg.getFootServo().getChannel());
        Serial.println(leg.getFootServo().isReverse() ? F("(R)") : F(""));
    }
    Serial.println(F("(R) = Reversed"));
    Serial.println(F("-----------------------------\n"));
}

/**
 * Print foot positions for all legs
 */
void printFootPositions() {
    Serial.println(F("\n--- Foot Positions (mm) ---"));
    
    const char* legNames[] = {"LFront", "LMiddle", "LBack", "RFront", "RMiddle", "RBack"};
    
    for (int i = 0; i < 6; i++) {
        Hexapod::LegId legId = static_cast<Hexapod::LegId>(i);
        Hexapod::HexapodLeg& leg = robot.getLeg(legId);
        Hexapod::Position pos = leg.getFootPosition();
        
        Serial.print(legNames[i]);
        Serial.print(F(": X="));
        Serial.print(pos.x);
        Serial.print(F(" Y="));
        Serial.print(pos.y);
        Serial.print(F(" Z="));
        Serial.println(pos.z);
    }
    Serial.println(F("---------------------------\n"));
}

/**
 * Test a single servo by moving it
 */
void testServo(uint8_t legIndex, const char* servoType) {
    if (legIndex >= 6) return;
    
    Hexapod::LegId legId = static_cast<Hexapod::LegId>(legIndex);
    Hexapod::HexapodLeg& leg = robot.getLeg(legId);
    
    Hexapod::HexapodServo* servo = nullptr;
    if (strcmp(servoType, "hip") == 0) servo = &leg.getHipServo();
    else if (strcmp(servoType, "knee") == 0) servo = &leg.getKneeServo();
    else if (strcmp(servoType, "foot") == 0) servo = &leg.getFootServo();
    
    if (servo == nullptr) return;
    
    Serial.print(F("Testing "));
    Serial.print(servo->getId());
    Serial.println(F(" - moving +10 degrees"));
    
    float currentAngle = servo->getTargetAngle();
    servo->setAngle(currentAngle + 10);
    robot.update();
    
    delay(500);
    
    Serial.println(F("Returning to original position"));
    servo->setAngle(currentAngle);
    robot.update();
}

// ============================================================================
// CUSTOM DRIVER EXAMPLE
// ============================================================================

/**
 * Example: Read servo angles and send to your own hardware
 * 
 * If you have unsupported hardware, you can:
 * 1. Use robot.setPose() or robot.goHome() to calculate angles
 * 2. Read the calculated angles from each servo
 * 3. Send them to your hardware using your own code
 */
void exampleCustomDriverUsage() {
    Serial.println(F("\n--- Custom Driver Example ---"));
    Serial.println(F("Calculating home position angles..."));
    
    // Set pose (this calculates IK but doesn't necessarily move hardware
    // if you haven't called begin() or are using a different approach)
    robot.goHome(50);
    
    // Now read the calculated angles
    for (int i = 0; i < 6; i++) {
        Hexapod::LegId legId = static_cast<Hexapod::LegId>(i);
        Hexapod::HexapodLeg& leg = robot.getLeg(legId);
        
        // Get calculated angles
        float hipAngle = leg.getHipServo().getTargetAngle();
        float kneeAngle = leg.getKneeServo().getTargetAngle();
        float footAngle = leg.getFootServo().getTargetAngle();
        
        // Get channel assignments
        uint8_t hipCh = leg.getHipServo().getChannel();
        uint8_t kneeCh = leg.getKneeServo().getChannel();
        uint8_t footCh = leg.getFootServo().getChannel();
        
        // Check if reversed
        bool hipRev = leg.getHipServo().isReverse();
        bool kneeRev = leg.getKneeServo().isReverse();
        bool footRev = leg.getFootServo().isReverse();
        
        // Here you would send to your custom hardware:
        // myCustomDriver.setServo(hipCh, hipRev ? -hipAngle : hipAngle);
        // myCustomDriver.setServo(kneeCh, kneeRev ? -kneeAngle : kneeAngle);
        // myCustomDriver.setServo(footCh, footRev ? -footAngle : footAngle);
        
        Serial.print(F("Leg "));
        Serial.print(i);
        Serial.print(F(": Hip(ch"));
        Serial.print(hipCh);
        Serial.print(F(")="));
        Serial.print(hipAngle, 1);
        Serial.print(F("° Knee(ch"));
        Serial.print(kneeCh);
        Serial.print(F(")="));
        Serial.print(kneeAngle, 1);
        Serial.print(F("° Foot(ch"));
        Serial.print(footCh);
        Serial.print(F(")="));
        Serial.print(footAngle, 1);
        Serial.println(F("°"));
    }
    
    Serial.println(F("-----------------------------\n"));
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println(F("HexapodKinematics - Custom/Debug Example"));
    Serial.println(F("========================================\n"));
    
    // Initialize robot
    robot.begin();
    
    if (!robot.isInitialized()) {
        Serial.println(F("WARNING: Driver initialization failed."));
        Serial.println(F("Continuing anyway for debug purposes.\n"));
    }
    
    robot.setHeight(5);
    
    // Show initial configuration
    printServoMapping();
    
    Serial.println(F("Commands:"));
    Serial.println(F("  h - Go home"));
    Serial.println(F("  s - Go sleep"));
    Serial.println(F("  w - Walk step"));
    Serial.println(F("  a - Print all servo angles"));
    Serial.println(F("  p - Print foot positions"));
    Serial.println(F("  m - Print servo mapping"));
    Serial.println(F("  c - Custom driver example"));
    Serial.println(F("  0-5 + j - Test leg N joint (hip/knee/foot)"));
    Serial.println(F("  v - Toggle verbose mode"));
    Serial.println();
}

uint8_t selectedLeg = 0;

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'h':
                Serial.println(F("\n> Going home"));
                robot.goHome(50);
                robot.update();
                if (verboseMode) printAllServoAngles();
                break;
                
            case 's':
                Serial.println(F("\n> Going to sleep"));
                robot.goSleep(50);
                robot.update();
                if (verboseMode) printAllServoAngles();
                break;
                
            case 'w':
                Serial.println(F("\n> Walk step"));
                robot.walkForward(50);
                robot.update();
                if (verboseMode) printAllServoAngles();
                break;
                
            case 'a':
                printAllServoAngles();
                break;
                
            case 'p':
                printFootPositions();
                break;
                
            case 'm':
                printServoMapping();
                break;
                
            case 'c':
                exampleCustomDriverUsage();
                break;
                
            case '0': case '1': case '2': case '3': case '4': case '5':
                selectedLeg = cmd - '0';
                Serial.print(F("Selected leg "));
                Serial.println(selectedLeg);
                break;
                
            case 'j':
                Serial.println(F("Enter joint (h=hip, k=knee, f=foot):"));
                while (!Serial.available());
                {
                    char joint = Serial.read();
                    const char* jointName = (joint == 'h') ? "hip" : 
                                           (joint == 'k') ? "knee" : 
                                           (joint == 'f') ? "foot" : nullptr;
                    if (jointName) testServo(selectedLeg, jointName);
                }
                break;
                
            case 'v':
                verboseMode = !verboseMode;
                Serial.print(F("Verbose mode: "));
                Serial.println(verboseMode ? F("ON") : F("OFF"));
                break;
        }
    }
}
