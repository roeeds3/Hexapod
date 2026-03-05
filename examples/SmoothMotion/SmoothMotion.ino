/**
 * SmoothMotion.ino
 * 
 * Demonstrates the smooth motion feature of HexapodKinematics library.
 * Servos move smoothly with acceleration/deceleration instead of jumping
 * instantly to target positions.
 * 
 * This example shows:
 * - Using updateSmooth() for interpolated motion
 * - Using isMoving() to wait for completion
 * - Different speed settings
 * - Comparing instant vs smooth motion
 * 
 * Hardware: Same as BasicWalk example
 */

#include <HexapodKinematics.h>

// Create hexapod with PCA9685 driver
Hexapod::Hexapod robot(0);

// Timing for smooth updates
const unsigned long UPDATE_INTERVAL = 20;  // 50Hz update rate
unsigned long lastUpdate = 0;

// State
bool useSmoothMotion = true;
uint8_t currentSpeed = 50;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println(F("HexapodKinematics - Smooth Motion Example"));
    Serial.println(F("=========================================="));
    Serial.println();
    Serial.println(F("Commands:"));
    Serial.println(F("  h - Go home (using current mode)"));
    Serial.println(F("  s - Go sleep (using current mode)"));
    Serial.println(F("  m - Toggle smooth/instant motion"));
    Serial.println(F("  1-9 - Set speed (1=slow, 9=fast)"));
    Serial.println(F("  d - Demo: compare smooth vs instant"));
    Serial.println(F("  w - Wave animation"));
    Serial.println();
    
    // Initialize robot
    robot.begin();
    
    if (!robot.isInitialized()) {
        Serial.println(F("ERROR: Robot initialization failed!"));
        while (1) delay(1000);
    }
    
    robot.setHeight(5);
    
    // Start with smooth motion to home
    Serial.println(F("Moving to home position (smooth)..."));
    robot.goHomeSmooth(50);
    waitForMove();
    
    Serial.println(F("Ready!"));
    Serial.print(F("Mode: SMOOTH, Speed: "));
    Serial.println(currentSpeed);
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }
    
    // Keep updating smooth motion if in progress
    if (robot.isMoving()) {
        unsigned long now = millis();
        if (now - lastUpdate >= UPDATE_INTERVAL) {
            lastUpdate = now;
            robot.updateSmooth();
        }
    }
}

void handleCommand(char cmd) {
    switch (cmd) {
        case 'h':
        case 'H':
            Serial.println(F("Going home..."));
            if (useSmoothMotion) {
                robot.goHomeSmooth(currentSpeed);
                waitForMove();
            } else {
                robot.goHome(currentSpeed);
                robot.update();
            }
            Serial.println(F("Done."));
            break;
            
        case 's':
        case 'S':
            Serial.println(F("Going to sleep..."));
            if (useSmoothMotion) {
                robot.goSleepSmooth(currentSpeed);
                waitForMove();
            } else {
                robot.goSleep(currentSpeed);
                robot.update();
            }
            Serial.println(F("Done."));
            break;
            
        case 'm':
        case 'M':
            useSmoothMotion = !useSmoothMotion;
            Serial.print(F("Mode: "));
            Serial.println(useSmoothMotion ? F("SMOOTH") : F("INSTANT"));
            break;
            
        case '1': case '2': case '3': case '4': case '5':
        case '6': case '7': case '8': case '9':
            currentSpeed = (cmd - '0') * 11;  // Map 1-9 to ~11-99
            Serial.print(F("Speed: "));
            Serial.println(currentSpeed);
            break;
            
        case 'd':
        case 'D':
            runDemo();
            break;
            
        case 'w':
        case 'W':
            waveAnimation();
            break;
    }
}

/**
 * Wait for smooth motion to complete
 */
void waitForMove() {
    while (robot.isMoving()) {
        robot.updateSmooth();
        delay(UPDATE_INTERVAL);
    }
}

/**
 * Demo comparing instant vs smooth motion
 */
void runDemo() {
    Serial.println(F("\n=== Motion Comparison Demo ===\n"));
    
    // Get current config for poses
    const Hexapod::HexapodConfig& cfg = robot.getConfig();
    uint8_t height = robot.getHeight();
    
    // Define two distinct poses
    Hexapod::KeyFrame pose1 = Hexapod::Poses::home(height, cfg);
    Hexapod::KeyFrame pose2 = Hexapod::Poses::home(height, cfg);
    
    // Modify pose2 to be different (raise front legs)
    pose2.LFront.z += 40;
    pose2.RFront.z += 40;
    pose2.LFront.x += 30;
    pose2.RFront.x += 30;
    
    // First: Instant motion
    Serial.println(F("1. INSTANT motion (watch servos jump):"));
    delay(1000);
    
    Serial.println(F("   Moving to pose 2..."));
    robot.setPose(pose2, 50);
    robot.update();
    delay(1500);
    
    Serial.println(F("   Moving back to pose 1..."));
    robot.setPose(pose1, 50);
    robot.update();
    delay(1500);
    
    // Second: Smooth motion
    Serial.println(F("\n2. SMOOTH motion (watch servos glide):"));
    delay(1000);
    
    Serial.println(F("   Moving to pose 2..."));
    robot.setPoseSmooth(pose2, 50);
    waitForMove();
    delay(500);
    
    Serial.println(F("   Moving back to pose 1..."));
    robot.setPoseSmooth(pose1, 50);
    waitForMove();
    delay(500);
    
    // Third: Very slow smooth motion
    Serial.println(F("\n3. SLOW smooth motion (speed=20):"));
    delay(1000);
    
    Serial.println(F("   Moving to pose 2..."));
    robot.setPoseSmooth(pose2, 20);
    waitForMove();
    delay(500);
    
    Serial.println(F("   Moving back to pose 1..."));
    robot.setPoseSmooth(pose1, 20);
    waitForMove();
    
    Serial.println(F("\n=== Demo Complete ===\n"));
}

/**
 * Simple wave animation using smooth motion
 */
void waveAnimation() {
    Serial.println(F("Wave animation..."));
    
    const Hexapod::HexapodConfig& cfg = robot.getConfig();
    uint8_t height = robot.getHeight();
    
    // Start from home
    robot.goHomeSmooth(70);
    waitForMove();
    delay(200);
    
    // Wave sequence - lift and wave each leg pair
    for (int wave = 0; wave < 2; wave++) {
        // Front legs up
        Hexapod::KeyFrame pose = Hexapod::Poses::home(height, cfg);
        pose.LFront.z += 50;
        pose.RFront.z += 50;
        robot.setPoseSmooth(pose, 60);
        waitForMove();
        delay(100);
        
        // Front legs down, middle up
        pose = Hexapod::Poses::home(height, cfg);
        pose.LMiddle.z += 50;
        pose.RMiddle.z += 50;
        robot.setPoseSmooth(pose, 60);
        waitForMove();
        delay(100);
        
        // Middle down, back up
        pose = Hexapod::Poses::home(height, cfg);
        pose.LBack.z += 50;
        pose.RBack.z += 50;
        robot.setPoseSmooth(pose, 60);
        waitForMove();
        delay(100);
    }
    
    // Return home
    robot.goHomeSmooth(50);
    waitForMove();
    
    Serial.println(F("Wave complete."));
}
