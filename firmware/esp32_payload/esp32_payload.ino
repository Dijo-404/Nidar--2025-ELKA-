/*
 * ESP32 Payload Drop Controller
 * 
 * Controls servo via PCA9685 PWM driver for payload delivery.
 * Receives commands via Serial (USB) from companion computer.
 * 
 * Commands (Serial):
 *   DROP    - Execute drop sequence
 *   HOLD    - Set servo to hold position
 *   TEST    - Test servo movement
 *   STATUS  - Get current status
 * 
 * Hardware:
 *   ESP32 <-> PCA9685 (I2C: SDA=21, SCL=22)
 *   PCA9685 -> Servo (Channel 0)
 * 
 * Author: Nidar Project
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== CONFIGURATION ====================

// PCA9685 Settings
#define PCA9685_ADDRESS 0x40      // Default I2C address
#define SERVO_CHANNEL   0         // Which channel servo is connected to
#define SERVO_FREQ      50        // 50Hz for standard servos

// Servo PWM values (adjust for your servo)
// PCA9685 uses 12-bit values (0-4095) for duty cycle
// For 50Hz: 4096 steps = 20ms period
// 1ms pulse = ~205, 2ms pulse = ~410
#define SERVO_MIN       150       // ~0.75ms - Minimum position
#define SERVO_MAX       600       // ~3ms - Maximum position
#define SERVO_HOLD      200       // Hold/locked position
#define SERVO_DROP      450       // Drop/release position

// Timing
#define DROP_DURATION_MS  2000    // Time to stay in drop position
#define SERIAL_BAUD       115200  // Serial communication speed

// I2C Pins (ESP32 default)
#define I2C_SDA 21
#define I2C_SCL 22

// ==================== GLOBALS ====================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

bool servoInitialized = false;
int currentPosition = SERVO_HOLD;
int dropCount = 0;

// ==================== FUNCTIONS ====================

void setServoPosition(int position) {
    if (!servoInitialized) {
        Serial.println("ERROR: Servo not initialized");
        return;
    }
    
    // Clamp to valid range
    position = constrain(position, SERVO_MIN, SERVO_MAX);
    
    pwm.setPWM(SERVO_CHANNEL, 0, position);
    currentPosition = position;
    
    Serial.print("SERVO: ");
    Serial.println(position);
}

void executeHold() {
    Serial.println("CMD: HOLD");
    setServoPosition(SERVO_HOLD);
    Serial.println("OK: HOLD");
}

void executeDrop() {
    Serial.println("CMD: DROP");
    
    // Move to drop position
    setServoPosition(SERVO_DROP);
    Serial.println("DROPPING...");
    
    // Wait for payload to release
    delay(DROP_DURATION_MS);
    
    // Return to hold position
    setServoPosition(SERVO_HOLD);
    
    dropCount++;
    Serial.print("OK: DROP #");
    Serial.println(dropCount);
}

void executeTest() {
    Serial.println("CMD: TEST");
    
    // Move to drop position
    Serial.println("TEST: Moving to DROP position");
    setServoPosition(SERVO_DROP);
    delay(1000);
    
    // Return to hold position
    Serial.println("TEST: Moving to HOLD position");
    setServoPosition(SERVO_HOLD);
    delay(500);
    
    Serial.println("OK: TEST COMPLETE");
}

void printStatus() {
    Serial.println("--- STATUS ---");
    Serial.print("Initialized: ");
    Serial.println(servoInitialized ? "YES" : "NO");
    Serial.print("Position: ");
    Serial.println(currentPosition);
    Serial.print("Drops: ");
    Serial.println(dropCount);
    Serial.print("Hold PWM: ");
    Serial.println(SERVO_HOLD);
    Serial.print("Drop PWM: ");
    Serial.println(SERVO_DROP);
    Serial.println("--------------");
}

void processCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "DROP") {
        executeDrop();
    } else if (cmd == "HOLD") {
        executeHold();
    } else if (cmd == "TEST") {
        executeTest();
    } else if (cmd == "STATUS") {
        printStatus();
    } else if (cmd.length() > 0) {
        Serial.print("ERROR: Unknown command: ");
        Serial.println(cmd);
        Serial.println("Valid commands: DROP, HOLD, TEST, STATUS");
    }
}

// ==================== SETUP ====================

void setup() {
    // Initialize Serial
    Serial.begin(SERIAL_BAUD);
    delay(100);
    
    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32 Payload Drop Controller");
    Serial.println("================================");
    
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Initialize PCA9685
    Serial.print("Initializing PCA9685... ");
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // Internal oscillator
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
    
    Serial.println("OK");
    servoInitialized = true;
    
    // Set initial position
    Serial.println("Setting servo to HOLD position");
    setServoPosition(SERVO_HOLD);
    
    Serial.println();
    Serial.println("Ready for commands:");
    Serial.println("  DROP   - Drop payload");
    Serial.println("  HOLD   - Hold position");
    Serial.println("  TEST   - Test servo");
    Serial.println("  STATUS - Show status");
    Serial.println();
}

// ==================== LOOP ====================

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        processCommand(cmd);
    }
    
    // Small delay to prevent CPU hogging
    delay(10);
}
