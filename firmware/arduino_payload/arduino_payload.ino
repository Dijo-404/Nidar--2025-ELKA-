/*
 * Arduino Uno 10-Payload Drop Controller
 *
 * Controls 10 servos via PCA9685 PWM driver for sequential payload delivery.
 * Triggered by PWM signal from Cube Orange flight controller (AUX 2).
 *
 * Operation:
 *   - Each trigger cycle drops ONE payload from the next servo
 *   - Servos 0-9 open sequentially (first trigger = servo 0, second = servo 1,
 * etc.)
 *   - After switch is triggered (>1400us) and reset (<1100us), next payload is
 * armed
 *
 * PWM Input:
 *   - Pin 2 (interrupt capable)
 *   - Min: 1051us, Max: 2000us
 *   - Trigger threshold: >1400us (high = drop)
 *   - Reset threshold: <1100us (low = armed for next drop)
 *
 * Hardware:
 *   Arduino Uno <-> PCA9685 16-Channel Servo Driver
 *   PCA9685 Channels 0-9 -> 10 Payload Servos
 *   Cube Orange AUX 2 -> Arduino Pin 2
 *
 * Wiring:
 *   Cube Orange      Arduino Uno
 *   -----------      -----------
 *   AUX 2 Signal ->  Pin 2
 *   GND          ->  GND
 *
 *   Arduino Uno    PCA9685
 *   -----------    -------
 *   5V         ->  VCC
 *   GND        ->  GND
 *   A4 (SDA)   ->  SDA
 *   A5 (SCL)   ->  SCL
 *
 *   External 5-6V power for servos -> PCA9685 V+ pin
 *
 * Author: Nidar Project
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// ==================== CONFIGURATION ====================

// PWM Input Pin (must be interrupt capable: Pin 2 or 3 on Uno)
#define PWM_INPUT_PIN 2

// PWM Thresholds (microseconds) - Cube Orange AUX 2
#define PWM_MIN 1051           // Minimum PWM from FC
#define PWM_MAX 2000           // Maximum PWM from FC
#define PWM_DROP_TRIGGER 1400  // PWM > this = DROP command
#define PWM_RESET_TRIGGER 1100 // PWM < this = Reset/arm next drop
#define PWM_DEBOUNCE_MS 300    // Minimum time between state changes

// PCA9685 Settings
#define PCA9685_ADDRESS 0x40 // Default I2C address
#define SERVO_FREQ 50        // 50Hz for standard servos

// Servo PWM values (12-bit: 0-4095)
// For 50Hz: 4096 steps = 20ms period
// 1ms = 205, 1.5ms = 307, 2ms = 410
#define SERVO_HOLD 200 // Closed/hold position (~1ms)
#define SERVO_DROP 450 // Open/drop position (~2.2ms)

// Timing
#define DROP_DURATION_MS 1500 // Time servo stays open for drop
#define SERIAL_BAUD 115200

// Payload Configuration
#define NUM_SERVOS 10 // Number of servos (channels 0-9)

// ==================== GLOBALS ====================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Servo state
bool servoInitialized = false;
int currentServo = 0;    // Which servo drops next (0-9)
int totalDropped = 0;    // Total payloads dropped
bool isDropping = false; // Currently in drop sequence

// PWM measurement (using interrupts)
volatile unsigned long pwmRiseTime = 0;
volatile unsigned long pwmPulseWidth = 0;
volatile bool pwmNewReading = false;

// State machine for trigger
enum TriggerState {
  STATE_ARMED,     // Ready to drop, waiting for high PWM
  STATE_TRIGGERED, // High PWM detected, dropping
  STATE_WAIT_RESET // Waiting for low PWM to reset
};
TriggerState triggerState = STATE_ARMED;
unsigned long lastStateChange = 0;

// Serial command buffer
String inputBuffer = "";

// ==================== PWM INPUT FUNCTIONS ====================

void pwmInterrupt() {
  if (digitalRead(PWM_INPUT_PIN) == HIGH) {
    pwmRiseTime = micros();
  } else {
    if (pwmRiseTime > 0) {
      pwmPulseWidth = micros() - pwmRiseTime;
      pwmNewReading = true;
    }
  }
}

unsigned long getPWMPulseWidth() {
  noInterrupts();
  unsigned long pw = pwmPulseWidth;
  pwmNewReading = false;
  interrupts();
  return pw;
}

// ==================== SERVO FUNCTIONS ====================

void setServoPosition(int channel, int position) {
  if (!servoInitialized || channel < 0 || channel >= NUM_SERVOS) {
    return;
  }
  position = constrain(position, 100, 600);
  pwm.setPWM(channel, 0, position);
}

void closeAllServos() {
  Serial.println(F("Closing all servos..."));
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoPosition(i, SERVO_HOLD);
    delay(50); // Stagger to reduce power surge
  }
  Serial.println(F("All servos closed"));
}

bool dropPayload() {
  // Check if we have payloads remaining
  if (currentServo >= NUM_SERVOS) {
    Serial.println(F("ERROR: All payloads dropped!"));
    return false;
  }

  if (isDropping) {
    Serial.println(F("ERROR: Drop in progress"));
    return false;
  }

  isDropping = true;

  Serial.print(F("DROPPING Payload "));
  Serial.print(currentServo + 1);
  Serial.print(F("/"));
  Serial.print(NUM_SERVOS);
  Serial.print(F(" (Servo CH"));
  Serial.print(currentServo);
  Serial.println(F(")"));

  // Open this servo
  setServoPosition(currentServo, SERVO_DROP);
  delay(DROP_DURATION_MS);

  // Close this servo
  setServoPosition(currentServo, SERVO_HOLD);

  totalDropped++;
  currentServo++; // Move to next servo for next drop
  isDropping = false;

  Serial.print(F("OK: Dropped. Remaining: "));
  Serial.println(NUM_SERVOS - currentServo);

  return true;
}

void executeTest() {
  Serial.println(F("CMD: TEST"));
  Serial.println(F("Testing all servos..."));

  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("  Servo "));
    Serial.print(i);
    Serial.print(F(": OPEN"));
    setServoPosition(i, SERVO_DROP);
    delay(500);

    Serial.println(F(" -> CLOSE"));
    setServoPosition(i, SERVO_HOLD);
    delay(300);
  }

  Serial.println(F("OK: TEST COMPLETE"));
}

void executeReset() {
  Serial.println(F("CMD: RESET"));
  currentServo = 0;
  totalDropped = 0;
  triggerState = STATE_ARMED;
  closeAllServos();
  Serial.print(F("OK: Reset. Payloads ready: "));
  Serial.println(NUM_SERVOS);
}

void printStatus() {
  Serial.println(F("======= STATUS ======="));
  Serial.print(F("Next servo: "));
  Serial.print(currentServo);
  Serial.print(F("/"));
  Serial.println(NUM_SERVOS);
  Serial.print(F("Total dropped: "));
  Serial.println(totalDropped);
  Serial.print(F("Remaining: "));
  Serial.println(NUM_SERVOS - currentServo);
  Serial.print(F("PWM Input: "));
  Serial.print(pwmPulseWidth);
  Serial.println(F(" us"));
  Serial.print(F("State: "));
  switch (triggerState) {
  case STATE_ARMED:
    Serial.println(F("ARMED"));
    break;
  case STATE_TRIGGERED:
    Serial.println(F("TRIGGERED"));
    break;
  case STATE_WAIT_RESET:
    Serial.println(F("WAIT_RESET"));
    break;
  }
  Serial.print(F("Is Dropping: "));
  Serial.println(isDropping ? F("YES") : F("NO"));
  Serial.println(F("======================"));
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "DROP") {
    dropPayload();
  } else if (cmd == "HOLD" || cmd == "CLOSE") {
    closeAllServos();
  } else if (cmd == "TEST") {
    executeTest();
  } else if (cmd == "STATUS") {
    printStatus();
  } else if (cmd == "RESET") {
    executeReset();
  } else if (cmd.length() > 0) {
    Serial.print(F("ERROR: Unknown: "));
    Serial.println(cmd);
    Serial.println(F("Commands: DROP, CLOSE, TEST, STATUS, RESET"));
  }
}

// ==================== PWM TRIGGER STATE MACHINE ====================

void checkPWMTrigger() {
  unsigned long currentPWM = getPWMPulseWidth();

  // Ignore invalid readings
  if (currentPWM < 900 || currentPWM > 2200) {
    return;
  }

  unsigned long now = millis();

  // Debounce check
  if ((now - lastStateChange) < PWM_DEBOUNCE_MS) {
    return;
  }

  switch (triggerState) {
  case STATE_ARMED:
    // Waiting for high PWM to trigger drop
    if (currentPWM > PWM_DROP_TRIGGER) {
      Serial.print(F("PWM HIGH: "));
      Serial.print(currentPWM);
      Serial.println(F(" us -> DROPPING"));

      if (dropPayload()) {
        triggerState = STATE_WAIT_RESET;
        lastStateChange = now;
      }
    }
    break;

  case STATE_WAIT_RESET:
    // Waiting for low PWM to arm next drop
    if (currentPWM < PWM_RESET_TRIGGER) {
      Serial.print(F("PWM LOW: "));
      Serial.print(currentPWM);
      Serial.println(F(" us -> ARMED"));

      triggerState = STATE_ARMED;
      lastStateChange = now;

      if (currentServo < NUM_SERVOS) {
        Serial.print(F("Ready for drop "));
        Serial.print(currentServo + 1);
        Serial.print(F("/"));
        Serial.println(NUM_SERVOS);
      } else {
        Serial.println(F("All payloads dropped!"));
      }
    }
    break;

  default:
    triggerState = STATE_ARMED;
    break;
  }
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  delay(100);

  Serial.println();
  Serial.println(F("=================================="));
  Serial.println(F("10-PAYLOAD DROP CONTROLLER"));
  Serial.println(F("=================================="));

  // PWM input setup
  pinMode(PWM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmInterrupt, CHANGE);
  Serial.print(F("PWM Input: Pin "));
  Serial.println(PWM_INPUT_PIN);
  Serial.print(F("Drop trigger: >"));
  Serial.print(PWM_DROP_TRIGGER);
  Serial.println(F(" us"));
  Serial.print(F("Reset trigger: <"));
  Serial.print(PWM_RESET_TRIGGER);
  Serial.println(F(" us"));

  // I2C and PCA9685 setup
  Wire.begin();
  Serial.print(F("Initializing PCA9685... "));
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  Serial.println(F("OK"));
  servoInitialized = true;

  // Close all servos initially
  closeAllServos();

  Serial.println();
  Serial.print(F("Payloads ready: "));
  Serial.println(NUM_SERVOS);
  Serial.println();
  Serial.println(F("Commands: DROP, CLOSE, TEST, STATUS, RESET"));
  Serial.println(F("Or trigger via PWM on Pin 2"));
  Serial.println();
  Serial.println(F("ARMED - Waiting for trigger..."));
  Serial.println();
}

// ==================== LOOP ====================

void loop() {
  // Check PWM trigger from flight controller
  checkPWMTrigger();

  // Check serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  delay(10);
}
