/*
 * 360° LiDAR Scanner with Stepper Motor
 * Uses A4988 stepper driver (Step pin 3, Direction pin 2)
 * Takes TF Luna distance reading at each position via I2C
 * Sends step, angle, distance, strength to Python (lidar_live.py) for 2D mapping + CSV logging
 *
 * Output line format:  step,angle_deg,dist_cm,flux
 * Lines starting with '#' are comments/markers; "#SCAN n" starts each sweep.
 *
 * TFLuna-I2C Library: https://github.com/budryerson/TFLuna-I2C.git
 */

#include <Wire.h>
#include <TFLI2C.h>  // TFLuna I2C library

// ---- Motor / microstepping ----
// Set to match the A4988 MS1/MS2/MS3 pins:
//   LOW LOW LOW = 1, HIGH LOW LOW = 2, LOW HIGH LOW = 4, HIGH HIGH LOW = 8, HIGH HIGH HIGH = 16
const int MICROSTEPS = 16;
const int STEPS_PER_REV = 200 * MICROSTEPS;  // pulses for one real 360° turn

// ADJUSTABLE Timing
const unsigned int STEP_DELAY_US = 1000;  // Delay between step pulses (smaller = faster; too small → motor stalls)
const int DATA_SEND_INTERVAL = 8;         // Read + send every N pulses (8 @ 1/16 = every 0.9°)

// A4988 Stepper driver pins
const int STEP_PIN = 7;
const int DIR_PIN = 6;

// TF Luna I2C sensor
TFLI2C tfLuna;
const uint8_t TFL_ADDR = 0x10;  // TF Luna default I2C address
int16_t tfDist = 0;    // Distance measurement
int16_t tfFlux = 0;    // Signal strength
int16_t tfTemp = 0;    // Temperature

// Scanning variables
int currentStep = 0;
const float DEGREES_PER_STEP = 360.0 / STEPS_PER_REV;
const int TOTAL_STEPS = STEPS_PER_REV;  // Full 360° rotation
uint16_t scanNum = 0;         // CHANGED: sweep counter for "#SCAN n" markers

void setup() {
  // Initialize serial communication with PC
  Serial.begin(57600);          // unchanged — lidar_live.py now uses 57600

  // Initialize I2C for TF Luna
  Wire.begin();

  // Configure stepper driver pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Set direction (clockwise)
  digitalWrite(DIR_PIN, HIGH);
  reverseDirection();

  // Wait for connections to stabilize
  delay(100);

  // CHANGED: status lines prefixed with '#' so the logger ignores them
  Serial.println("#LiDAR Scanner Initialized");
  Serial.println("#Format: step,angle_deg,dist_cm,flux");

  // Test TF Luna communication
  if (tfLuna.getData(tfDist, tfFlux, tfTemp, TFL_ADDR)) {
    Serial.println("#TF Luna connected successfully");
  } else {
    Serial.println("#TF Luna connection failed - check I2C wiring");
  }

  delay(1000);

  Serial.print("#SCAN ");       // CHANGED: mark start of first sweep
  Serial.println(scanNum);
}

void loop() {
  // CHANGED: send every reading raw (dist + flux); filtering happens in Python
  if (currentStep % DATA_SEND_INTERVAL == 0) {
    if (tfLuna.getData(tfDist, tfFlux, tfTemp, TFL_ADDR)) {
      Serial.print(currentStep);
      Serial.print(",");
      Serial.print(currentStep * DEGREES_PER_STEP, 2);  // computed from step count, no float drift
      Serial.print(",");
      Serial.print(tfDist);
      Serial.print(",");
      Serial.println(tfFlux);
    } else {
      Serial.println("#read_fail");
    }
  }

  // Move stepper one step
  takeStep();
  currentStep++;

  // Reset after full rotation
  if (currentStep >= TOTAL_STEPS) {
    currentStep = 0;
    scanNum++;
    Serial.print("#SCAN ");     // CHANGED: replaces SWEEP_COMPLETE
    Serial.println(scanNum);
  }

  // Wait before next step
  delayMicroseconds(STEP_DELAY_US);
}

/**
 * Trigger one step on the A4988 stepper driver
 */
void takeStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);  // Minimum pulse width for A4988
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(2);
}

/**
 * Optional: Change direction of rotation
 * Call this function to reverse direction
 */
void reverseDirection() {
  digitalWrite(DIR_PIN, !digitalRead(DIR_PIN));
}