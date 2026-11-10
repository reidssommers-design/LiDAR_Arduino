/*
 * 360° LiDAR Scanner, timer-driven version
 *
 * Same hardware and output format as 360sweep_nano, but:
 *   - The stepper is driven by a Timer1 interrupt at a constant rate, so the
 *     motor never pauses for sensor reads (smoother and quieter).
 *   - The motor ramps up gently at power-on instead of starting at full speed.
 *   - The TF-Luna runs at READ_HZ (default 250 Hz instead of 100) on a 400 kHz
 *     I2C bus, and it's read on its own clock. Each reading is tagged with
 *     the motor position at the moment it was read.
 *   - Weak/saturated reads are logged with their real values instead of being
 *     dropped (Python filters them from the plot); I2C failures log as 0,0.
 *     So "no return" directions still show up in the CSV.
 *
 * Output line format:  step,angle_deg,dist_cm,flux
 * Lines starting with '#' are comments/markers; "#SCAN n" starts each sweep.
 * NOTE: baud is 115200. Set BAUD = 115200 in lidar_live.py to match.
 *
 * TFLuna-I2C Library: https://github.com/budryerson/TFLuna-I2C.git
 */

#include <Wire.h>
#include <TFLI2C.h>

// ---- Motor / microstepping ----
// Set to match the A4988 MS1/MS2/MS3 pins:
//   LOW LOW LOW = 1, HIGH LOW LOW = 2, LOW HIGH LOW = 4, HIGH HIGH LOW = 8, HIGH HIGH HIGH = 16
const uint8_t  MICROSTEPS    = 16;
const uint16_t STEPS_PER_REV = 200U * MICROSTEPS;   // pulses for one real 360° turn

// ---- Tuning ----
const float    REV_SECONDS = 1.0;    // time per revolution. Points/rev = REV_SECONDS * READ_HZ
const uint16_t READ_HZ     = 250;    // TF-Luna frame rate and read rate (max 250)
const float    START_RPS   = 0.1;    // ramp starts at this speed (revs/second)
const uint16_t RAMP_MS     = 1500;   // time to ramp from START_RPS to full speed
const long     BAUD        = 115200;

// At 250 Hz and 2 s/rev you get 500 points per turn (0.72° apart).
// If it buzzes at some speed (resonance), nudge REV_SECONDS up or down.

// ---- Pins (A4988) ----
// STEP is pulsed from the ISR with direct port writes, so it has to stay on D7 (PORTD bit 7).
const uint8_t STEP_PIN = 7;
const uint8_t DIR_PIN  = 6;
const bool    DIR_LEVEL = LOW;       // same rotation direction as 360sweep_nano

// ---- TF-Luna ----
TFLI2C tfLuna;
const uint8_t TFL_ADDR = 0x10;
int16_t tfDist = 0, tfFlux = 0, tfTemp = 0;

// ---- Timer1 stepping ----
// Prescaler 8 -> 2 MHz timer clock. Compare value = 2e6 / step_rate - 1.
const uint32_t TIMER_HZ = 2000000UL;

volatile uint16_t stepPos  = 0;      // pulse count within the current revolution
volatile uint16_t revCount = 0;      // completed revolutions
volatile uint16_t nextOcr  = 0;      // compare value the ISR loads on its next tick

ISR(TIMER1_COMPA_vect) {
  PORTD |= _BV(PD7);                 // STEP high
  OCR1A = nextOcr;                   // safe to change here: TCNT1 has just reset to 0
  if (++stepPos >= STEPS_PER_REV) {
    stepPos = 0;
    revCount++;
  }
  __builtin_avr_delay_cycles(16);    // ensure >= 1 us pulse width (A4988 min)
  PORTD &= ~_BV(PD7);                // STEP low
}

uint16_t ocrForRate(float stepsPerSec) {
  uint32_t ocr = (uint32_t)(TIMER_HZ / stepsPerSec) - 1;
  if (ocr > 65535) ocr = 65535;
  if (ocr < 100)   ocr = 100;        // hard floor: 20 kHz max step rate
  return (uint16_t)ocr;
}

void setStepRate(float stepsPerSec) {
  uint16_t ocr = ocrForRate(stepsPerSec);
  noInterrupts();                    // 16-bit write must be atomic
  nextOcr = ocr;
  interrupts();
}

void startStepper(float stepsPerSec) {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);   // CTC mode, prescaler 8
  TCNT1  = 0;
  nextOcr = ocrForRate(stepsPerSec);
  OCR1A  = nextOcr;
  TIMSK1 = _BV(OCIE1A);
  interrupts();
}

void readPosition(uint16_t &step, uint16_t &rev) {
  noInterrupts();
  step = stepPos;
  rev  = revCount;
  interrupts();
}

// ---- Main ----
const float    TARGET_RATE    = STEPS_PER_REV / REV_SECONDS;     // pulses per second
const float    START_RATE     = STEPS_PER_REV * START_RPS;
const float    DEG_PER_STEP   = 360.0 / STEPS_PER_REV;
const uint32_t READ_PERIOD_US = 1000000UL / READ_HZ;

uint16_t firstRev  = 0;      // revCount at which logging starts (after ramp-up)
uint16_t lastRev   = 0;
uint32_t nextReadUs = 0;

void setup() {
  Serial.begin(BAUD);

  Wire.begin();
  Wire.setClock(400000);             // TF-Luna supports fast-mode I2C

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, DIR_LEVEL);

  delay(100);
  Serial.println("#LiDAR Scanner Initialized (timer-driven)");
  Serial.println("#Format: step,angle_deg,dist_cm,flux");

  if (tfLuna.getData(tfDist, tfFlux, tfTemp, TFL_ADDR)) {
    Serial.println("#TF Luna connected successfully");
  } else {
    Serial.println("#TF Luna connection failed - check I2C wiring");
  }

  // Not saved to the sensor's flash; it's set again on every boot.
  uint16_t fps = READ_HZ;
  if (tfLuna.Set_Frame_Rate(fps, TFL_ADDR)) {
    Serial.print("#TF Luna frame rate set to ");
    Serial.println(READ_HZ);
  } else {
    Serial.println("#TF Luna frame rate set FAILED");
  }

  Serial.print("#Rev time s: ");
  Serial.print(REV_SECONDS);
  Serial.print("  points/rev: ");
  Serial.println((uint16_t)(REV_SECONDS * READ_HZ));

  delay(500);

  // Soft start: ramp linearly from START_RATE to TARGET_RATE.
  startStepper(START_RATE);
  uint32_t t0 = millis();
  uint32_t elapsed;
  while ((elapsed = millis() - t0) < RAMP_MS) {
    setStepRate(START_RATE + (TARGET_RATE - START_RATE) * elapsed / RAMP_MS);
    delay(5);
  }
  setStepRate(TARGET_RATE);

  // Start logging at the next revolution boundary so scan 0 is a full turn.
  uint16_t s, r;
  readPosition(s, r);
  firstRev = r + 1;
  while (true) {
    readPosition(s, r);
    if (r >= firstRev) break;
  }
  lastRev = r;
  Serial.print("#SCAN ");
  Serial.println(0);

  nextReadUs = micros();
}

void loop() {
  if ((int32_t)(micros() - nextReadUs) < 0) return;
  nextReadUs += READ_PERIOD_US;

  uint16_t step, rev;
  readPosition(step, rev);           // position at the moment of the read

  // A revolution may finish between reads; print its marker first.
  if (rev != lastRev) {
    lastRev = rev;
    Serial.print("#SCAN ");
    Serial.println(rev - firstRev);
  }

  // getData() returns false for weak (<100) or saturated signal too, but it has
  // already filled in dist/flux, so those get logged as-is and Python filters them.
  // A real I2C failure leaves them untouched, so clear first: that logs as 0,0.
  tfDist = 0;
  tfFlux = 0;
  tfLuna.getData(tfDist, tfFlux, tfTemp, TFL_ADDR);

  Serial.print(step);
  Serial.print(',');
  Serial.print(step * DEG_PER_STEP, 2);
  Serial.print(',');
  Serial.print(tfDist);
  Serial.print(',');
  Serial.println((uint16_t)tfFlux);  // flux 65535 = saturated, don't print as -1

  // If we fell far behind (e.g. a long I2C stall), resync instead of bursting.
  if ((int32_t)(micros() - nextReadUs) > (int32_t)(4 * READ_PERIOD_US)) {
    nextReadUs = micros();
  }
}
