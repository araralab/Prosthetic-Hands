//////////////////////////////////////////////////////////////////////////////
// EMG-Triggered Multi-Grasp System for ARARA UNO Prosthetic //
//////////////////////////////////////////////////////////////////////////////
// Combined EMG trigger detection with button & keyboard grasp control
//
// CONTROL FLOW:
// 1. Buttons pre-select a grasp (but don't execute)
// 2. EMG muscle contraction triggers the pre-selected grasp
// 3. Keyboard inputs work as before (immediate execution)
//
// Button Controls:
// Button 1 (Pin 1):  Open Hand (immediate execution)
// Button 2 (Pin 9):  Pre-select Power Grasp
// Button 3 (Pin 13): Pre-select Fine Pinch
// Button 4 (Pin 10): Pre-select Tripod
//
// Keyboard Controls (via Serial) - Immediate execution:
// Key 0: Open Hand (all fingers extended)
// Key 1: Power Grasp (all fingers closed)
// Key 2: Fine Pinch (thumb + middle closed, index + ring+pinky extended)
// Key 3: Tripod (thumb + middle + index closed, ring+pinky extended)
// Key 4: Point (thumb + middle + ring+pinky closed, index extended)
// Key 5: Peace Sign (thumb + ring+pinky closed, index + middle extended)
// Key 6: Close Thumb only
// Key 7: Close Index only
// Key 8: Close Middle only
// Key 9: Close Ring+Pinky only
//
// EMG Sensor: DFRobot Gravity Analog EMG Sensor on pin A5
// - Detects sudden increases in muscle activity
// - Triggers the pre-selected grasp when spike detected
//
// EMG SIGNAL PROCESSING ARCHITECTURE:
// This system uses a three-path processing approach for robust spike detection:
//
// 1. FAST PATH (α=0.25, ~8ms time constant):
//    - Primary signal for spike detection
//    - Tracks muscle contractions with minimal delay (~30ms latency)
//    - Updates continuously (never frozen)
//
// 2. NOISE PATH (Welford's algorithm, ~5s time constant):
//    - Estimates background noise statistics (mean, variance)
//    - Updates only during low-activity periods (velocity-gated)
//    - Used to set adaptive detection thresholds
//
// 3. DC PATH (median filter, ~10s time constant):
//    - Tracks slow baseline drift (temperature, skin contact)
//    - Immune to spikes via median calculation
//    - FROZEN during grasp execution to prevent drift
//
// SPIKE DETECTION:
// - Velocity-based gating: checks rate of signal change
// - CFAR-inspired double threshold (primary + guard)
// - Spike buffer: maintains recent 32 samples for baseline calculation
// - All baselines frozen during graspInProgress flag
//
// BASELINE FREEZE MECHANISM (graspInProgress flag):
// - Critical for preventing drift during motor execution
// - When set to TRUE: DC path and Noise path stop updating
// - Fast path continues (never frozen)
// - Prevents motor noise and hand position changes from corrupting baselines
// - Must be set before ALL motor operations (buttons, keyboard, EMG triggers)
//
// MEMORY USAGE: ~1280 bytes (vs 1440 bytes in old system)
// LATENCY: ~30ms (vs 100ms in old system)
//////////////////////////////////////////////////////////////////////////////
// Written & Maintained by: Rudy de Lange
// Last maintained: 09/01/2026 - Redesigned EMG processing architecture
// Bug fixes applied: 09/01/2026 - Fixed 5 critical bugs
//////////////////////////////////////////////////////////////////////////////

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"

// ========== MOTOR PINS ==========
#define M1_DIR 4     // Index finger
#define E1_PWM 3
#define M2_DIR 7     // Ring + Pinky
#define E2_PWM 6
#define M3_DIR 8     // Thumb
#define E3_PWM 5
#define M4_DIR 12    // Middle finger
#define E4_PWM 11

// ========== BUTTON PINS ==========
#define BUTTON_1 1   // Open Hand (immediate)
#define BUTTON_2 9   // Pre-select Power Grasp
#define BUTTON_3 13  // Pre-select Fine Pinch
#define BUTTON_4 10  // Pre-select Tripod

// ========== EMG SENSOR ==========
#define SensorInputPin A5           // EMG sensor connected to A5

// ========== TIMING CONSTANTS ==========
#define FINGER_EXTEND_TIME 3000
#define FINGER_RETRACT_TIME 3000
#define THUMB_RETRACT_TIME 2300    // 1/2 of full retraction
#define THUMB_DELAY 480            // Delay before thumb starts closing
#define THUMB_PINCH_DELAY 200      // Delay before thumb starts closing for fine pinch
#define DEBOUNCE_DELAY 50          // Button debounce delay in ms

// ========== MOTOR SPEEDS ==========
#define MOTOR_SPEED 255

// ========== EMG CONFIGURATION ==========
#define BAUD_RATE 115200           // IMPROVED: Increased from 9600 to 115200 for faster serial communication
#define UPDATE_INTERVAL 20         // Update serial plotter every 20ms (50 Hz)

// Y-axis scaling for Serial Plotter
#define Y_AXIS_MIN 0
#define Y_AXIS_MAX 70000  // MODIFIED: Increased from 45000 to 70000 for better spike visibility

// EMG Filter setup
EMGFilters myFilter;
int sampleRate = SAMPLE_FREQ_500HZ;  // 500 Hz sampling
int humFreq = NOTCH_FREQ_50HZ;       // 50 Hz notch filter (change to NOTCH_FREQ_60HZ if in US)

// Timing for plotter updates
unsigned long lastPlotTime = 0;

// ========== SIGNAL PROCESSING CONSTANTS ==========
// Fast Path: Primary signal tracking (α=0.25, ~8ms time constant)
#define FAST_EMA_ALPHA 0.25f        // Fast response for spike detection

// Noise Path: Background noise estimation (~5s time constant)
#define NOISE_UPDATE_INTERVAL 250   // Update noise stats every 250 samples (~0.5s)
#define VELOCITY_GATE_THRESHOLD 50.0f // Only update noise when velocity < threshold

// DC Path: Slow baseline drift tracking (~10s time constant)
#define DC_BUFFER_SIZE 16           // Median filter size (keep small for Arduino)
#define DC_UPDATE_INTERVAL 500      // Update DC baseline every 500 samples (~1s)

// Spike Buffer: Recent samples for baseline calculation
#define SPIKE_BUFFER_SIZE 32        // Must be power of 2 for efficient wrapping

// Detection Thresholds
#define PRIMARY_THRESHOLD_K 3.5f    // Primary threshold (K × noise_std)
#define GUARD_THRESHOLD_K 2.5f      // Guard threshold for confirmation
#define MIN_PRIMARY_THRESHOLD 800.0f // Minimum absolute threshold
#define MIN_GUARD_THRESHOLD 500.0f  // Minimum guard threshold

// Timing
#define TRIGGER_COOLDOWN 3000       // 3 second cooldown after grasp execution (ms)
#define CALIBRATION_SAMPLES 5000    // Collect 5000 samples (~10 seconds at 500Hz)

// ========== GLOBAL VARIABLES ==========
// Fast Path
float fastSignal = 0.0f;            // Fast EMA output
float prevFastSignal = 0.0f;        // Previous sample for velocity calculation

// Noise Path (Welford's algorithm for online variance)
float noiseMean = 0.0f;             // Running mean of noise
float noiseM2 = 0.0f;               // Running sum of squared differences
unsigned long noiseSampleCount = 0; // Number of samples in noise estimate
unsigned int noiseSamplesSinceUpdate = 0; // Counter for update interval

// DC Path (median filter for baseline drift)
float dcBuffer[DC_BUFFER_SIZE];     // Circular buffer for median calculation
unsigned int dcBufferIndex = 0;     // Current position in buffer
bool dcBufferFilled = false;        // Has buffer been filled once?
float dcBaseline = 0.0f;            // Current DC baseline estimate
unsigned int dcSamplesSinceUpdate = 0; // Counter for update interval

// Spike Buffer (recent samples for local baseline)
float spikeBuffer[SPIKE_BUFFER_SIZE]; // Circular buffer
unsigned int spikeBufferIndex = 0;  // Current position
bool spikeBufferFilled = false;     // Has buffer been filled once?

// Detection State
unsigned long lastTriggerTime = 0;  // Time of last trigger (for cooldown)
bool triggerEnabled = false;        // Enable after calibration
bool graspInProgress = false;       // Flag to block triggers during grasp execution

// Calibration
unsigned int calibrationCount = 0;  // Number of samples collected
bool calibrationComplete = false;   // Calibration finished flag
unsigned long calibrationStartTime = 0; // Time when calibration started

// ========== STATE TRACKING ==========
enum GraspType {
  OPEN,
  POWER_GRASP,
  FINE_PINCH,
  TRIPOD,
  POINT,
  PEACE_SIGN,
  THUMB_ONLY,
  INDEX_ONLY,
  MIDDLE_ONLY,
  RING_PINKY_ONLY
};

GraspType currentGrasp = OPEN;
GraspType preselectedGrasp = OPEN;  // Grasp pre-selected by buttons
bool graspPreselected = false;       // Flag to indicate a grasp is waiting for trigger

// Button state tracking
int lastBtn1 = HIGH;
int lastBtn2 = HIGH;
int lastBtn3 = HIGH;
int lastBtn4 = HIGH;

void setup() {
  // Initialize EMG filter
  myFilter.init(sampleRate, humFreq, true, true, true);

  // Initialize serial
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for R4)
  }

  Serial.println(F("========================================"));
  Serial.println(F("  EMG-Triggered Grasp Control System"));
  Serial.println(F("========================================"));
  Serial.println(F("--- Button Controls ---"));
  Serial.println(F("Button 1 (Pin 1):  Open Hand (immediate)"));
  Serial.println(F("Button 2 (Pin 9):  Pre-select Power Grasp"));
  Serial.println(F("Button 3 (Pin 13): Pre-select Fine Pinch"));
  Serial.println(F("Button 4 (Pin 10): Pre-select Tripod"));
  Serial.println(F("--- Keyboard Controls (immediate) ---"));
  Serial.println(F("0: Open Hand"));
  Serial.println(F("1: Power Grasp"));
  Serial.println(F("2: Fine Pinch"));
  Serial.println(F("3: Tripod"));
  Serial.println(F("4: Point"));
  Serial.println(F("5: Peace Sign"));
  Serial.println(F("6: Close Thumb only"));
  Serial.println(F("7: Close Index only"));
  Serial.println(F("8: Close Middle only"));
  Serial.println(F("9: Close Ring+Pinky only"));
  Serial.println(F("========================================"));

  // Initialize motor pins
  pinMode(M1_DIR, OUTPUT);
  pinMode(E1_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(E2_PWM, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(E3_PWM, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(E4_PWM, OUTPUT);

  // Initialize button pins with internal pull-up resistors
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);

  // Stop all motors
  stopAllMotors();

  delay(1000);

  // Initialize to open position
  Serial.println(F("Initializing hand to open position..."));
  openAllFingers();
  Serial.println(F("Hand initialized!"));
  Serial.println();

  // Initialize EMG buffers
  for (int i = 0; i < DC_BUFFER_SIZE; i++) {
    dcBuffer[i] = 0.0f;
  }
  for (int i = 0; i < SPIKE_BUFFER_SIZE; i++) {
    spikeBuffer[i] = 0.0f;
  }

  // Start EMG calibration
  Serial.println(F("EMG CALIBRATION:"));
  Serial.println(F("Keep your muscle RELAXED for 10 seconds..."));
  Serial.println(F("========================================"));

  // Record calibration start time
  calibrationStartTime = millis();
}

void loop() {
  // ========== EMG PROCESSING (runs at 500 Hz) ==========
  processEMG();

  // ========== BUTTON HANDLING ==========
  checkButtons();

  // ========== KEYBOARD HANDLING ==========
  if (Serial.available() > 0) {
    char key = Serial.read();
    handleKeyboardInput(key);
  }

  // No delay - run at maximum speed for faster EMG processing
}

// ========== HELPER FUNCTIONS FOR SIGNAL PROCESSING ==========

// Calculate median of DC buffer using efficient partial sort
float computeMedian(float* buffer, int size) {
  // Create temporary array for sorting (avoid modifying original)
  float temp[DC_BUFFER_SIZE];
  for (int i = 0; i < size; i++) {
    temp[i] = buffer[i];
  }

  // Simple selection sort for small arrays (efficient for DC_BUFFER_SIZE=16)
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[j] < temp[i]) {
        float swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }

  // Return median
  if (size % 2 == 0) {
    return (temp[size/2 - 1] + temp[size/2]) / 2.0f;
  } else {
    return temp[size/2];
  }
}

// Calculate mean of spike buffer for local baseline
float getSpikeBaseline() {
  if (!spikeBufferFilled && spikeBufferIndex == 0) {
    return 0.0f;  // No data yet
  }

  int count = spikeBufferFilled ? SPIKE_BUFFER_SIZE : spikeBufferIndex;
  float sum = 0.0f;
  for (int i = 0; i < count; i++) {
    sum += spikeBuffer[i];
  }
  return sum / count;
}

// Calculate velocity (rate of change) of fast signal
float getVelocity() {
  return abs(fastSignal - prevFastSignal);
}

// Update Fast Path: Fast EMA for primary signal tracking
void updateFastPath(float envelope) {
  // Initialize on first sample
  if (fastSignal == 0.0f && prevFastSignal == 0.0f) {
    fastSignal = envelope;
    prevFastSignal = envelope;
    return;
  }

  // Store previous for velocity calculation
  prevFastSignal = fastSignal;

  // Apply fast EMA: y[n] = α×x[n] + (1-α)×y[n-1]
  fastSignal = (FAST_EMA_ALPHA * envelope) + ((1.0f - FAST_EMA_ALPHA) * fastSignal);
}

// Update Noise Path: Welford's algorithm for online variance (velocity-gated)
void updateNoisePath(float dcCorrectedSignal) {
  noiseSamplesSinceUpdate++;

  // Only update periodically to save computation
  if (noiseSamplesSinceUpdate < NOISE_UPDATE_INTERVAL) {
    return;
  }
  noiseSamplesSinceUpdate = 0;

  // Only update if velocity is low (signal is stable)
  float velocity = getVelocity();
  if (velocity > VELOCITY_GATE_THRESHOLD) {
    return;  // Skip update during rapid changes
  }

  // Only update during grasp rest (not during execution)
  if (graspInProgress) {
    return;  // Skip update during grasp
  }

  // Welford's online algorithm for mean and variance
  // FIX: Initialize on first sample to avoid division by zero
  if (noiseSampleCount == 0) {
    noiseSampleCount = 1;
    noiseMean = dcCorrectedSignal;
    noiseM2 = 0.0f;
    return;
  }

  noiseSampleCount++;
  float delta = dcCorrectedSignal - noiseMean;
  noiseMean += delta / noiseSampleCount;
  float delta2 = dcCorrectedSignal - noiseMean;
  noiseM2 += delta * delta2;

  // Limit sample count to prevent overflow and allow adaptation
  if (noiseSampleCount > 1000) {
    // Reset with current statistics as initial values
    float currentVariance = noiseM2 / noiseSampleCount;
    noiseSampleCount = 100;
    noiseM2 = currentVariance * noiseSampleCount;
  }
}

// Update DC Path: Median filter for slow baseline drift (FROZEN during grasp)
void updateDCPath(float fastSignalValue) {
  dcSamplesSinceUpdate++;

  // Only update periodically
  if (dcSamplesSinceUpdate < DC_UPDATE_INTERVAL) {
    return;
  }
  dcSamplesSinceUpdate = 0;

  // FREEZE during grasp execution to prevent drift
  if (graspInProgress) {
    return;
  }

  // Add current value to buffer
  dcBuffer[dcBufferIndex] = fastSignalValue;
  dcBufferIndex++;

  if (dcBufferIndex >= DC_BUFFER_SIZE) {
    dcBufferIndex = 0;
    dcBufferFilled = true;
  }

  // Calculate median (immune to spikes)
  int validSamples = dcBufferFilled ? DC_BUFFER_SIZE : dcBufferIndex;
  if (validSamples > 0) {
    dcBaseline = computeMedian(dcBuffer, validSamples);
  }
}

// Update Spike Buffer: Maintain recent samples for local baseline
void updateSpikeBuffer(float dcCorrectedSignal) {
  // Always update spike buffer (never frozen)
  spikeBuffer[spikeBufferIndex] = dcCorrectedSignal;
  spikeBufferIndex = (spikeBufferIndex + 1) & (SPIKE_BUFFER_SIZE - 1); // Efficient modulo for power of 2

  if (spikeBufferIndex == 0) {
    spikeBufferFilled = true;
  }
}

// Detect spike using CFAR-inspired double threshold
bool detectSpike(float dcCorrectedSignal) {
  // Get local baseline from spike buffer
  float spikeBaseline = getSpikeBaseline();

  // Calculate deviation from local baseline
  float deviation = dcCorrectedSignal - spikeBaseline;
  if (deviation < 0) {
    deviation = 0;  // Clamp to zero
  }

  // Calculate adaptive thresholds from noise statistics
  float noiseStd = 0.0f;
  if (noiseSampleCount > 1) {
    float variance = noiseM2 / noiseSampleCount;
    // FIX: Clamp variance to 0 to prevent NaN from sqrt() due to floating point errors
    if (variance < 0.0f) {
      variance = 0.0f;
    }
    noiseStd = sqrt(variance);
  }

  // Primary threshold: K × noise_std, with minimum floor
  float primaryThreshold = max(PRIMARY_THRESHOLD_K * noiseStd, MIN_PRIMARY_THRESHOLD);

  // Guard threshold: Lower threshold for confirmation
  float guardThreshold = max(GUARD_THRESHOLD_K * noiseStd, MIN_GUARD_THRESHOLD);

  // Check double threshold
  if (deviation > primaryThreshold) {
    // Primary threshold exceeded - check velocity for confirmation
    float velocity = getVelocity();
    if (velocity > guardThreshold) {
      return true;  // SPIKE DETECTED!
    }
  }

  return false;
}

// Reset spike detector (called when needed)
void resetSpikeDetector() {
  // Clear spike buffer
  for (int i = 0; i < SPIKE_BUFFER_SIZE; i++) {
    spikeBuffer[i] = 0.0f;
  }
  spikeBufferIndex = 0;
  spikeBufferFilled = false;
}

// ========== MAIN EMG PROCESSING FUNCTION ==========
void processEMG() {
  // Step 1: Read and filter raw EMG signal
  int rawValue = analogRead(SensorInputPin);
  int filteredValue = myFilter.update(rawValue);
  float envelope = (float)sq(filteredValue);

  // ============================================================
  // CALIBRATION PHASE: Initialize all processing paths
  // ============================================================
  if (!calibrationComplete) {
    // Update fast path
    updateFastPath(envelope);

    // Initialize DC baseline
    if (calibrationCount == 0) {
      dcBaseline = fastSignal;
    }

    // Update DC path during calibration
    updateDCPath(fastSignal);

    calibrationCount++;

    // Print countdown every second (every 500 samples at 500Hz)
    if (calibrationCount % 500 == 0) {
      unsigned long elapsedSeconds = (millis() - calibrationStartTime) / 1000;
      unsigned long remainingSeconds = 10 - elapsedSeconds;

      Serial.print(F("Calibrating... "));
      Serial.print(remainingSeconds);
      Serial.println(F(" seconds remaining"));
    }

    // Calibration complete
    if (calibrationCount >= CALIBRATION_SAMPLES) {
      calibrationComplete = true;
      triggerEnabled = true;

      // Initialize noise statistics with calibration data
      float dcCorrected = fastSignal - dcBaseline;
      if (dcCorrected < 0) dcCorrected = 0;
      noiseMean = dcCorrected;
      noiseSampleCount = 1;
      noiseM2 = 0.0f;

      // FIX: Initialize spike buffer with current baseline after calibration
      // This prevents incorrect baseline initially
      for (int i = 0; i < SPIKE_BUFFER_SIZE; i++) {
        spikeBuffer[i] = dcCorrected;
      }
      spikeBufferIndex = 0;
      spikeBufferFilled = true;

      Serial.println();
      Serial.println(F("========================================"));
      Serial.println(F("Calibration complete!"));
      Serial.print(F("DC Baseline: "));
      Serial.println((unsigned long)dcBaseline);
      Serial.println(F("Ready to detect muscle contractions!"));
      Serial.println(F("========================================"));
      Serial.println();

      delay(1000);
    }

    return;  // Skip normal processing during calibration
  }

  // ============================================================
  // NORMAL OPERATION: Three-path signal processing
  // ============================================================

  // Step 2: Update Fast Path (always updates, never frozen)
  updateFastPath(envelope);

  // Step 3: DC correction (subtract slow baseline drift)
  float dcCorrectedSignal = fastSignal - dcBaseline;
  if (dcCorrectedSignal < 0) {
    dcCorrectedSignal = 0;  // Clamp to zero
  }

  // Step 4: Update Noise Path (velocity-gated, frozen during grasp)
  updateNoisePath(dcCorrectedSignal);

  // Step 5: Update DC Path (frozen during grasp)
  updateDCPath(fastSignal);

  // Step 6: Update Spike Buffer (always updates)
  updateSpikeBuffer(dcCorrectedSignal);

  // Step 7: Spike Detection
  unsigned long currentTime = millis();
  bool inCooldown = (currentTime - lastTriggerTime) < TRIGGER_COOLDOWN;

  // Only detect spikes when conditions are met
  if (triggerEnabled && !graspInProgress && !inCooldown) {
    bool spikeDetected = detectSpike(dcCorrectedSignal);

    // Debug: If spike detected but no grasp pre-selected
    if (spikeDetected && !graspPreselected) {
      Serial.println(F("Spike detected but no grasp pre-selected! Press button 2/3/4 first."));
    }

    // Execute grasp if spike detected and grasp is pre-selected
    if (spikeDetected && graspPreselected) {
      // SPIKE DETECTED!
      float spikeBaseline = getSpikeBaseline();
      float deviation = dcCorrectedSignal - spikeBaseline;

      Serial.println();
      Serial.println(F("*** EMG SPIKE DETECTED! ***"));
      Serial.print(F("Deviation: "));
      Serial.print((int)deviation);
      Serial.print(F(" | Signal: "));
      Serial.print((int)dcCorrectedSignal);
      Serial.print(F(" | Baseline: "));
      Serial.println((int)spikeBaseline);

      // Set flag to block updates during grasp
      graspInProgress = true;

      // TOGGLE behavior: if open, execute grasp; if closed, open
      if (currentGrasp == OPEN) {
        Serial.print(F("Executing pre-selected grasp: "));
        executeGrasp(preselectedGrasp);
        Serial.println(F("Grasp completed!"));
      } else {
        Serial.println(F("Opening hand..."));
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> OPEN"));
      }

      // Clear flag after grasp completes
      graspInProgress = false;

      // Update cooldown timer
      lastTriggerTime = millis();

      Serial.println(F("Cooldown active for 3 seconds."));
      Serial.println();
    }
  }

  // ============================================================
  // SERIAL PLOTTER OUTPUT: Update at 50 Hz
  // ============================================================
  if (currentTime - lastPlotTime >= UPDATE_INTERVAL) {
    float spikeBaseline = getSpikeBaseline();
    float noiseStd = 0.0f;
    if (noiseSampleCount > 1) {
      float variance = noiseM2 / noiseSampleCount;
      // FIX: Clamp variance to 0 to prevent NaN from sqrt() due to floating point errors
      if (variance < 0.0f) {
        variance = 0.0f;
      }
      noiseStd = sqrt(variance);
    }
    float primaryThreshold = max(PRIMARY_THRESHOLD_K * noiseStd, MIN_PRIMARY_THRESHOLD);

    // Print for Serial Plotter
    Serial.print("Min:");
    Serial.print(Y_AXIS_MIN);
    Serial.print(",");
    Serial.print("EMG:");
    Serial.print((unsigned long)dcCorrectedSignal);
    Serial.print(",");
    Serial.print("SpikeBaseline:");
    Serial.print((unsigned long)spikeBaseline);
    Serial.print(",");
    Serial.print("Threshold:");
    Serial.print((unsigned long)(spikeBaseline + primaryThreshold));
    Serial.print(",");
    Serial.print("Max:");
    Serial.println(Y_AXIS_MAX);

    lastPlotTime = currentTime;
  }

  // Maintain 500 Hz sample rate
  delayMicroseconds(1500);
}

// ========== BUTTON HANDLING ==========
void checkButtons() {
  // Read button states
  int btn1 = digitalRead(BUTTON_1);
  int btn2 = digitalRead(BUTTON_2);
  int btn3 = digitalRead(BUTTON_3);
  int btn4 = digitalRead(BUTTON_4);

  // Button 1 - Open Hand (IMMEDIATE execution)
  if (btn1 != lastBtn1) {
    // FIX: Buttons use INPUT_PULLUP, so LOW = pressed, HIGH = released
    if (btn1 == LOW) {  // Button pressed (was HIGH - now corrected)
      Serial.println(F("Button 1 pressed: Opening Hand"));
      graspInProgress = true;  // Block baseline updates during execution
      openAllFingers();
      currentGrasp = OPEN;
      graspPreselected = false;  // Clear any preselection
      graspInProgress = false;  // Re-enable baseline updates
      Serial.println(F(">>> OPEN"));
    }
    lastBtn1 = btn1;
    delay(DEBOUNCE_DELAY);
  }

  // Button 2 - PRE-SELECT Power Grasp
  if (btn2 != lastBtn2) {
    // FIX: Buttons use INPUT_PULLUP, so LOW = pressed, HIGH = released
    if (btn2 == LOW) {  // Button pressed (was HIGH - now corrected)
      if (!triggerEnabled) {
        Serial.println(F("ERROR: System still calibrating, please wait"));
      } else if (currentGrasp == OPEN) {
        preselectedGrasp = POWER_GRASP;
        graspPreselected = true;
        Serial.println(F("Button 2 pressed: Power Grasp PRE-SELECTED"));
        Serial.println(F(">>> Waiting for EMG trigger..."));
      } else {
        Serial.println(F("ERROR: Release hand first (press Button 1)"));
      }
    }
    lastBtn2 = btn2;
    delay(DEBOUNCE_DELAY);
  }

  // Button 3 - PRE-SELECT Fine Pinch
  if (btn3 != lastBtn3) {
    // FIX: Buttons use INPUT_PULLUP, so LOW = pressed, HIGH = released
    if (btn3 == LOW) {  // Button pressed (was HIGH - now corrected)
      if (!triggerEnabled) {
        Serial.println(F("ERROR: System still calibrating, please wait"));
      } else if (currentGrasp == OPEN) {
        preselectedGrasp = FINE_PINCH;
        graspPreselected = true;
        Serial.println(F("Button 3 pressed: Fine Pinch PRE-SELECTED"));
        Serial.println(F(">>> Waiting for EMG trigger..."));
      } else {
        Serial.println(F("ERROR: Release hand first (press Button 1)"));
      }
    }
    lastBtn3 = btn3;
    delay(DEBOUNCE_DELAY);
  }

  // Button 4 - PRE-SELECT Tripod
  if (btn4 != lastBtn4) {
    // FIX: Buttons use INPUT_PULLUP, so LOW = pressed, HIGH = released
    if (btn4 == LOW) {  // Button pressed (was HIGH - now corrected)
      if (!triggerEnabled) {
        Serial.println(F("ERROR: System still calibrating, please wait"));
      } else if (currentGrasp == OPEN) {
        preselectedGrasp = TRIPOD;
        graspPreselected = true;
        Serial.println(F("Button 4 pressed: Tripod PRE-SELECTED"));
        Serial.println(F(">>> Waiting for EMG trigger..."));
      } else {
        Serial.println(F("ERROR: Release hand first (press Button 1)"));
      }
    }
    lastBtn4 = btn4;
    delay(DEBOUNCE_DELAY);
  }
}

// ========== KEYBOARD HANDLING (IMMEDIATE EXECUTION) ==========
void handleKeyboardInput(char key) {
  switch (key) {
    case '0':
      openAllFingers();
      currentGrasp = OPEN;
      graspPreselected = false;  // Clear any preselection
      Serial.println(F(">>> OPEN"));
      break;

    case '1':
      if (currentGrasp == POWER_GRASP) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performPowerGrasp();
        currentGrasp = POWER_GRASP;
        graspInProgress = false;
        Serial.println(F(">>> POWER GRASP"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '2':
      if (currentGrasp == FINE_PINCH) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performFinePinch();
        currentGrasp = FINE_PINCH;
        graspInProgress = false;
        Serial.println(F(">>> FINE PINCH"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '3':
      if (currentGrasp == TRIPOD) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performTripod();
        currentGrasp = TRIPOD;
        graspInProgress = false;
        Serial.println(F(">>> TRIPOD"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '4':
      if (currentGrasp == POINT) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performPoint();
        currentGrasp = POINT;
        graspInProgress = false;
        Serial.println(F(">>> POINT"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '5':
      if (currentGrasp == PEACE_SIGN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> PEACE SIGN"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performPeaceSign();
        currentGrasp = PEACE_SIGN;
        graspInProgress = false;
        Serial.println(F(">>> PEACE SIGN"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '6':
      if (currentGrasp == THUMB_ONLY) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> THUMB ONLY"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performThumbOnly();
        currentGrasp = THUMB_ONLY;
        graspInProgress = false;
        Serial.println(F(">>> THUMB ONLY"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '7':
      if (currentGrasp == INDEX_ONLY) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> INDEX ONLY"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performIndexOnly();
        currentGrasp = INDEX_ONLY;
        graspInProgress = false;
        Serial.println(F(">>> INDEX ONLY"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '8':
      if (currentGrasp == MIDDLE_ONLY) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> MIDDLE ONLY"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performMiddleOnly();
        currentGrasp = MIDDLE_ONLY;
        graspInProgress = false;
        Serial.println(F(">>> MIDDLE ONLY"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '9':
      if (currentGrasp == RING_PINKY_ONLY) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        openAllFingers();
        currentGrasp = OPEN;
        graspInProgress = false;
        Serial.println(F(">>> RING+PINKY ONLY"));
      } else if (currentGrasp == OPEN) {
        // FIX: Set flag to freeze baseline during motor execution
        graspInProgress = true;
        performRingPinkyOnly();
        currentGrasp = RING_PINKY_ONLY;
        graspInProgress = false;
        Serial.println(F(">>> RING+PINKY ONLY"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;
  }
}

// ========== EXECUTE GRASP (Called by EMG trigger) ==========
void executeGrasp(GraspType grasp) {
  switch (grasp) {
    case POWER_GRASP:
      Serial.println(F("POWER GRASP"));
      performPowerGrasp();
      currentGrasp = POWER_GRASP;
      break;

    case FINE_PINCH:
      Serial.println(F("FINE PINCH"));
      performFinePinch();
      currentGrasp = FINE_PINCH;
      break;

    case TRIPOD:
      Serial.println(F("TRIPOD"));
      performTripod();
      currentGrasp = TRIPOD;
      break;

    default:
      Serial.println(F("ERROR: Invalid grasp type"));
      break;
  }
}

// ========== HELPER FUNCTIONS ==========

// Simple blocking delay - no EMG processing or plotting during motor movements
// This completely prevents baseline drift during grasps
void delayWithEMG(unsigned long milliseconds) {
  delay(milliseconds);  // Pure blocking delay - no EMG updates, no drift
}

void openAllFingers() {
  // Set all to extend
  digitalWrite(M1_DIR, HIGH);   // Index extend
  digitalWrite(M2_DIR, LOW);    // Ring+Pinky extend
  digitalWrite(M3_DIR, HIGH);   // Thumb extend
  digitalWrite(M4_DIR, LOW);    // Middle extend

  // Start all motors
  analogWrite(E1_PWM, MOTOR_SPEED);
  analogWrite(E2_PWM, MOTOR_SPEED);
  analogWrite(E3_PWM, MOTOR_SPEED);
  analogWrite(E4_PWM, MOTOR_SPEED);

  delayWithEMG(FINGER_EXTEND_TIME);  // Non-blocking delay with EMG processing

  // Stop all motors
  stopAllMotors();
}

void stopAllMotors() {
  analogWrite(E1_PWM, 0);
  analogWrite(E2_PWM, 0);
  analogWrite(E3_PWM, 0);
  analogWrite(E4_PWM, 0);
}

// ========== GRASP FUNCTIONS ==========

void performPowerGrasp() {
  // Close: Index, Ring+Pinky, Middle, Thumb (delayed)
  // Set directions
  digitalWrite(M1_DIR, LOW);    // Index retract
  digitalWrite(M2_DIR, HIGH);   // Ring+Pinky retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  digitalWrite(M4_DIR, HIGH);   // Middle retract

  // Start fingers first (not thumb)
  analogWrite(E1_PWM, MOTOR_SPEED);
  analogWrite(E2_PWM, MOTOR_SPEED);
  analogWrite(E4_PWM, MOTOR_SPEED);

  delayWithEMG(THUMB_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delayWithEMG(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for other fingers to finish
  delayWithEMG(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME - THUMB_DELAY);

  stopAllMotors();
}

void performFinePinch() {
  // Retract Middle and thumb
  digitalWrite(M4_DIR, HIGH);    // Middle retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract

  analogWrite(E4_PWM, MOTOR_SPEED);

  delayWithEMG(THUMB_PINCH_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delayWithEMG(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for Middle to finish
  delayWithEMG(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME);

  stopAllMotors();
}

void performTripod() {
  // Retract index and middle first
  digitalWrite(M1_DIR, LOW);    // Index retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  digitalWrite(M4_DIR, HIGH);   // Middle retract

  analogWrite(E1_PWM, MOTOR_SPEED);
  analogWrite(E4_PWM, MOTOR_SPEED);

  delayWithEMG(THUMB_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delayWithEMG(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for other fingers to finish
  delayWithEMG(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME - THUMB_DELAY);

  stopAllMotors();
}

void performPoint() {
  // Retract ring+pinky and middle first
  digitalWrite(M2_DIR, HIGH);   // Ring+Pinky retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  digitalWrite(M4_DIR, HIGH);   // Middle retract

  analogWrite(E2_PWM, MOTOR_SPEED);
  analogWrite(E4_PWM, MOTOR_SPEED);

  delayWithEMG(THUMB_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delayWithEMG(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for other fingers to finish
  delayWithEMG(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME - THUMB_DELAY);

  stopAllMotors();
}

void performPeaceSign() {
  // Retract ring+pinky and thumb together
  digitalWrite(M2_DIR, HIGH);   // Ring+Pinky retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract

  analogWrite(E2_PWM, MOTOR_SPEED);  // Ring + Pinky
  analogWrite(E3_PWM, MOTOR_SPEED);  // Thumb

  delayWithEMG(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for ring+pinky to finish
  delayWithEMG(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME);

  stopAllMotors();
}

void performThumbOnly() {
  // Retract thumb only
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  analogWrite(E3_PWM, MOTOR_SPEED);

  delayWithEMG(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  stopAllMotors();
}

void performIndexOnly() {
  // Retract index only
  digitalWrite(M1_DIR, LOW);    // Index retract
  analogWrite(E1_PWM, MOTOR_SPEED);   // Index Finger

  delayWithEMG(FINGER_RETRACT_TIME);
  stopAllMotors();
}

void performMiddleOnly() {
  // Retract middle only
  digitalWrite(M4_DIR, HIGH);   // Middle retract
  analogWrite(E4_PWM, MOTOR_SPEED);  // Middle Finger

  delayWithEMG(FINGER_RETRACT_TIME);
  stopAllMotors();
}

void performRingPinkyOnly() {
  // Retract ring+pinky only
  digitalWrite(M2_DIR, HIGH);   // Ring+Pinky retract
  analogWrite(E2_PWM, MOTOR_SPEED); // Ring+Pinky

  delayWithEMG(FINGER_RETRACT_TIME);
  stopAllMotors();
}
