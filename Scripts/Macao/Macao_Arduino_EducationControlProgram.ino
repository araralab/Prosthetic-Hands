//////////////////////////////////////////////////////////////////////////////
// Button & Keyboard-Controlled Multi-Grasp System for ARARA UNO Prosthetic //
//////////////////////////////////////////////////////////////////////////////
// Button 1 (Pin 2):  Open Hand
// Button 2 (Pin 9):  Power Grasp (all fingers closed)
// Button 3 (Pin 10): Fine Pinch (thumb + middle closed)
// Button 4 (Pin 13): Tripod (thumb + middle + index closed)
//
// Keyboard Controls (via Serial):
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
//////////////////////////////////////////////////////////////////////////////
// Written & Maintained by: Rudy de Lange (3063568D@student.gla.ac.uk)
// Last maintained: 13/01/2026 - Added button control + Fixed Logic
//////////////////////////////////////////////////////////////////////////////

// Motor pins
#define M1_DIR 7     // Middle finger (physically wired)
#define E1_PWM 6
#define M2_DIR 4     // Index Finger
#define E2_PWM 3
#define M3_DIR 8     // Thumb
#define E3_PWM 5
#define M4_DIR 12    // Ring + Pinky (physically wired)
#define E4_PWM 11

// Button pins
#define BUTTON_1 1   // Open Hand
#define BUTTON_2 9   // Power Grasp
#define BUTTON_3 13  // Fine Pinch
#define BUTTON_4 10  // Tripod

// Timing constants
#define FINGER_EXTEND_TIME 3000
#define FINGER_RETRACT_TIME 3000
#define THUMB_RETRACT_TIME 3000    
#define THUMB_DELAY 480            // Delay before thumb starts closing
#define THUMB_PINCH_DELAY 200      // Delay before thumb starts closing for fine pinch
#define DEBOUNCE_DELAY 50          // Button debounce delay in ms

// Motor speeds
#define MOTOR_SPEED 255

// State tracking
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

// Button state tracking
int lastBtn1 = HIGH;
int lastBtn2 = HIGH;
int lastBtn3 = HIGH;
int lastBtn4 = HIGH;

void setup() {
  Serial.begin(9600);
  Serial.println(F("=== Multi-Grasp Control System ==="));
  Serial.println(F("--- Button Controls ---"));
  Serial.println(F("Button 1 (Pin 2):  Open Hand"));
  Serial.println(F("Button 2 (Pin 9):  Power Grasp"));
  Serial.println(F("Button 3 (Pin 10): Fine Pinch"));
  Serial.println(F("Button 4 (Pin 13): Tripod"));
  Serial.println(F("--- Keyboard Controls ---"));
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
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);
  digitalWrite(M4_DIR, LOW);
  analogWrite(E1_PWM, 0);
  analogWrite(E2_PWM, 0);
  analogWrite(E3_PWM, 0);
  analogWrite(E4_PWM, 0);

  delay(1000);

  // Initialize to open position
  Serial.println(F("Initializing to open position..."));
  openAllFingers();

  Serial.println(F("Ready!"));
}

void loop() {
  // Check button inputs
  checkButtons();

  // Check serial keyboard inputs
  if (Serial.available() > 0) {
    char key = Serial.read();
    handleKeyboardInput(key);
  }

  delay(10);
}

// ========== BUTTON HANDLING ==========

void checkButtons() {
  // Read button states
  int btn1 = digitalRead(BUTTON_1);
  int btn2 = digitalRead(BUTTON_2);
  int btn3 = digitalRead(BUTTON_3);
  int btn4 = digitalRead(BUTTON_4);

  // Button 1 - Open Hand (trigger on press: LOW->HIGH transition)
  if (btn1 != lastBtn1) {
    if (btn1 == HIGH) {  // Button pressed (HIGH = pressed)
      Serial.println(F("Button 1 pressed: Open Hand"));
      openAllFingers();
      currentGrasp = OPEN;
      Serial.println(F(">>> OPEN"));
    }
    lastBtn1 = btn1;
    delay(50);  // Debounce
  }

  // Button 2 - Power Grasp
  if (btn2 != lastBtn2) {
    if (btn2 == HIGH) {  // Button pressed
      Serial.println(F("Button 2 pressed: Power Grasp"));
      if (currentGrasp == POWER_GRASP) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        performPowerGrasp();
        currentGrasp = POWER_GRASP;
        Serial.println(F(">>> POWER GRASP"));
      } else {
        Serial.println(F("ERROR: Release hand first (press Button 1)"));
      }
    }
    lastBtn2 = btn2;
    delay(50);
  }

  // Button 3 - Fine Pinch
  if (btn3 != lastBtn3) {
    if (btn3 == HIGH) {  // Button pressed
      Serial.println(F("Button 3 pressed: Fine Pinch"));
      if (currentGrasp == FINE_PINCH) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        performFinePinch();
        currentGrasp = FINE_PINCH;
        Serial.println(F(">>> FINE PINCH"));
      } else {
        Serial.println(F("ERROR: Release hand first (press Button 1)"));
      }
    }
    lastBtn3 = btn3;
    delay(50);
  }

  // Button 4 - Tripod
  if (btn4 != lastBtn4) {
    if (btn4 == HIGH) {  // Button pressed
      Serial.println(F("Button 4 pressed: Tripod"));
      if (currentGrasp == TRIPOD) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        performTripod();
        currentGrasp = TRIPOD;
        Serial.println(F(">>> TRIPOD"));
      } else {
        Serial.println(F("ERROR: Release hand first (press Button 1)"));
      }
    }
    lastBtn4 = btn4;
    delay(50);
  }
}

// ========== KEYBOARD HANDLING ==========

void handleKeyboardInput(char key) {
  switch (key) {
    case '0':
      openAllFingers();
      currentGrasp = OPEN;
      Serial.println(F(">>> OPEN"));
      break;

    case '1':
      if (currentGrasp == POWER_GRASP) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        performPowerGrasp();
        currentGrasp = POWER_GRASP;
        Serial.println(F(">>> POWER GRASP"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '2':
      if (currentGrasp == FINE_PINCH) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        performFinePinch();
        currentGrasp = FINE_PINCH;
        Serial.println(F(">>> FINE PINCH"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '3':
      if (currentGrasp == TRIPOD) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        performTripod();
        currentGrasp = TRIPOD;
        Serial.println(F(">>> TRIPOD"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '4':
      if (currentGrasp == POINT) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> OPEN"));
      } else if (currentGrasp == OPEN) {
        performPoint();
        currentGrasp = POINT;
        Serial.println(F(">>> POINT"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '5':
      if (currentGrasp == PEACE_SIGN) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> PEACE SIGN"));
      } else if (currentGrasp == OPEN) {
        performPeaceSign();
        currentGrasp = PEACE_SIGN;
        Serial.println(F(">>> PEACE SIGN"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '6':
      if (currentGrasp == THUMB_ONLY) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> THUMB ONLY"));
      } else if (currentGrasp == OPEN) {
        performThumbOnly();
        currentGrasp = THUMB_ONLY;
        Serial.println(F(">>> THUMB ONLY"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '7':
      if (currentGrasp == INDEX_ONLY) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> INDEX ONLY"));
      } else if (currentGrasp == OPEN) {
        performIndexOnly();
        currentGrasp = INDEX_ONLY;
        Serial.println(F(">>> INDEX ONLY"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '8':
      if (currentGrasp == MIDDLE_ONLY) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> MIDDLE ONLY"));
      } else if (currentGrasp == OPEN) {
        performMiddleOnly();
        currentGrasp = MIDDLE_ONLY;
        Serial.println(F(">>> MIDDLE ONLY"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;

    case '9':
      if (currentGrasp == RING_PINKY_ONLY) {
        openAllFingers();
        currentGrasp = OPEN;
        Serial.println(F(">>> RING+PINKY ONLY"));
      } else if (currentGrasp == OPEN) {
        performRingPinkyOnly();
        currentGrasp = RING_PINKY_ONLY;
        Serial.println(F(">>> RING+PINKY ONLY"));
      } else {
        Serial.println(F("ERROR: Release hand first (press 0 or Button 1)"));
      }
      break;
  }
}

// ========== HELPER FUNCTIONS ==========

void openAllFingers() {
  Serial.println(F("Opening all fingers..."));

  // Set all to extend
  digitalWrite(M1_DIR, LOW);    // Middle extend 
  digitalWrite(M2_DIR, HIGH);   // Index extend
  digitalWrite(M3_DIR, HIGH);   // Thumb extend
  digitalWrite(M4_DIR, LOW);    // Ring+Pinky extend 

  // Start all motors
  analogWrite(E1_PWM, MOTOR_SPEED);
  analogWrite(E2_PWM, MOTOR_SPEED);
  analogWrite(E3_PWM, MOTOR_SPEED);
  analogWrite(E4_PWM, MOTOR_SPEED);

  delay(FINGER_EXTEND_TIME);

  // Stop all motors
  analogWrite(E1_PWM, 0);
  analogWrite(E2_PWM, 0);
  analogWrite(E3_PWM, 0);
  analogWrite(E4_PWM, 0);
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
  Serial.println(F("Power Grasp..."));

  // Set directions
  digitalWrite(M1_DIR, HIGH);   // Middle retract 
  digitalWrite(M2_DIR, LOW);    // Index retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  digitalWrite(M4_DIR, HIGH);   // Ring+Pinky retract 

  // Start fingers first (not thumb)
  analogWrite(E2_PWM, MOTOR_SPEED);  // Index
  analogWrite(E1_PWM, MOTOR_SPEED);  // Middle 
  analogWrite(E4_PWM, MOTOR_SPEED);  // Ring+Pinky 

  delay(THUMB_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delay(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for other fingers to finish
  delay(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME - THUMB_DELAY);

  stopAllMotors();
}

void performFinePinch() {
  Serial.println(F("Fine Pinch..."));

  // Retract index and thumb only
  digitalWrite(M2_DIR, LOW);    // Index retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract

  analogWrite(E2_PWM, MOTOR_SPEED);  // Index

  delay(THUMB_PINCH_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delay(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for Index to finish
  delay(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME);

  stopAllMotors();
}

void performTripod() {
  Serial.println(F("Tripod..."));

  // Retract index, middle, and thumb (keep ring+pinky extended)
  digitalWrite(M2_DIR, LOW);    // Index retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  digitalWrite(M1_DIR, HIGH);   // Middle retract

  analogWrite(E2_PWM, MOTOR_SPEED);  // Index
  analogWrite(E1_PWM, MOTOR_SPEED);  // Middle

  delay(THUMB_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delay(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for other fingers to finish
  delay(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME - THUMB_DELAY);

  stopAllMotors();
}

void performPoint() {
  Serial.println(F("Point..."));

  // Retract middle, ring+pinky, and thumb (keep index extended)
  digitalWrite(M1_DIR, HIGH);   // Middle retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  digitalWrite(M4_DIR, HIGH);   // Ring+Pinky retract

  analogWrite(E1_PWM, MOTOR_SPEED);  // Middle
  analogWrite(E4_PWM, MOTOR_SPEED);  // Ring+Pinky

  delay(THUMB_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delay(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for other fingers to finish
  delay(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME - THUMB_DELAY);

  stopAllMotors();
}

void performPeaceSign() {
  Serial.println(F("Peace Sign..."));

  // Retract middle, ring+pinky, and thumb (keep index+middle extended)
  digitalWrite(M1_DIR, HIGH);   // Middle retract
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  digitalWrite(M4_DIR, HIGH);   // Ring+Pinky retract

  analogWrite(E1_PWM, MOTOR_SPEED);  // Middle
  analogWrite(E4_PWM, MOTOR_SPEED);  // Ring+Pinky

  delay(THUMB_DELAY);
  // Start thumb
  analogWrite(E3_PWM, MOTOR_SPEED);
  delay(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  // Wait for other fingers to finish
  delay(FINGER_RETRACT_TIME - THUMB_RETRACT_TIME - THUMB_DELAY);

  stopAllMotors();
}

void performThumbOnly() {
  Serial.println(F("Thumb Only..."));

  // Retract thumb only
  digitalWrite(M3_DIR, LOW);    // Thumb retract
  analogWrite(E3_PWM, MOTOR_SPEED);

  delay(THUMB_RETRACT_TIME);
  // Stop thumb at 50%
  analogWrite(E3_PWM, 0);

  stopAllMotors();
}

void performIndexOnly() {
  Serial.println(F("Index Only..."));
  // Retract index only
  digitalWrite(M2_DIR, LOW);    // Index retract
  analogWrite(E2_PWM, MOTOR_SPEED);   // Index Finger

  delay(FINGER_RETRACT_TIME);
  stopAllMotors();
}

void performMiddleOnly() {
  Serial.println(F("Middle Only..."));
  // Retract middle only
  digitalWrite(M1_DIR, HIGH);   // Middle retract
  analogWrite(E1_PWM, MOTOR_SPEED);  // Middle Finger

  delay(FINGER_RETRACT_TIME);
  stopAllMotors();
}

void performRingPinkyOnly() {
  Serial.println(F("Ring+Pinky Only..."));
  // Retract ring+pinky only
  digitalWrite(M4_DIR, HIGH);   // Ring+Pinky retract
  analogWrite(E4_PWM, MOTOR_SPEED); // Ring+Pinky

  delay(FINGER_RETRACT_TIME);
  stopAllMotors();
}
