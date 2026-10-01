// Motor Control - Simple & Reliable
// 4 independent stepper motors on the CNC Shield's X/Y/Z/A driver slots, run in
// full-step mode (set by the shield's MS1/MS2/MS3 jumpers, not by this code).
// Motor 1 = X slot | Motor 2 = Y slot | Motor 3 = Z slot | Motor 4 = A slot
//
// Motor 1 (X) and Motor 2 (Y) are wired as a CoreXY belt pair, not independent
// orthogonal axes - see the CoreXY note in PYTHON_MAIN_v0.py for the belt math
// that turns a real-world X/Y direction into a pair of belt commands. This
// firmware doesn't know anything about CoreXY; all it needs to be able to do
// is move any combination of the 4 motors at the same time, which is why
// movement state below is tracked per-motor (arrays) instead of one shared
// "currentDirection" like earlier versions of this file used.
const int xStepPin = 2, xDirPin = 5;    // Motor 1 (CoreXY belt 1)
const int yStepPin = 3, yDirPin = 6;    // Motor 2 (CoreXY belt 2)
const int zStepPin = 4, zDirPin = 7;    // Motor 3
const int aStepPin = 12, aDirPin = 13;  // Motor 4
const int enablePin = 8;                // shared enable line for all 4 driver slots

const int NUM_MOTORS = 4;
const int stepPins[NUM_MOTORS] = {xStepPin, yStepPin, zStepPin, aStepPin};
const int dirPins[NUM_MOTORS]  = {xDirPin,  yDirPin,  zDirPin,  aDirPin};

// Serial commands, per motor index [0..3] = [Motor1, Motor2, Motor3, Motor4]:
//   FWD_CMD[m] / REV_CMD[m] jog that motor; STOP_CMD[m] stops only that motor
//   (needed so a CoreXY move can hold one belt still while the other turns).
//   ' ' stops every motor at once. Which physical key/button sends which byte
//   lives entirely in PYTHON_MAIN_v0.py and can change without touching this file.
const char FWD_CMD[NUM_MOTORS]  = {'q', 'w', 'e', 'r'};
const char REV_CMD[NUM_MOTORS]  = {'a', 's', 'd', 'f'};
const char STOP_CMD[NUM_MOTORS] = {'Q', 'W', 'E', 'R'};

// Speed settings (shared by all motors - CoreXY needs both belts stepping at
// the same rate for a coordinated move to actually track straight)
unsigned long stepDelays[] = {5000, 3000, 1500};  // 1, 2, 3
int currentSpeed = 2;
unsigned long stepDelay = stepDelays[currentSpeed - 1];
const unsigned long stepPulse = 20;  // step pulse width, us

// Modes
bool continuousMode = true;  // true = continuous, false = fixed
const int fixedSteps = 100;

// Per-motor movement state
bool moving[NUM_MOTORS]              = {false, false, false, false};
int8_t dirSign[NUM_MOTORS]           = {0, 0, 0, 0};  // +1 fwd, -1 rev, 0 idle
int stepsRemaining[NUM_MOTORS]       = {0, 0, 0, 0};
unsigned long lastStepTime[NUM_MOTORS]    = {0, 0, 0, 0};
unsigned long lastCommandTime[NUM_MOTORS] = {0, 0, 0, 0};
const unsigned long commandTimeout = 300;  // ms before auto-stop, per motor

void setup() {
  Serial.begin(115200);

  for (int m = 0; m < NUM_MOTORS; m++) {
    pinMode(stepPins[m], OUTPUT);
    pinMode(dirPins[m], OUTPUT);
    digitalWrite(stepPins[m], LOW);
    digitalWrite(dirPins[m], HIGH);
  }
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  Serial.println("READY");
  Serial.print("MODE:");
  Serial.println(continuousMode ? "CONTINUOUS" : "FIXED");
  Serial.print("SPEED:");
  Serial.println(currentSpeed);
}

void loop() {
  // Read serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;
    processCommand(cmd);
  }

  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  for (int m = 0; m < NUM_MOTORS; m++) {
    // Auto-stop this motor if its commands have gone stale (continuous mode only)
    if (continuousMode && moving[m] && (nowMs - lastCommandTime[m] > commandTimeout)) {
      stopOne(m);
      continue;
    }

    if (!moving[m]) continue;

    if (nowUs - lastStepTime[m] >= stepDelay) {
      if (continuousMode) {
        doStep(m);
      } else if (stepsRemaining[m] > 0) {
        doStep(m);
        stepsRemaining[m]--;
        if (stepsRemaining[m] <= 0) {
          stopOne(m);
          Serial.print("FIXED_DONE:");
          Serial.println(m + 1);
        }
      }
      lastStepTime[m] = nowUs;
    }
  }
}

void processCommand(char cmd) {
  // Per-motor movement / stop commands
  for (int m = 0; m < NUM_MOTORS; m++) {
    if (cmd == FWD_CMD[m] || cmd == REV_CMD[m]) {
      startOne(m, cmd == FWD_CMD[m] ? 1 : -1, cmd);
      return;
    }
    if (cmd == STOP_CMD[m]) {
      stopOne(m);
      Serial.print("STOP:");
      Serial.println(m + 1);
      return;
    }
  }

  switch (cmd) {
    case ' ':  // Stop everything
      for (int m = 0; m < NUM_MOTORS; m++) stopOne(m);
      Serial.println("STOP");
      break;

    case '1': case '2': case '3':
      currentSpeed = cmd - '0';
      stepDelay = stepDelays[currentSpeed - 1];
      Serial.print("SPEED:");
      Serial.println(currentSpeed);
      break;

    case 'm':  // Toggle continuous/fixed mode
      for (int m = 0; m < NUM_MOTORS; m++) stopOne(m);
      continuousMode = !continuousMode;
      Serial.print("MODE:");
      Serial.println(continuousMode ? "CONTINUOUS" : "FIXED");
      break;

    case '?':  // Status - one direction sign per motor: -1 rev, 0 idle, 1 fwd
      Serial.print("STATUS:mode=");
      Serial.print(continuousMode ? "CONT" : "FIXED");
      Serial.print(",speed=");
      Serial.print(currentSpeed);
      for (int m = 0; m < NUM_MOTORS; m++) {
        Serial.print(",m");
        Serial.print(m + 1);
        Serial.print("=");
        Serial.print(dirSign[m]);
      }
      Serial.println();
      break;

    case 'I':  // Identify - used by host script to auto-discover this device's port
      Serial.println("ID:LONGRUNER_MOTOR");
      break;

    case 't': {  // Diagnostic: slowly pulse each motor's STEP+DIR pins in turn
      // so you can confirm with a multimeter/LED that the Arduino side is
      // actually driving each pin, independent of the CNC shield's wiring.
      for (int m = 0; m < NUM_MOTORS; m++) stopOne(m);
      Serial.println("PINTEST_BEGIN");
      const char* names[NUM_MOTORS] = {"M1(X)", "M2(Y)", "M3(Z)", "M4(A)"};
      for (int m = 0; m < NUM_MOTORS; m++) {
        Serial.print("Testing "); Serial.print(names[m]);
        Serial.print(" step=D"); Serial.print(stepPins[m]);
        Serial.print(" dir=D");  Serial.println(dirPins[m]);
        for (int p = 0; p < 20; p++) {
          digitalWrite(dirPins[m], p % 2);
          digitalWrite(stepPins[m], HIGH);
          delay(50);
          digitalWrite(stepPins[m], LOW);
          delay(50);
        }
      }
      Serial.println("PINTEST_DONE");
      break;
    }
  }
}

void startOne(int m, int8_t sign, char cmd) {
  if (dirSign[m] != sign) {
    // New direction (including a reversal) for this motor
    digitalWrite(dirPins[m], sign > 0 ? HIGH : LOW);
    dirSign[m] = sign;
    moving[m] = true;

    if (continuousMode) {
      Serial.print("CONT:");
      Serial.println(cmd);
    } else {
      stepsRemaining[m] = fixedSteps;
      Serial.print("FIXED:");
      Serial.println(cmd);
    }
  }
  // Whether this is a new direction or a repeat of the current one, refresh
  // this motor's own watchdog timer
  lastCommandTime[m] = millis();
}

void stopOne(int m) {
  moving[m] = false;
  dirSign[m] = 0;
  stepsRemaining[m] = 0;
  digitalWrite(stepPins[m], LOW);
}

void doStep(int m) {
  digitalWrite(stepPins[m], HIGH);
  delayMicroseconds(stepPulse);
  digitalWrite(stepPins[m], LOW);
}
