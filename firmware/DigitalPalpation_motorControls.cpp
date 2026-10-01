// =====================================================================
// DigitalPalpation_motorControls.cpp   (firmware for the LONGRUNER board)
// Host script: DigitalPalpation_v0.py
//
// Same hardware / pins as LONGRUNER_FLASH_v0.ino:
//   Motor 1 = X slot (CoreXY belt 1)   Motor 2 = Y slot (CoreXY belt 2)
//   Motor 3 = Z slot (translation stage) Motor 4 = A slot (syringe injector)
//
// What's new vs LONGRUNER_FLASH_v0:
//   * Every motor's position is tracked as an absolute step count (long),
//     so the host can remember HOME / sample positions and drive back to them.
//   * Speed is set PER MOTOR (the old firmware had one shared speed), so the
//     stage can run at 1/2 while the CoreXY + injector run at 2/3.
//   * Absolute "go to" moves. All motors in a move are linearly interpolated
//     (Bresenham) so they start and finish together -> CoreXY moves are
//     straight lines. The move runs at the speed of the motor with the most
//     steps to travel (the host only ever moves one group at a time:
//     XY together, Z alone, or A alone).
//
// Positions are relative to power-on / serial-open (opening the port resets
// the Arduino -> all positions read 0). There are no endstops, so HOME is
// simply "wherever the user jogged to and pressed SET HOME".
//
// ---------------- Serial protocol (115200 baud, one command per line) -----
//   I                 -> "ID:DP_MOTOR"          (port auto-discovery)
//   S<m><lvl>         set speed level 1..3 for motor m (1..4), e.g. "S43"
//                     -> "SPEED:m,lvl"
//   D<m> <us>         set an exact step delay (microseconds) for motor m,
//                     e.g. "D1 4000" (= a "1.5" speed, between levels 1 and 2)
//                     clamped to 800..20000 us   -> "DELAY:m,us"
//   J<m><+|->         jog motor m continuously. Must be re-sent at least
//                     every JOG_TIMEOUT_MS or the motor auto-stops (same
//                     watchdog idea as the old firmware). e.g. "J3+"
//   H<m>              halt one jogging motor           -> "HALT:m"
//   X   (or ' ')      stop EVERYTHING (jogs + moves)   -> "STOPPED"
//   G a b c d         absolute move: motor1..4 to step positions a..d
//                     -> "ACK:G" then "DONE a,b,c,d" when finished
//                     -> "ERR:BUSY" if a move is already running
//   P                 -> "POS a,b,c,d"
//   Z                 zero all position counters      -> "ZEROED"
//   E1 / E0           energize / de-energize ALL drivers -> "ENABLED" / "DISABLED"
//                     (the CNC shield has ONE shared enable pin, D8, so the 4
//                      motors can only be powered on/off together). E0 stops any
//                     motion first. Any J or G command re-energizes automatically.
//                     Position counters are kept while de-energized.
//   ?                 -> "STATUS:..." (speeds, busy flag, positions)
// While anything is moving the board also streams "POS a,b,c,d" ~10x/s.
// =====================================================================
#include <Arduino.h>
#include <stdlib.h>

// ---------------- Pins (unchanged from LONGRUNER_FLASH_v0) --------------
static const int NUM_MOTORS = 4;
static const int stepPins[NUM_MOTORS] = {2, 3, 4, 12};
static const int dirPins[NUM_MOTORS]  = {5, 6, 7, 13};
static const int enablePin = 8;          // LOW = drivers energized, HIGH = coils off
static bool motorsEnabled = true;
static const unsigned int ENABLE_SETTLE_US = 2000;   // driver wake-up before the first step

// ---------------- Speeds ------------------------------------------------
// Same step-delay table as LONGRUNER_FLASH_v0 (microseconds between steps):
//   level 1 = 5000 us (200 steps/s)
//   level 2 = 3000 us (~333 steps/s)
//   level 3 = 1500 us (~667 steps/s)
// Speed convention used by DigitalPalpation_v0.py:
//   CoreXY (M1,M2) + injector (M4):  LOW/slow = 2, HIGH/fast = 3
//   Z stage (M3):                    LOW/slow = 1, HIGH/fast = 2
static const unsigned long stepDelays[3] = {5000UL, 3000UL, 1500UL};
static const unsigned long stepPulseUs = 20;
static unsigned long stepDelayUs[NUM_MOTORS] = {1500UL, 1500UL, 3000UL, 1500UL};
static const unsigned long MIN_DELAY_US = 800, MAX_DELAY_US = 20000;

// ---------------- State -------------------------------------------------
static long pos[NUM_MOTORS] = {0, 0, 0, 0};

// Jog state (per motor)
static int8_t jogDir[NUM_MOTORS] = {0, 0, 0, 0};   // +1 / -1 / 0
static unsigned long jogLastCmdMs[NUM_MOTORS] = {0, 0, 0, 0};
static unsigned long jogLastStepUs[NUM_MOTORS] = {0, 0, 0, 0};
static const unsigned long JOG_TIMEOUT_MS = 300;

// Move (absolute, interpolated) state
static bool moveActive = false;
static long moveDelta[NUM_MOTORS];      // |steps| per motor
static int8_t moveSign[NUM_MOTORS];
static long moveErr[NUM_MOTORS];
static long moveLeadSteps = 0;          // steps of the longest axis
static long moveLeadDone = 0;
static unsigned long moveDelayUs = 0;
static unsigned long moveLastStepUs = 0;

// Position streaming
static unsigned long lastPosReportMs = 0;
static const unsigned long POS_REPORT_MS = 100;
static bool wasMoving = false;

// Serial line buffer
static char lineBuf[64];
static uint8_t lineLen = 0;

// ---------------- Helpers -----------------------------------------------
static void setDir(int m, int8_t sign) {
  digitalWrite(dirPins[m], sign > 0 ? HIGH : LOW);
}

static void pulse(int m) {
  digitalWrite(stepPins[m], HIGH);
  delayMicroseconds(stepPulseUs);
  digitalWrite(stepPins[m], LOW);
}

static void printPos(const char* prefix) {
  Serial.print(prefix);
  for (int m = 0; m < NUM_MOTORS; m++) {
    Serial.print(pos[m]);
    if (m < NUM_MOTORS - 1) Serial.print(',');
  }
  Serial.println();
}

static bool anyMoving() {
  if (moveActive) return true;
  for (int m = 0; m < NUM_MOTORS; m++) if (jogDir[m] != 0) return true;
  return false;
}

static void stopAll() {
  moveActive = false;
  for (int m = 0; m < NUM_MOTORS; m++) {
    jogDir[m] = 0;
    digitalWrite(stepPins[m], LOW);
  }
}

static void enableMotors() {
  if (motorsEnabled) return;
  digitalWrite(enablePin, LOW);
  motorsEnabled = true;
  delayMicroseconds(ENABLE_SETTLE_US);     // let the drivers' outputs come up
  Serial.println("ENABLED");
}

static void disableMotors() {
  stopAll();                               // never cut power mid-move
  digitalWrite(enablePin, HIGH);
  motorsEnabled = false;
  Serial.println("DISABLED");
}

static void startMove(const long target[NUM_MOTORS]) {
  moveLeadSteps = 0;
  int lead = 0;
  for (int m = 0; m < NUM_MOTORS; m++) {
    long d = target[m] - pos[m];
    moveSign[m] = (d >= 0) ? 1 : -1;
    moveDelta[m] = labs(d);
    if (moveDelta[m] > moveLeadSteps) { moveLeadSteps = moveDelta[m]; lead = m; }
    setDir(m, moveSign[m]);
  }
  Serial.println("ACK:G");
  if (moveLeadSteps == 0) {           // already there
    printPos("DONE ");
    return;
  }
  for (int m = 0; m < NUM_MOTORS; m++) moveErr[m] = moveLeadSteps / 2;
  moveDelayUs = stepDelayUs[lead];
  moveLeadDone = 0;
  moveLastStepUs = micros();
  delayMicroseconds(5);               // DIR setup time before first STEP
  moveActive = true;
}

static void serviceMove(unsigned long nowUs) {
  if (!moveActive) return;
  if (nowUs - moveLastStepUs < moveDelayUs) return;
  // Fixed cadence: schedule the next step from the IDEAL time of this one,
  // not from "now", so loop jitter never accumulates. Every move therefore
  // runs at exactly N x delay, which lets the host map time -> steps
  // precisely for the pressure data. (Resync if we ever fall a full step behind.)
  moveLastStepUs += moveDelayUs;
  if (nowUs - moveLastStepUs >= moveDelayUs) moveLastStepUs = nowUs;

  // Bresenham: every motor advances proportionally to the lead motor.
  bool stepNow[NUM_MOTORS] = {false, false, false, false};
  for (int m = 0; m < NUM_MOTORS; m++) {
    if (moveDelta[m] == 0) continue;
    moveErr[m] -= moveDelta[m];
    if (moveErr[m] < 0) {
      moveErr[m] += moveLeadSteps;
      stepNow[m] = true;
    }
  }
  // Pulse all due motors together
  for (int m = 0; m < NUM_MOTORS; m++) if (stepNow[m]) digitalWrite(stepPins[m], HIGH);
  delayMicroseconds(stepPulseUs);
  for (int m = 0; m < NUM_MOTORS; m++) {
    if (stepNow[m]) {
      digitalWrite(stepPins[m], LOW);
      pos[m] += moveSign[m];
    }
  }

  moveLeadDone++;
  if (moveLeadDone >= moveLeadSteps) {
    moveActive = false;
    printPos("DONE ");
  }
}

static void serviceJogs(unsigned long nowMs, unsigned long nowUs) {
  for (int m = 0; m < NUM_MOTORS; m++) {
    if (jogDir[m] == 0) continue;
    if (nowMs - jogLastCmdMs[m] > JOG_TIMEOUT_MS) {  // watchdog
      jogDir[m] = 0;
      continue;
    }
    if (nowUs - jogLastStepUs[m] >= stepDelayUs[m]) {
      pulse(m);
      pos[m] += jogDir[m];
      jogLastStepUs[m] = nowUs;
    }
  }
}

// ---------------- Command parsing ---------------------------------------
static int motorIndex(char c) {          // '1'..'4' -> 0..3, else -1
  if (c >= '1' && c <= '4') return c - '1';
  return -1;
}

static void handleLine(char* s) {
  while (*s == ' ' && s[1] != '\0') s++;  // allow leading spaces (but keep lone ' ')
  char c = s[0];

  switch (c) {
    case 'I':
      Serial.println("ID:DP_MOTOR");
      break;

    case 'X': case ' ':
      stopAll();
      Serial.println("STOPPED");
      printPos("POS ");
      break;

    case 'P':
      printPos("POS ");
      break;

    case 'Z':
      if (anyMoving()) { Serial.println("ERR:BUSY"); break; }
      for (int m = 0; m < NUM_MOTORS; m++) pos[m] = 0;
      Serial.println("ZEROED");
      break;

    case 'S': {
      int m = motorIndex(s[1]);
      int lvl = s[2] - '0';
      if (m < 0 || lvl < 1 || lvl > 3) { Serial.println("ERR:BAD_S"); break; }
      stepDelayUs[m] = stepDelays[lvl - 1];
      Serial.print("SPEED:"); Serial.print(m + 1); Serial.print(','); Serial.println(lvl);
      break;
    }

    case 'D': {
      int m = motorIndex(s[1]);
      char* end;
      long us = strtol(s + 2, &end, 10);
      if (m < 0 || end == s + 2) { Serial.println("ERR:BAD_D"); break; }
      if (us < (long)MIN_DELAY_US) us = MIN_DELAY_US;
      if (us > (long)MAX_DELAY_US) us = MAX_DELAY_US;
      stepDelayUs[m] = (unsigned long)us;
      Serial.print("DELAY:"); Serial.print(m + 1); Serial.print(','); Serial.println(us);
      break;
    }

    case 'J': {
      if (moveActive) break;              // ignore jogs during automated moves
      int m = motorIndex(s[1]);
      if (m < 0 || (s[2] != '+' && s[2] != '-')) { Serial.println("ERR:BAD_J"); break; }
      int8_t sign = (s[2] == '+') ? 1 : -1;
      enableMotors();                     // auto re-energize (no-op if already on)
      if (jogDir[m] != sign) {
        setDir(m, sign);
        jogDir[m] = sign;
        jogLastStepUs[m] = micros();
      }
      jogLastCmdMs[m] = millis();         // refresh watchdog
      break;
    }

    case 'H': {
      int m = motorIndex(s[1]);
      if (m < 0) { Serial.println("ERR:BAD_H"); break; }
      jogDir[m] = 0;
      Serial.print("HALT:"); Serial.println(m + 1);
      break;
    }

    case 'G': {
      if (anyMoving()) { Serial.println("ERR:BUSY"); break; }
      long target[NUM_MOTORS];
      char* p = s + 1;
      for (int m = 0; m < NUM_MOTORS; m++) {
        char* end;
        target[m] = strtol(p, &end, 10);
        if (end == p) { Serial.println("ERR:BAD_G"); return; }
        p = end;
      }
      enableMotors();                     // auto re-energize (no-op if already on)
      startMove(target);
      break;
    }

    case 'E':
      if (s[1] == '1') { if (motorsEnabled) Serial.println("ENABLED"); else enableMotors(); }
      else if (s[1] == '0') disableMotors();
      else Serial.println("ERR:BAD_E");
      break;

    case '?':
      Serial.print("STATUS:busy=");
      Serial.print(anyMoving() ? 1 : 0);
      Serial.print(",enabled=");
      Serial.print(motorsEnabled ? 1 : 0);
      Serial.print(",delays_us=");
      for (int m = 0; m < NUM_MOTORS; m++) {
        Serial.print(stepDelayUs[m]);
        if (m < NUM_MOTORS - 1) Serial.print('/');
      }
      Serial.print(",pos=");
      for (int m = 0; m < NUM_MOTORS; m++) {
        Serial.print(pos[m]);
        if (m < NUM_MOTORS - 1) Serial.print('/');
      }
      Serial.println();
      break;

    default:
      Serial.println("ERR:UNKNOWN");
      break;
  }
}

static void readSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
      }
      lineLen = 0;
      continue;
    }
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = ch;
  }
}

// ---------------- Arduino entry points ----------------------------------
void setup() {
  Serial.begin(115200);
  for (int m = 0; m < NUM_MOTORS; m++) {
    pinMode(stepPins[m], OUTPUT);
    pinMode(dirPins[m], OUTPUT);
    digitalWrite(stepPins[m], LOW);
    digitalWrite(dirPins[m], HIGH);
  }
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);   // drivers enabled

  Serial.println("READY");
  Serial.println("ID:DP_MOTOR");
}

void loop() {
  readSerial();

  unsigned long nowUs = micros();
  unsigned long nowMs = millis();
  serviceMove(nowUs);
  serviceJogs(nowMs, nowUs);

  // Stream positions while moving + one final report when everything stops
  bool moving = anyMoving();
  if (moving && nowMs - lastPosReportMs >= POS_REPORT_MS) {
    printPos("POS ");
    lastPosReportMs = nowMs;
  }
  if (wasMoving && !moving) printPos("POS ");
  wasMoving = moving;
}
