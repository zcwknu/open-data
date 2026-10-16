// =====================================================================
// DigitalPalpation_motorControls.cpp      firmware for the LONGRUNER board
//                                         (Arduino Uno + GRBL CNC shield V3,
//                                          DRV8825 drivers at 1/8 microstep)
// Host script: DigitalPalpation_v0.py     (protocol version: DP_MOTOR_V4)
//
// HOW THIS FILE IS LAID OUT
//   1. USER SETTINGS ...... every pin, timing and limit you might want to change
//   2. REFERENCE NOTES .... hardware wiring + the serial command list (read-only)
//   3. INTERNAL STATE ..... variables the code uses at run time (don't edit)
//   4. HELPERS
//   5. MOTION ............. absolute moves, homing seeks, jogs
//   6. COMMAND PARSER ..... one case per serial command
//   7. ARDUINO ENTRY POINTS (setup / loop)
// Only section 1 should need editing. If you change something that the Python
// script also knows about, the matching Python setting is named in the comment.
// =====================================================================
#include <Arduino.h>
#include <stdlib.h>


// #####################################################################
// 1. USER SETTINGS
// #####################################################################

// ---- 1a. Identity / serial link -------------------------------------
#define FIRMWARE_ID   "ID:DP_MOTOR_V4"   // Python: MOTOR_ID (must match exactly)
#define SERIAL_BAUD   115200             // Python: BAUD
#define LINE_BUF_LEN  64                 // longest command line accepted (chars)

// ---- 1b. Motor pins (GRBL CNC shield V3 slots) ----------------------
//   index 0 = Motor 1 = X slot (CoreXY belt 1)
//   index 1 = Motor 2 = Y slot (CoreXY belt 2)
//   index 2 = Motor 3 = Z slot (translation stage, NEMA23)
//   index 3 = Motor 4 = A slot (syringe injector, NEMA11; needs the A-axis jumpers)
static const int NUM_MOTORS = 4;
static const int NUM_BELTS  = 2;                          // motors 0..1 are the CoreXY belts
static const int STEP_PINS[NUM_MOTORS] = {2, 3, 4, 12};   // X, Y, Z, A step
static const int DIR_PINS[NUM_MOTORS]  = {5, 6, 7, 13};   // X, Y, Z, A direction

// ---- 1c. Motor power / driver enable   [MOTOR-POWER] ----------------
// CURRENTLY OFF (2026-10-07): the drivers are ALWAYS energized and the firmware
// never drives an enable pin. This board's D8 is shorted to GND (damaged while the
// shield's EN-GND jumper was fitted), so it must never be driven again.
// The EN pin is left as a high-impedance INPUT; the DRV8825s' internal pull-down
// keeps them on. E1 / E0 still answer "ENABLED" so the host protocol is unchanged.
//
// TO BRING MOTOR POWER CONTROL BACK (after swapping hardware):
//   1. set ENABLE_PIN_CONTROL to 1
//   2. set ENABLE_PIN: 8 = the shield's own EN line (new Arduino, NO EN-GND jumper),
//      or 11 = wire D11 (Z-limit header) -> EN header pin through ~330 ohm, with
//      the old D8 isolated from the shield
//   3. re-enable the POWER button / idle-off in DigitalPalpation_v0.py
//      (MOTOR_POWER_BUTTON / AUTO_MOTOR_OFF, search for [MOTOR-POWER])
#define ENABLE_PIN_CONTROL 0             // 0 = always on (never drive EN), 1 = E0/E1 work
#define ENABLE_PIN         8             // active LOW, shared by all 4 drivers
static const unsigned int ENABLE_SETTLE_US = 2000;   // driver wake-up before the first step

// ---- 1d. Limit switches (CoreXY home corner) ------------------------
static const int X_LIMIT_PIN = 9;        // CNC shield X endstop header (X+ and X- both = D9)
static const int Y_LIMIT_PIN = 10;       // CNC shield Y endstop header (Y+ and Y- both = D10)
static const unsigned long LIMIT_DEBOUNCE_US = 1000;  // switch must read steady this long

// ---- 1e. Step timing --------------------------------------------------
// A "delay" is the time between two (micro)steps in microseconds:
//   steps/s = 1,000,000 / delay.   The host normally sends exact delays ("D"),
//   so the speed levels below are only used by the old "S" command.
static const unsigned long SPEED_LEVEL_US[3] = {5000UL, 3000UL, 1500UL};  // S..1 / S..2 / S..3
static const unsigned long DEFAULT_DELAY_US[NUM_MOTORS] = {1500UL, 1500UL, 3000UL, 1500UL};
static const unsigned long STEP_PULSE_US = 5;        // STEP high time (DRV8825 needs 1.9 us)
static const unsigned long DIR_SETUP_US  = 5;        // DIR -> first STEP setup time
static const unsigned long MIN_DELAY_US  = 100;      // fastest allowed: 100 us = 10 kHz
                                                     // (Python never asks below MIN_STEP_US = 188)
static const unsigned long MAX_DELAY_US  = 1000000UL;// slowest allowed: 1 s per step
                                                     // (quasi-static injection rates)

// ---- 1f. CoreXY belt ramps (motors 1 + 2 only) -----------------------
// The ramp itself (start delay + length) is sent by the host with "R"
// (Python: XY_RAMP_START_US / XY_RAMP_FULLSTEPS). These are just safety caps.
static const unsigned long MAX_RAMP_START_US = 20000UL;  // keeps ramp math within 32 bits
static const long          MAX_RAMP_LEN      = 5000;     // steps

// ---- 1g. Jogging / reporting -----------------------------------------
static const unsigned long JOG_TIMEOUT_MS = 300;  // jog stops if "J" isn't re-sent this often
                                                  // (Python re-sends every JOG_REPEAT_S)
static const unsigned long POS_REPORT_MS  = 200;  // "POS" stream interval while jogging
static const unsigned long BOOT_SETTLE_MS = 5;    // pull-up settle before reading switches


// #####################################################################
// 2. REFERENCE NOTES (nothing to edit here)
// #####################################################################
//
// ---------------- Positions -------------------------------------------
// Every motor's position is an absolute step count (long). Positions are relative
// to power-on / serial-open (opening the port resets the Arduino -> all read 0)
// until the host runs X-Y HOMING, which sets X = 0, Y = 0 at the switch corner.
// Microstepping is set by the MS jumpers under each driver, NOT by this code:
// the host (Python XY_MICROSTEPS / Z_MICROSTEPS / A_MICROSTEPS) already sends
// delays and targets in microsteps, so this firmware just counts steps.
//
// ---------------- Moves -------------------------------------------------
//   * "G" moves are absolute and linearly interpolated (Bresenham), so all motors
//     in a move start and finish together -> CoreXY moves are straight lines.
//     The move runs at the speed of the motor with the most steps.
//   * CoreXY moves ramp up and down (finishing exactly on the target step).
//     Z / A moves run at ONE fixed cadence (the host maps time -> steps for the
//     pressure data), and can be trimmed live by the closed-loop "V" command.
//   * Jogs only ramp UP; releasing a jog (H), limits, soft max, the watchdog and
//     X all stop immediately - no coasting.
//   * Position lines are NOT streamed during automated moves (printing costs a
//     few hundred us and caused stutters), only during jogs.
//
// ---------------- Limit switches (SOUTH-WEST corner = -X / -Y) ----------
//   Normally-open switches to GND with internal pull-ups: pressed = LOW.
//   Manual jogs can't drive INTO a pressed switch ("LIMIT:X" / "LIMIT:Y");
//   driving away from it is always allowed. G moves don't check the switches
//   (they only go to positions you already jogged to).
//
// ---------------- Soft max limits ("M maxX maxY", set after homing) ----
//   carriage X = -(belt1 + belt2) / 2,  Y = (belt1 - belt2) / 2  (same as Python)
//   Jogs stop at maxX / maxY ("LIMIT:MAX"), G moves past them -> "ERR:SOFTLIMIT".
//
// ---------------- Serial protocol (one command per line) ---------------
//   I                 -> FIRMWARE_ID                (port auto-discovery)
//   S<m><lvl>         speed level 1..3 for motor m (1..4), e.g. "S43" -> "SPEED:m,lvl"
//   D<m> <us>         exact step delay for motor m, e.g. "D1 4000"
//                     clamped to MIN_DELAY_US..MAX_DELAY_US -> "DELAY:m,us"
//   R startUs steps   belt ramp: start/stop delay + ramp length. "R 0 0" = off
//                     -> "RAMP:startUs,steps"
//   J<m><+|->         jog motor m; re-send within JOG_TIMEOUT_MS or it stops
//   H<m>              halt one jogging motor           -> "HALT:m"
//   X   (or ' ')      stop EVERYTHING (jogs + moves)   -> "STOPPED"
//   G a b c d         absolute move of motors 1..4     -> "ACK:G" ... "DONE a,b,c,d"
//                     ("ERR:BUSY" if already moving)
//   V us [extra]      closed-loop trim of the running Z / A move: new step delay,
//                     plus `extra` steps appended   -> "VP <done> <total>"
//   P                 -> "POS a,b,c,d"
//   Z / Z12           zero all / only the listed position counters -> "ZEROED"
//   L                 limit switch states             -> "LIM:<x>,<y>" (1 = pressed)
//   M maxX maxY / M0  soft max on / off  -> "SOFTMAX:maxX,maxY" / "SOFTMAX:OFF"
//   K s1 s2 A w max   homing SEEK: step belt 1 / 2 in directions s1, s2 (-1/0/+1)
//                     until switch A (X/Y) reads w (1 pressed, 0 released) or max
//                     steps -> "SEEK:HIT|MAX|OTHER a,b,c,d"
//   E1 / E0           drivers on / off (only with ENABLE_PIN_CONTROL 1; otherwise
//                     both reply "ENABLED")
//   ?                 -> "STATUS:busy=..,enabled=..,delays_us=..,pos=..,lim=.."
// While jogging the board streams "POS a,b,c,d" every POS_REPORT_MS.


// #####################################################################
// 3. INTERNAL STATE (run-time variables - no need to edit)
// #####################################################################
static bool motorsEnabled = true;

struct Limit { int pin; bool stable; bool cand; unsigned long since; };
static Limit limX = {X_LIMIT_PIN, false, false, 0};
static Limit limY = {Y_LIMIT_PIN, false, false, 0};

static unsigned long stepDelayUs[NUM_MOTORS];  // current delay per motor (set in setup)

// Belt ramp (from "R"): linear change of delay between rampStartUs and the target.
static unsigned long rampStartUs = 0;   // 0 = ramps off
static long rampLen = 0;                // steps for a full ramp (target -> 0 us)

static long pos[NUM_MOTORS] = {0, 0, 0, 0};

// Jog state (per motor)
static int8_t jogDir[NUM_MOTORS] = {0, 0, 0, 0};   // +1 / -1 / 0
static unsigned long jogLastCmdMs[NUM_MOTORS] = {0, 0, 0, 0};
static unsigned long jogLastStepUs[NUM_MOTORS] = {0, 0, 0, 0};
static long jogRampN[NUM_MOTORS] = {0, 0, 0, 0};       // steps into the ramp-up (0 = slowest)

// Move (absolute, interpolated) state
static bool moveActive = false;
static long moveDelta[NUM_MOTORS];      // |steps| per motor
static int8_t moveSign[NUM_MOTORS];
static long moveErr[NUM_MOTORS];
static long moveLeadSteps = 0;          // steps of the longest axis
static int moveLead = 0;                // index of that axis
static long moveLeadDone = 0;
static unsigned long moveDelayUs = 0;
static unsigned long moveLastStepUs = 0;
static long moveRampLen = 0;            // 0 = constant speed (Z / A moves, or ramps off)

// Soft max X/Y limits (carriage steps, valid after homing)
static bool softMaxOn = false;
static long softMaxX = 0, softMaxY = 0;

// Seek (homing) state
static bool seekActive = false;
static int8_t seekSign[2] = {0, 0};
static Limit* seekLimit = nullptr;
static Limit* seekOther = nullptr;
static bool seekOtherStart = false;
static bool seekWant = true;
static long seekMax = 0, seekDone = 0;
static unsigned long seekLastStepUs = 0;

// Position streaming
static unsigned long lastPosReportMs = 0;
static bool wasMoving = false;

// Serial line buffer
static char lineBuf[LINE_BUF_LEN];
static uint8_t lineLen = 0;


// #####################################################################
// 4. HELPERS
// #####################################################################
static void setDir(int m, int8_t sign) {
  digitalWrite(DIR_PINS[m], sign > 0 ? HIGH : LOW);
}

static void pulse(int m) {
  digitalWrite(STEP_PINS[m], HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PINS[m], LOW);
}

static void printPos(const char* prefix) {
  Serial.print(prefix);
  for (int m = 0; m < NUM_MOTORS; m++) {
    Serial.print(pos[m]);
    if (m < NUM_MOTORS - 1) Serial.print(',');
  }
  Serial.println();
}

// Debounced limit switch: "pressed" only after the pin has read LOW steadily
// for LIMIT_DEBOUNCE_US (filters noise picked up from the motor cables).
static void updateLimit(Limit& l, unsigned long nowUs) {
  bool raw = (digitalRead(l.pin) == LOW);
  if (raw != l.cand) { l.cand = raw; l.since = nowUs; }
  else if (raw != l.stable && nowUs - l.since >= LIMIT_DEBOUNCE_US) l.stable = raw;
}

static bool anyMoving() {
  if (moveActive || seekActive) return true;
  for (int m = 0; m < NUM_MOTORS; m++) if (jogDir[m] != 0) return true;
  return false;
}

// Steps needed to ramp from rampStartUs to this target delay (0 = no ramp).
static long rampLenFor(unsigned long target) {
  if (rampStartUs == 0 || rampLen <= 0 || target >= rampStartUs) return 0;
  return (long)((unsigned long)rampLen * (rampStartUs - target) / rampStartUs);
}

// Delay for a step that is r steps from the slow end of a ramp of length len.
static unsigned long rampDelay(unsigned long target, long r, long len) {
  if (len <= 0 || r >= len) return target;
  if (r < 0) r = 0;
  return target + (rampStartUs - target) * (unsigned long)(len - r) / (unsigned long)len;
}

static void stopAll() {
  moveActive = false;
  seekActive = false;
  for (int m = 0; m < NUM_MOTORS; m++) {
    jogRampN[m] = 0;
    jogDir[m] = 0;
    digitalWrite(STEP_PINS[m], LOW);
  }
}

static void enableMotors() {
#if !ENABLE_PIN_CONTROL
  motorsEnabled = true;                    // always on: nothing to switch
  return;
#endif
  if (motorsEnabled) return;
  digitalWrite(ENABLE_PIN, LOW);
  motorsEnabled = true;
  delayMicroseconds(ENABLE_SETTLE_US);     // let the drivers' outputs come up
  Serial.println("ENABLED");
}

static void disableMotors() {
#if !ENABLE_PIN_CONTROL
  Serial.println("ENABLED");               // power control is off: drivers stay on
  return;
#endif
  stopAll();                               // never cut power mid-move
  digitalWrite(ENABLE_PIN, HIGH);
  motorsEnabled = false;
  Serial.println("DISABLED");
}

// #####################################################################
// 5. MOTION (absolute moves, homing seeks, jogs)
// #####################################################################
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
  // CoreXY moves ramp up and down; Z / A moves stay at one constant speed
  // (the host maps time -> steps for the pressure data on those).
  moveRampLen = 0;
  if (lead < NUM_BELTS) {
    moveRampLen = rampLenFor(moveDelayUs);
    if (moveRampLen > moveLeadSteps / 2) moveRampLen = moveLeadSteps / 2;
  }
  moveLeadDone = 0;
  moveLead = lead;
  moveLastStepUs = micros();
  delayMicroseconds(DIR_SETUP_US);               // DIR setup time before first STEP
  moveActive = true;
}

static void serviceMove(unsigned long nowUs) {
  if (!moveActive) return;
  unsigned long d = moveDelayUs;
  if (moveRampLen > 0) {                        // CoreXY ramp: distance to the nearer end
    long r = moveLeadDone;
    long toEnd = moveLeadSteps - 1 - moveLeadDone;
    if (toEnd < r) r = toEnd;
    d = rampDelay(moveDelayUs, r, moveRampLen);
  }
  if (nowUs - moveLastStepUs < d) return;
  // Fixed cadence: schedule the next step from the IDEAL time of this one,
  // not from "now", so loop jitter never accumulates. Every constant-speed
  // move therefore runs at exactly N x delay, which lets the host map
  // time -> steps precisely for the pressure data (Z / A moves).
  moveLastStepUs += d;
  if (nowUs - moveLastStepUs >= d) moveLastStepUs = nowUs;

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
  for (int m = 0; m < NUM_MOTORS; m++) if (stepNow[m]) digitalWrite(STEP_PINS[m], HIGH);
  delayMicroseconds(STEP_PULSE_US);
  for (int m = 0; m < NUM_MOTORS; m++) {
    if (stepNow[m]) {
      digitalWrite(STEP_PINS[m], LOW);
      pos[m] += moveSign[m];
    }
  }

  moveLeadDone++;
  if (moveLeadDone >= moveLeadSteps) {
    moveActive = false;
    printPos("DONE ");
  }
}

// CoreXY direction of the current belt jogs (same mapping as the Python
// COREXY_DIRECTIONS table):  x moves with -(belt1 + belt2),  y with (belt1 - belt2).
// Stop the belt jogs if they're heading INTO a pressed switch.
static void checkJogLimits() {
  int dx = -(jogDir[0] + jogDir[1]);
  int dy = jogDir[0] - jogDir[1];
  const char* hit = nullptr;
  if (limX.stable && dx < 0) hit = "LIMIT:X";
  else if (limY.stable && dy < 0) hit = "LIMIT:Y";
  else if (softMaxOn) {
    long x2 = -(pos[0] + pos[1]);          // 2 * X
    long y2 = pos[0] - pos[1];             // 2 * Y
    if (dx > 0 && x2 >= 2 * softMaxX) hit = "LIMIT:XMAX";
    else if (dy > 0 && y2 >= 2 * softMaxY) hit = "LIMIT:YMAX";
  }
  if (hit) {                               // limits stop immediately (no ramp)
    for (int m = 0; m < 2; m++) { jogDir[m] = 0; jogRampN[m] = 0; }
    Serial.println(hit);
  }
}

static void serviceSeek(unsigned long nowUs) {
  if (!seekActive) return;
  if (nowUs - seekLastStepUs < stepDelayUs[0]) return;
  seekLastStepUs += stepDelayUs[0];
  if (nowUs - seekLastStepUs >= stepDelayUs[0]) seekLastStepUs = nowUs;
  if (seekOther->stable && !seekOtherStart) {   // wrong switch closed - stop, don't grind
    seekActive = false;
    printPos("SEEK:OTHER ");
    return;
  }
  if (seekLimit->stable == seekWant) {          // reached the switch condition
    seekActive = false;
    printPos("SEEK:HIT ");
    return;
  }
  if (seekDone >= seekMax) {
    seekActive = false;
    printPos("SEEK:MAX ");
    return;
  }
  for (int m = 0; m < 2; m++) if (seekSign[m]) digitalWrite(STEP_PINS[m], HIGH);
  delayMicroseconds(STEP_PULSE_US);
  for (int m = 0; m < 2; m++) {
    if (seekSign[m]) {
      digitalWrite(STEP_PINS[m], LOW);
      pos[m] += seekSign[m];
    }
  }
  seekDone++;
}

// Would the next jog step (belt deltas d0, d1) carry the carriage past the soft
// max along an axis the jog is heading +ve on? Checked BEFORE stepping, so jogs
// stop exactly at the limit.
static bool jogStepPastMax(int8_t d0, int8_t d1) {
  if (!softMaxOn) return false;
  long p0 = pos[0] + d0, p1 = pos[1] + d1;
  int dx = -(jogDir[0] + jogDir[1]), dy = jogDir[0] - jogDir[1];
  return (dx > 0 && -(p0 + p1) > 2 * softMaxX) || (dy > 0 && (p0 - p1) > 2 * softMaxY);
}

static void stopBeltJogs() {
  for (int k = 0; k < 2; k++) { jogDir[k] = 0; jogRampN[k] = 0; }
  Serial.println("LIMIT:MAX");
}

static void serviceJogs(unsigned long nowMs, unsigned long nowUs) {
  checkJogLimits();
  for (int m = 0; m < NUM_MOTORS; m++) {
    if (jogDir[m] == 0) continue;
    if (nowMs - jogLastCmdMs[m] > JOG_TIMEOUT_MS) {  // watchdog: stop now
      jogDir[m] = 0;
      continue;
    }
    long len = (m < NUM_BELTS) ? rampLenFor(stepDelayUs[m]) : 0;   // belts ramp UP only
    unsigned long d = (len > 0) ? rampDelay(stepDelayUs[m], jogRampN[m], len) : stepDelayUs[m];
    if (nowUs - jogLastStepUs[m] >= d && m >= 2) {   // Z / injector: plain step
      pulse(m);
      pos[m] += jogDir[m];
      jogLastStepUs[m] = nowUs;
    } else if (nowUs - jogLastStepUs[m] >= d) {      // belts (soft-max checked first)
      bool pair = (m == 0 && jogDir[1] != 0);  // N/S/E/W: both belts step together
      if (m == 1 && jogDir[0] != 0) continue;  // (belt 2 is stepped with belt 1)
      int8_t d0 = (m == 0) ? jogDir[0] : 0;
      int8_t d1 = (m == 1 || pair) ? jogDir[1] : 0;
      if (jogStepPastMax(d0, d1)) { stopBeltJogs(); break; }
      if (d0) digitalWrite(STEP_PINS[0], HIGH);
      if (d1) digitalWrite(STEP_PINS[1], HIGH);
      delayMicroseconds(STEP_PULSE_US);
      if (d0) { digitalWrite(STEP_PINS[0], LOW); pos[0] += d0; }
      if (d1) { digitalWrite(STEP_PINS[1], LOW); pos[1] += d1; }
      jogLastStepUs[m] = nowUs;
      if (len > 0 && jogRampN[m] < len) jogRampN[m]++;
      if (pair) { jogLastStepUs[1] = nowUs; jogRampN[1] = jogRampN[0]; }
    }
  }
}

// #####################################################################
// 6. COMMAND PARSER
// #####################################################################
static int motorIndex(char c) {          // '1'..'4' -> 0..3, else -1
  if (c >= '1' && c <= '4') return c - '1';
  return -1;
}

static void handleLine(char* s) {
  while (*s == ' ' && s[1] != '\0') s++;  // allow leading spaces (but keep lone ' ')
  char c = s[0];

  switch (c) {
    case 'I':
      Serial.println(FIRMWARE_ID);
      break;

    case 'X': case ' ':
      stopAll();
      Serial.println("STOPPED");
      printPos("POS ");
      break;

    case 'P':
      printPos("POS ");
      break;

    case 'Z': {
      if (anyMoving()) { Serial.println("ERR:BUSY"); break; }
      bool any = false;
      for (const char* q = s + 1; *q; q++) {            // "Z12" -> only motors 1 and 2
        int m = motorIndex(*q);
        if (m >= 0) { pos[m] = 0; any = true; }
      }
      if (!any) for (int m = 0; m < NUM_MOTORS; m++) pos[m] = 0;   // plain "Z" -> all
      Serial.println("ZEROED");
      break;
    }

    case 'M': {
      char* end;
      long mx = strtol(s + 1, &end, 10);
      if (end == s + 1) { Serial.println("ERR:BAD_M"); break; }
      if (mx <= 0) {                        // "M0" -> off
        softMaxOn = false;
        Serial.println("SOFTMAX:OFF");
        break;
      }
      char* p2 = end;
      long my = strtol(p2, &end, 10);
      if (end == p2 || my <= 0) { Serial.println("ERR:BAD_M"); break; }
      softMaxX = mx; softMaxY = my; softMaxOn = true;
      Serial.print("SOFTMAX:"); Serial.print(softMaxX); Serial.print(','); Serial.println(softMaxY);
      break;
    }

    case 'R': {
      char* end;
      long su = strtol(s + 1, &end, 10);
      char* p2 = end;
      long st = strtol(p2, &end, 10);
      if (end == p2 || su < 0 || st < 0) { Serial.println("ERR:BAD_R"); break; }
      if (st > MAX_RAMP_LEN) st = MAX_RAMP_LEN;
      if (su > (long)MAX_RAMP_START_US) su = MAX_RAMP_START_US;
      rampStartUs = (unsigned long)su;
      rampLen = st;
      Serial.print("RAMP:"); Serial.print(su); Serial.print(','); Serial.println(st);
      break;
    }

    case 'L':
      Serial.print("LIM:"); Serial.print(limX.stable ? 1 : 0);
      Serial.print(','); Serial.println(limY.stable ? 1 : 0);
      break;

    case 'K': {
      if (anyMoving()) { Serial.println("ERR:BUSY"); break; }
      char* p = s + 1;
      char* end;
      long s1 = strtol(p, &end, 10); if (end == p) { Serial.println("ERR:BAD_K"); break; } p = end;
      long s2 = strtol(p, &end, 10); if (end == p) { Serial.println("ERR:BAD_K"); break; } p = end;
      while (*p == ' ') p++;
      char axis = *p++;
      long want = strtol(p, &end, 10); if (end == p) { Serial.println("ERR:BAD_K"); break; } p = end;
      long mx = strtol(p, &end, 10);   if (end == p) { Serial.println("ERR:BAD_K"); break; }
      if ((axis != 'X' && axis != 'Y') || s1 < -1 || s1 > 1 || s2 < -1 || s2 > 1 || mx <= 0) {
        Serial.println("ERR:BAD_K"); break;
      }
      seekSign[0] = (int8_t)s1; seekSign[1] = (int8_t)s2;
      seekLimit = (axis == 'X') ? &limX : &limY;
      seekOther = (axis == 'X') ? &limY : &limX;
      seekOtherStart = seekOther->stable;
      seekWant = (want != 0);
      seekMax = mx; seekDone = 0;
      enableMotors();
      if (seekSign[0]) setDir(0, seekSign[0]);
      if (seekSign[1]) setDir(1, seekSign[1]);
      delayMicroseconds(DIR_SETUP_US);
      seekLastStepUs = micros();
      seekActive = true;
      Serial.println("ACK:K");
      break;
    }

    case 'S': {
      int m = motorIndex(s[1]);
      int lvl = s[2] - '0';
      if (m < 0 || lvl < 1 || lvl > 3) { Serial.println("ERR:BAD_S"); break; }
      stepDelayUs[m] = SPEED_LEVEL_US[lvl - 1];
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
      if (jogDir[m] != sign) {            // start (or reverse): ramp from the slow end
        setDir(m, sign);
        jogDir[m] = sign;
        jogRampN[m] = 0;
        jogLastStepUs[m] = micros();
      }
      jogLastCmdMs[m] = millis();         // refresh watchdog
      break;
    }

    case 'H': {
      int m = motorIndex(s[1]);
      if (m < 0) { Serial.println("ERR:BAD_H"); break; }
      jogDir[m] = 0;                      // stop immediately - no coasting
      jogRampN[m] = 0;
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
      if (softMaxOn && (-(target[0] + target[1]) > 2 * softMaxX ||
                        (target[0] - target[1]) > 2 * softMaxY)) {
        Serial.println("ERR:SOFTLIMIT");   // target is past max X / max Y
        break;
      }
      enableMotors();                     // auto re-energize (no-op if already on)
      startMove(target);
      break;
    }

    case 'V': {                           // closed-loop trim of the running move
      char* end;
      long us = strtol(s + 1, &end, 10);
      if (end == s + 1 || !moveActive || moveRampLen > 0) break;
      char* p2 = end;
      long extra = strtol(p2, &end, 10);  // optional: extra steps to append (lost steps)
      if (end == p2 || extra < 0) extra = 0;
      if (us < (long)MIN_DELAY_US) us = MIN_DELAY_US;
      if (us > (long)MAX_DELAY_US) us = MAX_DELAY_US;
      moveDelayUs = (unsigned long)us;
      bool single = true;                 // extend single-axis moves only (Z or A)
      for (int m = 0; m < NUM_MOTORS; m++) if (m != moveLead && moveDelta[m] != 0) single = false;
      if (extra > 0 && single) {
        moveLeadSteps += extra;
        moveDelta[moveLead] += extra;
      }
      // progress report: steps done / steps total (incl. appended)
      Serial.print("VP "); Serial.print(moveLeadDone); Serial.print(' '); Serial.println(moveLeadSteps);
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
      Serial.print(",lim=");
      Serial.print(limX.stable ? 1 : 0); Serial.print('/'); Serial.print(limY.stable ? 1 : 0);
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

// #####################################################################
// 7. ARDUINO ENTRY POINTS
// #####################################################################
void setup() {
  Serial.begin(SERIAL_BAUD);
  for (int m = 0; m < NUM_MOTORS; m++) stepDelayUs[m] = DEFAULT_DELAY_US[m];
  for (int m = 0; m < NUM_MOTORS; m++) {
    pinMode(STEP_PINS[m], OUTPUT);
    pinMode(DIR_PINS[m], OUTPUT);
    digitalWrite(STEP_PINS[m], LOW);
    digitalWrite(DIR_PINS[m], HIGH);
  }
#if ENABLE_PIN_CONTROL
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);   // drivers enabled
#else
  pinMode(ENABLE_PIN, INPUT);      // never driven: drivers stay on via their pull-down
#endif
  pinMode(X_LIMIT_PIN, INPUT_PULLUP);   // NO switch to GND: pressed = LOW
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  delay(BOOT_SETTLE_MS);
  limX.stable = limX.cand = (digitalRead(X_LIMIT_PIN) == LOW);   // start from the real state
  limY.stable = limY.cand = (digitalRead(Y_LIMIT_PIN) == LOW);

  Serial.println("READY");
  Serial.println(FIRMWARE_ID);
}

void loop() {
  readSerial();

  unsigned long nowUs = micros();
  unsigned long nowMs = millis();
  updateLimit(limX, nowUs);
  updateLimit(limY, nowUs);
  serviceSeek(nowUs);
  serviceMove(nowUs);
  serviceJogs(nowMs, nowUs);

  // Stream positions while moving + one final report when everything stops
  bool moving = anyMoving();
  bool jogging = moving && !moveActive && !seekActive;   // no streaming during moves (smoothness)
  if (jogging && nowMs - lastPosReportMs >= POS_REPORT_MS) {
    printPos("POS ");
    lastPosReportMs = nowMs;
  }
  if (wasMoving && !moving) printPos("POS ");
  wasMoving = moving;
}
