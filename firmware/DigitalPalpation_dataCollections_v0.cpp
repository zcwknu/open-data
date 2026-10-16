// =====================================================================
// DigitalPalpation_dataCollections_v0.cpp   firmware for the Teensy 4.1 (DAQ)
//                                           NAU7802 pressure ADC + 2 x AS5600 encoders
// Host script: DigitalPalpation_v0.py       (protocol version: DP_DAQ_V3)
//
// HOW THIS FILE IS LAID OUT
//   1. USER SETTINGS ...... encoder mode, pins / buses, I2C speeds, ADC defaults
//   2. REFERENCE NOTES .... what the firmware does, packet format, command list
//   3. INTERNAL STATE ..... variables the code uses at run time (don't edit)
//   4. ENCODERS ........... AS5600 reads (I2C or analog)
//   5. PRESSURE ADC + PACKETS
//   6. COMMAND PARSER ..... one case per serial command
//   7. ARDUINO ENTRY POINTS (setup / loop)
// Only section 1 should need editing. If you change something that the Python
// script also knows about, the matching Python setting is named in the comment.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"


// #####################################################################
// 1. USER SETTINGS
// #####################################################################

// ---- 1a. Identity / serial link -------------------------------------
#define FIRMWARE_ID   "ID:DP_DAQ_V3"     // Python: DAQ_ID (the host matches "ID:DP_DAQ_V")
#define SERIAL_BAUD   115200             // native USB: the number is ignored (always full speed)
#define USB_WAIT_MS   3000               // at boot, wait up to this long for the host to open USB
#define LINE_BUF_LEN  32                 // longest command line accepted (chars)

// ---- 1b. Encoder read mode ---------------------------------------------
//   ENC_MODE_I2C    : each AS5600 on its OWN I2C bus (they share the fixed address
//                     0x36), 12-bit RAW ANGLE register, 0..4095 = one turn.
//   ENC_MODE_ANALOG : the old wiring, AS5600 OUT pin -> Teensy analog pin.
// The host asks which mode is running ("?" -> enc=i2c|analog) and labels the CSV.
#define ENC_MODE_ANALOG 0
#define ENC_MODE_I2C    1
#define ENCODER_MODE    ENC_MODE_I2C     // <- ENC_MODE_ANALOG for the old wiring

// ---- 1c. I2C encoders (ENC_MODE_I2C) -------------------------------------
//   AS5600s are powered from 3.3 V ONLY - the Teensy 4.1 is not 5 V tolerant.
//   A motor (injector) encoder -> Wire2: SDA 25, SCL 24
//   Z motor (stage) encoder    -> Wire1: SDA 17, SCL 16
#define ENC_A_BUS        Wire2
#define ENC_Z_BUS        Wire1
#define ENC_I2C_CLOCK    100000          // Hz. 100 kHz for cables > 50 cm; 400000 if short
static const int ENC_READ_TRIES   = 2;   // attempts per reading (1 retry) before giving up
static const int ENC_REINIT_AFTER = 20;  // consecutive failed readings -> re-initialise that bus

// ---- 1d. Analog encoders (ENC_MODE_ANALOG) ---------------------------------
static const int ENC_A_PIN = 15;         // A1 - injector (A motor) AS5600 OUT
static const int ENC_Z_PIN = 16;         // A2 - stage (Z motor) AS5600 OUT
static const int ANALOG_BITS = 12;       // 0..4095 over 0..3.3 V (Python: ENCODER_ADC_MAX)
static const int ANALOG_AVERAGING = 4;   // hardware averaging per read (a few us each)

// ---- 1e. Pressure ADC (NAU7802 on Wire: SDA 18, SCL 19) ---------------------
#define ADC_I2C_CLOCK    400000          // Hz. fast-mode: shorter polling + reads
static const int DEFAULT_RATE_SPS = 320; // start-up sample rate (host sets its own with "R")
static const int DEFAULT_GAIN = 1;       // start-up PGA gain (host sets its own with "G")
static const int SETTLE_DISCARD = 3;     // conversions thrown away after a rate / gain change


// #####################################################################
// 2. REFERENCE NOTES (nothing to edit here)
// #####################################################################
//
// Reads the pressure sensor through the NAU7802 24-bit ADC and STREAMS every
// conversion live over the Teensy's native USB, together with both AS5600
// encoders read at the same instant.
//
// Why live streaming instead of "log to SD, then dump":
//   * USB is 480 Mbit/s; even 320 SPS is only ~6 kB/s - nothing to buffer, and
//     USB bulk transfers are already error-checked and retried by the hardware.
//   * The host needs each sample when it happens to line it up with the motor
//     steps. An SD dump would add a transfer gap after every phase.
//   * Kept from the old sketch: a start marker (0xAA55), a CRC16-CCITT on every
//     packet, and a sequence number so the host can count dropped packets.
//
// ---------------- Binary data packet (18 bytes, little-endian) ----------
//   uint16  marker   0xAA55          (bytes on the wire: 0x55 0xAA)
//   uint16  seq      0,1,2,... per stream (wraps at 65535)
//   uint32  t_us     microseconds since this stream started
//   int32   reading  raw signed 24-bit ADC value ("bits")
//   uint16  encA     A-motor AS5600: I2C angle 0..4095 (0xFFFF = read failed)
//                    or analog ADC counts (ENC_A_PIN)
//   uint16  encZ     Z-motor AS5600: same (ENC_Z_BUS / ENC_Z_PIN)
//   uint16  crc16    CRC16-CCITT (XModem) over seq .. encZ (14 bytes)
//   Python: DAQ_PACKET = struct "<HHIiHHH", DAQ_MARKER, ENCODER_BAD
//
// ---------------- Text commands (one per line) --------------------------
//   I          -> FIRMWARE_ID
//   R<sps>     sample rate: 10, 20, 40, 80 or 320   -> "RATE:<sps>"
//   G<gain>    PGA gain: 1,2,4,8,16,32,64,128       -> "GAIN:<gain>"
//              (both re-run the AFE calibration and throw away SETTLE_DISCARD
//               conversions, THEN reply - so the next sample is good.)
//   C          re-run AFE calibration               -> "CAL:OK" / "CAL:FAIL"
//   S          start streaming   -> "STREAM_START" then binary packets
//              ("ERR:NO_SCALE" if the NAU7802 still isn't found - it is looked for again
//               by S, R, G, C and ?, not only at boot)
//   X          stop streaming    -> "STREAM_STOP <packets sent>"
//   A          read both encoders now        -> "ENC:<A counts>,<Z counts>"
//   M          magnet / link check (I2C mode) -> "MAG:A=<ok|MISSING>,md=..,ml=..,mh=..,agc=..;Z=..."
//              md = magnet detected, ml = too weak (gap too big), mh = too strong
//   ?          -> "STATUS:scale=<0/1>,rate=..,gain=..,streaming=..,enc=<i2c|analog>,enc_err=<A>/<Z>"
// Text replies are plain ASCII lines; they never start with 0x55 0xAA, so the
// host can tell packets and text apart in the same byte stream.


// #####################################################################
// 3. INTERNAL STATE (run-time variables - no need to edit)
// #####################################################################
static const uint16_t PACKET_MARKER = 0xAA55;
static const uint16_t ENC_BAD = 0xFFFF;  // sent when an I2C read fails
static const uint8_t AS5600_ADDR = 0x36;                                  // fixed by the chip
static const uint8_t AS5600_STATUS = 0x0B, AS5600_RAW_ANGLE = 0x0C, AS5600_AGC = 0x1A;  // registers

static NAU7802 scale;
static bool scaleOk = false;
static int curRate = DEFAULT_RATE_SPS;
static int curGain = DEFAULT_GAIN;

static uint32_t encErr[2] = {0, 0};      // failed reads since boot: [A, Z]

static bool streaming = false;
static uint16_t seq = 0;
static uint32_t t0us = 0;
static uint32_t sentCount = 0;

static char lineBuf[LINE_BUF_LEN];
static uint8_t lineLen = 0;

struct __attribute__((packed)) DataPacket {
  uint16_t marker;
  uint16_t seq;
  uint32_t t_us;
  int32_t  reading;
  uint16_t encA;
  uint16_t encZ;
  uint16_t crc16;
};


// #####################################################################
// 4. ENCODERS
// #####################################################################
#if ENCODER_MODE == ENC_MODE_I2C
static int encFailRun[2] = {0, 0};       // consecutive failures per bus
static TwoWire* const encBus[2] = {&ENC_A_BUS, &ENC_Z_BUS};   // [A, Z]

static void encBusBegin(int i) {
  encBus[i]->begin();
  encBus[i]->setClock(ENC_I2C_CLOCK);
}

static bool as5600Read(int i, uint8_t reg, uint8_t* buf, uint8_t n) {
  TwoWire* w = encBus[i];
  w->beginTransmission(AS5600_ADDR);
  w->write(reg);
  if (w->endTransmission(false) != 0) return false;     // repeated start
  if (w->requestFrom(AS5600_ADDR, n) != n) return false;
  for (uint8_t k = 0; k < n; k++) buf[k] = (uint8_t)w->read();
  return true;
}

// 12-bit raw angle (0..4095 = one turn), or ENC_BAD. Retries; a bus that keeps
// failing (cable glitch, hot-plug) is re-initialised.
static uint16_t readEncoder(int i) {
  uint8_t b[2];
  for (int attempt = 0; attempt < ENC_READ_TRIES; attempt++) {
    if (as5600Read(i, AS5600_RAW_ANGLE, b, 2)) {
      encFailRun[i] = 0;
      return (uint16_t)(((b[0] & 0x0F) << 8) | b[1]);
    }
  }
  encErr[i]++;
  if (++encFailRun[i] >= ENC_REINIT_AFTER) { encBusBegin(i); encFailRun[i] = 0; }
  return ENC_BAD;
}

static void printMagnet(const char* name, int i) {
  uint8_t st, agc;
  Serial.print(name);
  if (!as5600Read(i, AS5600_STATUS, &st, 1) || !as5600Read(i, AS5600_AGC, &agc, 1)) {
    Serial.print("=MISSING");
    return;
  }
  Serial.print("=ok,md="); Serial.print((st >> 5) & 1);
  Serial.print(",ml="); Serial.print((st >> 4) & 1);
  Serial.print(",mh="); Serial.print((st >> 3) & 1);
  Serial.print(",agc="); Serial.print(agc);
}

static void printMagnetLine() {
  Serial.print("MAG:"); printMagnet("A", 0); Serial.print(';'); printMagnet("Z", 1); Serial.println();
}
#else
static uint16_t readEncoder(int i) { return (uint16_t)analogRead(i == 0 ? ENC_A_PIN : ENC_Z_PIN); }
#endif


// #####################################################################
// 5. PRESSURE ADC + PACKETS
// #####################################################################
// CRC16-CCITT (XModem) - identical to the Python side (crc16)
static uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static int gainCode(int g) {
  switch (g) {
    case 1:   return NAU7802_GAIN_1;
    case 2:   return NAU7802_GAIN_2;
    case 4:   return NAU7802_GAIN_4;
    case 8:   return NAU7802_GAIN_8;
    case 16:  return NAU7802_GAIN_16;
    case 32:  return NAU7802_GAIN_32;
    case 64:  return NAU7802_GAIN_64;
    case 128: return NAU7802_GAIN_128;
  }
  return -1;
}

static int rateCode(int sps) {
  switch (sps) {
    case 10:  return NAU7802_SPS_10;
    case 20:  return NAU7802_SPS_20;
    case 40:  return NAU7802_SPS_40;
    case 80:  return NAU7802_SPS_80;
    case 320: return NAU7802_SPS_320;
  }
  return -1;
}

// Apply gain + rate, recalibrate, and wait out the settling conversions.
static bool applyConfig() {
  if (!scaleOk) return false;
  scale.setGain(gainCode(curGain));
  scale.setSampleRate(rateCode(curRate));
  bool ok = scale.calibrateAFE();
  // Drop the first few conversions after the change (they're unsettled).
  // Timeout = 3 conversion periods each at the current rate, plus margin.
  uint32_t perConvMs = 1000UL / (uint32_t)curRate + 5;
  for (int i = 0; i < SETTLE_DISCARD; i++) {
    uint32_t start = millis();
    while (!scale.available()) {
      if (millis() - start > perConvMs * 3) break;
    }
    if (scale.available()) scale.getReading();
  }
  return ok;
}

// The NAU7802 is normally found once, at boot. If it wasn't (powered after the Teensy,
// loose wire fixed since), look for it again before anything that needs it - so fixing
// the wiring doesn't need a Teensy reset.
static bool ensureScale() {
  if (scaleOk) return true;
  Wire.begin();
  Wire.setClock(ADC_I2C_CLOCK);
  scaleOk = scale.begin();
  if (scaleOk) applyConfig();
  return scaleOk;
}

static void sendPacket(int32_t reading, uint32_t tUs, uint16_t encA, uint16_t encZ) {
  DataPacket p;
  p.marker = PACKET_MARKER;
  p.seq = seq++;
  p.t_us = tUs;
  p.reading = reading;
  p.encA = encA;
  p.encZ = encZ;
  p.crc16 = crc16((const uint8_t*)&p.seq, sizeof(DataPacket) - 2 * sizeof(uint16_t));   // seq..encZ
  Serial.write((const uint8_t*)&p, sizeof(p));
  sentCount++;
}


// #####################################################################
// 6. COMMAND PARSER
// #####################################################################
static void handleLine(const char* s) {
  char c = s[0];
  switch (c) {
    case 'I':
      Serial.println(FIRMWARE_ID);
      break;

    case 'A':
      Serial.print("ENC:"); Serial.print(readEncoder(0));
      Serial.print(','); Serial.println(readEncoder(1));
      break;

    case 'M':
#if ENCODER_MODE == ENC_MODE_I2C
      printMagnetLine();
#else
      Serial.println("MAG:analog mode (no magnet status)");
#endif
      break;

    case 'R': {
      int sps = atoi(s + 1);
      if (rateCode(sps) < 0) { Serial.println("ERR:BAD_RATE"); break; }
      curRate = sps;
      ensureScale();
      applyConfig();
      Serial.print("RATE:"); Serial.println(curRate);
      break;
    }

    case 'G': {
      int g = atoi(s + 1);
      if (gainCode(g) < 0) { Serial.println("ERR:BAD_GAIN"); break; }
      curGain = g;
      ensureScale();
      applyConfig();
      Serial.print("GAIN:"); Serial.println(curGain);
      break;
    }

    case 'C':
      Serial.println(ensureScale() && applyConfig() ? "CAL:OK" : "CAL:FAIL");
      break;

    case 'S':
      if (!ensureScale()) { Serial.println("ERR:NO_SCALE"); break; }
      seq = 0;
      sentCount = 0;
      Serial.println("STREAM_START");
      t0us = micros();
      streaming = true;
      break;

    case 'X':
      streaming = false;
      Serial.print("STREAM_STOP "); Serial.println(sentCount);
      break;

    case '?':
      if (!streaming) ensureScale();      // re-check a missing NAU7802 (host asks every ~2 s)
      Serial.print("STATUS:scale="); Serial.print(scaleOk ? 1 : 0);
      Serial.print(",rate="); Serial.print(curRate);
      Serial.print(",gain="); Serial.print(curGain);
      Serial.print(",streaming="); Serial.print(streaming ? 1 : 0);
      Serial.print(",enc="); Serial.print(ENCODER_MODE == ENC_MODE_I2C ? "i2c" : "analog");
      Serial.print(",enc_err="); Serial.print(encErr[0]); Serial.print('/'); Serial.println(encErr[1]);
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
  while (!Serial && millis() < USB_WAIT_MS) {}

#if ENCODER_MODE == ENC_MODE_I2C
  encBusBegin(0);                       // A motor encoder bus
  encBusBegin(1);                       // Z motor encoder bus
#else
  analogReadResolution(ANALOG_BITS);
  analogReadAveraging(ANALOG_AVERAGING);
#endif

  Wire.begin();                         // NAU7802: SDA 18 / SCL 19
  Wire.setClock(ADC_I2C_CLOCK);

  scaleOk = scale.begin();
  if (scaleOk) {
    applyConfig();
    Serial.println("READY");
  } else {
    Serial.println("ERR:NO_SCALE (NAU7802 not detected on I2C)");
  }
#if ENCODER_MODE == ENC_MODE_I2C
  printMagnetLine();
#endif
  Serial.println(FIRMWARE_ID);
}

void loop() {
  readSerial();
  if (streaming && scale.available()) {
    uint32_t t = micros() - t0us;       // timestamp as soon as the conversion is ready
    int32_t reading = scale.getReading();
    uint16_t encA = readEncoder(0);     // encoders sampled right with it
    uint16_t encZ = readEncoder(1);
    sendPacket(reading, t, encA, encZ);
  }
}
