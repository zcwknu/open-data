// =====================================================================
// DigitalPalpation_dataCollections_v0.cpp   (firmware for the Teensy 4.1)
// Host script: DigitalPalpation_v0.py
//
// Reads the pressure sensor through a Nuvoton NAU7802 24-bit ADC on I2C
// (Wire: SDA = pin 18, SCL = pin 19 - same wiring as TEENSY_FLASH_v0) and
// STREAMS every conversion live over the Teensy's native USB.
//
// Why live streaming instead of TEENSY_FLASH_v0's "log to SD, then dump":
//   * The Teensy 4.1's USB is 480 Mbit/s; even 320 SPS is only ~4.5 kB/s,
//     so there's nothing to buffer. USB bulk transfers are already
//     error-checked and retried by the hardware.
//   * The host needs each sample at the moment it happens to line it up with
//     the motor steps. An SD dump would add a transfer gap after every phase.
//   * We KEEP the proven ideas from the old sketch: a start marker (0xAA55)
//     and a CRC16-CCITT on every packet, plus a sequence number so the host
//     can count any dropped packets.
//
// ---------------- Binary data packet (14 bytes, little-endian) ----------
//   uint16  marker   0xAA55          (bytes on the wire: 0x55 0xAA)
//   uint16  seq      0,1,2,... per stream (wraps at 65535)
//   uint32  t_us     microseconds since this stream started
//   int32   reading  raw signed 24-bit ADC value ("bits")
//   uint16  crc16    CRC16-CCITT (XModem) over seq + t_us + reading (10 bytes)
//
// ---------------- Text commands (one per line) --------------------------
//   I          -> "ID:DP_DAQ"
//   R<sps>     sample rate: 10, 20, 40, 80 or 320   -> "RATE:<sps>"
//   G<gain>    PGA gain: 1,2,4,8,16,32,64,128       -> "GAIN:<gain>"
//              (both re-run the AFE calibration and throw away the first
//               conversions while the ADC settles, THEN reply - so when the
//               host gets the reply, the next sample is good.)
//   C          re-run AFE calibration               -> "CAL:OK" / "CAL:FAIL"
//   S          start streaming   -> "STREAM_START" then binary packets
//   X          stop streaming    -> "STREAM_STOP <packets sent>"
//   ?          -> "STATUS:scale=<0/1>,rate=..,gain=..,streaming=.."
// Text replies are plain ASCII lines; they never start with 0x55 0xAA, so the
// host can tell packets and text apart in the same byte stream.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

static NAU7802 scale;
static bool scaleOk = false;

static int curRate = 320;
static int curGain = 1;
static const int SETTLE_DISCARD = 3;          // conversions dropped after a config change

static bool streaming = false;
static uint16_t seq = 0;
static uint32_t t0us = 0;
static uint32_t sentCount = 0;

static char lineBuf[32];
static uint8_t lineLen = 0;

struct __attribute__((packed)) DataPacket {
  uint16_t marker;
  uint16_t seq;
  uint32_t t_us;
  int32_t  reading;
  uint16_t crc16;
};

// CRC16-CCITT (XModem) - identical to TEENSY_FLASH_v0 and the Python side
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

static void sendPacket(int32_t reading, uint32_t tUs) {
  DataPacket p;
  p.marker = 0xAA55;
  p.seq = seq++;
  p.t_us = tUs;
  p.reading = reading;
  p.crc16 = crc16((const uint8_t*)&p.seq, sizeof(p.seq) + sizeof(p.t_us) + sizeof(p.reading));
  Serial.write((const uint8_t*)&p, sizeof(p));
  sentCount++;
}

static void handleLine(const char* s) {
  char c = s[0];
  switch (c) {
    case 'I':
      Serial.println("ID:DP_DAQ");
      break;

    case 'R': {
      int sps = atoi(s + 1);
      if (rateCode(sps) < 0) { Serial.println("ERR:BAD_RATE"); break; }
      curRate = sps;
      applyConfig();
      Serial.print("RATE:"); Serial.println(curRate);
      break;
    }

    case 'G': {
      int g = atoi(s + 1);
      if (gainCode(g) < 0) { Serial.println("ERR:BAD_GAIN"); break; }
      curGain = g;
      applyConfig();
      Serial.print("GAIN:"); Serial.println(curGain);
      break;
    }

    case 'C':
      Serial.println(applyConfig() ? "CAL:OK" : "CAL:FAIL");
      break;

    case 'S':
      if (!scaleOk) { Serial.println("ERR:NO_SCALE"); break; }
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
      Serial.print("STATUS:scale="); Serial.print(scaleOk ? 1 : 0);
      Serial.print(",rate="); Serial.print(curRate);
      Serial.print(",gain="); Serial.print(curGain);
      Serial.print(",streaming="); Serial.println(streaming ? 1 : 0);
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

void setup() {
  Serial.begin(115200);                 // native USB - baud value is ignored
  while (!Serial && millis() < 3000) {}

  Wire.begin();                         // SDA 18 / SCL 19
  Wire.setClock(400000);                // fast-mode I2C: shorter polling + reads

  scaleOk = scale.begin();
  if (scaleOk) {
    applyConfig();
    Serial.println("READY");
  } else {
    Serial.println("ERR:NO_SCALE (NAU7802 not detected on I2C)");
  }
  Serial.println("ID:DP_DAQ");
}

void loop() {
  readSerial();
  if (streaming && scale.available()) {
    uint32_t t = micros() - t0us;       // timestamp as soon as the conversion is ready
    sendPacket(scale.getReading(), t);
  }
}
