#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

#define LOG_DURATION_MS 5000
#define LOG_FILENAME "log.bin"

NAU7802 scale;
File logFile;

uint32_t startTime;
bool logging = false;
uint8_t packetID = 0;

// CRC16-CCITT (XModem) function
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// Packet format: 13 bytes
struct __attribute__((packed)) LogPacket {
  uint16_t start_marker;  // 0xAA55
  uint8_t  pkt_id;
  uint32_t time_ms;
  int32_t  reading;
  uint16_t crc16;         // over pkt_id + time_ms + reading
};

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("Teensy ready. Send 'p' to log.");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD FAIL - Check card");
    while (1);
  }
  Serial.println("SD OK");

  Wire.begin();

  if (!scale.begin()) {
    Serial.println("Scale not detected");
  } else {
    scale.setGain(NAU7802_GAIN_1);
    scale.setSampleRate(NAU7802_SPS_320);
    scale.calibrateAFE();
    Serial.println("Scale ready");
  }
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'p') startLogging();
    else if (cmd == 'I' && !logging) Serial.println("ID:TEENSY_DAQ");
  }

  if (logging && scale.available()) {
    LogPacket p;
    p.start_marker = 0xAA55;
    p.pkt_id = packetID++;
    p.time_ms = millis() - startTime;
    p.reading = scale.getReading();

    // Compute CRC16 over pkt_id + time_ms + reading
    uint8_t* crcData = (uint8_t*)&p.pkt_id;
    p.crc16 = crc16(crcData, sizeof(p.pkt_id) + sizeof(p.time_ms) + sizeof(p.reading));

    // Write to SD card
    logFile.write((uint8_t*)&p, sizeof(p));
  }

  if (logging && millis() - startTime >= LOG_DURATION_MS) {
    stopLogging();
  }
}

void startLogging() {
  SD.remove(LOG_FILENAME);
  logFile = SD.open(LOG_FILENAME, FILE_WRITE);
  if (!logFile) {
    Serial.println("ERROR: Cannot create log file");
    return;
  }

  startTime = millis();
  packetID = 0;
  logging = true;

  Serial.println("LOG_START");
}

void stopLogging() {
  logging = false;

  logFile.flush();
  logFile.close();

  Serial.println("LOG_DONE");
  sendBinaryFile();
}

void sendBinaryFile() {
  File f = SD.open(LOG_FILENAME, FILE_READ);
  if (!f) {
    Serial.println("ERROR: log file not found");
    return;
  }

  Serial.println("BIN_BEGIN");

  const size_t BUF_SIZE = 64;
  uint8_t buf[BUF_SIZE];
  size_t totalSent = 0;

  while (f.available()) {
    size_t n = f.read(buf, BUF_SIZE);
    Serial.write(buf, n);
    totalSent += n;
    delayMicroseconds(500); // prevent USB buffer overflow
  }

  f.close();
  Serial.flush();
  delay(20);

  Serial.println("BIN_END");
  Serial.print("Sent ");
  Serial.print(totalSent);
  Serial.println(" bytes");
}
