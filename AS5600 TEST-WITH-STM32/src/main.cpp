/*
 * AS5600 diagnostic + rotation counter - STM32 Bluepill (PlatformIO / Arduino)
 *
 * Wiring: SDA -> PB7, SCL -> PB6, VCC -> 3.3V, GND -> GND, DIR -> GND
 *
 * What this does differently from the first version:
 *  - Prints the config registers (ZPOS, MPOS, MANG, CONF) at startup
 *  - Reads BOTH the RAW angle and the scaled ANGLE so you can compare them
 *  - Shows AGC and magnet MAGNITUDE (tells you if the magnet is placed well)
 *  - Counts turns using the RAW angle (always full 0-360 range)
 *  - Serial commands:  'r' = reset config registers to defaults (RAM only)
 *                      'z' = zero the turn counter
 *  NOTE: This code never burns the chip (no permanent changes).
 */

#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDR   0x36

// Register map
#define REG_ZPOS_H    0x01
#define REG_MPOS_H    0x03
#define REG_MANG_H    0x05
#define REG_CONF_H    0x07
#define REG_STATUS    0x0B
#define REG_RAW_H     0x0C
#define REG_ANGLE_H   0x0E
#define REG_AGC       0x1A
#define REG_MAG_H     0x1B

#define MD (1 << 5)   // magnet detected
#define ML (1 << 4)   // magnet too weak
#define MH (1 << 3)   // magnet too strong

int32_t  totalTurns = 0;
uint16_t lastRaw    = 0;
uint32_t lastPrint  = 0;

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool read16(uint8_t reg, uint16_t &val, uint16_t mask = 0x0FFF) {
  uint8_t b[2];
  if (!readRegs(reg, b, 2)) return false;
  val = (((uint16_t)b[0] << 8) | b[1]) & mask;
  return true;
}

bool write16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

void printConfig() {
  uint16_t zpos, mpos, mang, conf;
  if (read16(REG_ZPOS_H, zpos) && read16(REG_MPOS_H, mpos) &&
      read16(REG_MANG_H, mang) && read16(REG_CONF_H, conf, 0x3FFF)) {
    Serial.print("ZPOS="); Serial.print(zpos);
    Serial.print("  MPOS="); Serial.print(mpos);
    Serial.print("  MANG="); Serial.print(mang);
    Serial.print("  CONF=0x"); Serial.println(conf, HEX);
    if (mang != 0 || mpos != 0) {
      Serial.println(">> MANG/MPOS are non-zero: scaled ANGLE is limited to a partial range!");
      Serial.println(">> Send 'r' to reset them, or just use the RAW value.");
    }
  } else {
    Serial.println("Could not read config registers");
  }
}

void printStatus() {
  uint8_t st;
  if (readRegs(REG_STATUS, &st, 1)) {
    Serial.print("Status: 0x"); Serial.print(st, HEX);
    Serial.print(" -> ");
    Serial.print((st & MD) ? "Magnet OK" : "NO MAGNET");
    if (st & ML) Serial.print(" (too weak - move closer)");
    if (st & MH) Serial.print(" (too strong - move away)");
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);   // slower clock = more reliable on long/loose wires

  delay(500);
  Serial.println("\n=== AS5600 diagnostic ===");

  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: AS5600 not found (check wiring / pull-ups)");
  } else {
    Serial.println("AS5600 found at 0x36");
    printStatus();
    printConfig();
  }
  Serial.println("Commands: 'r' reset config, 'z' zero turn counter\n");

  read16(REG_RAW_H, lastRaw);
}

void loop() {
  // Handle serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      bool ok = write16(REG_ZPOS_H, 0) && write16(REG_MPOS_H, 0) &&
                write16(REG_MANG_H, 0) && write16(REG_CONF_H, 0);
      Serial.println(ok ? "Config reset to defaults (RAM only)" : "Config reset FAILED");
      printConfig();
    } else if (c == 'z') {
      totalTurns = 0;
      Serial.println("Turn counter zeroed");
    }
  }

  // Read as often as possible so fast rotation isn't missed
  uint16_t raw;
  if (!read16(REG_RAW_H, raw)) {
    if (millis() - lastPrint > 500) {
      Serial.println("I2C read error");
      lastPrint = millis();
    }
    return;
  }

  // Wrap-around detection using the RAW angle
  int16_t diff = (int16_t)raw - (int16_t)lastRaw;
  if (diff < -2048)     totalTurns++;
  else if (diff > 2048) totalTurns--;
  lastRaw = raw;

  // Print every 100 ms
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    uint16_t scaled = 0, mag = 0;
    uint8_t agc = 0;
    read16(REG_ANGLE_H, scaled);
    read16(REG_MAG_H, mag);
    readRegs(REG_AGC, &agc, 1);

    float deg   = raw * 360.0f / 4096.0f;
    float total = totalTurns * 360.0f + deg;

    Serial.print("RAW: ");     Serial.print(raw);
    Serial.print(" (");        Serial.print(deg, 1);
    Serial.print(" deg) | ANGLE: "); Serial.print(scaled);
    Serial.print(" | Turns: ");  Serial.print(totalTurns);
    Serial.print(" | Total: ");  Serial.print(total, 1);
    Serial.print(" deg | AGC: "); Serial.print(agc);
    Serial.print(" | MAG: ");   Serial.println(mag);
  }
}