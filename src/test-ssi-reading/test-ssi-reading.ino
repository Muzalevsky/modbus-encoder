#include <SPI.h>

// Arduino Nano + W5500 + SSI encoder HAE18U5V12A + Modbus TCP skeleton.

const uint8_t SSI_CS_PIN = 2;
const uint8_t SSI_CLK_PIN = 3;
const uint8_t SSI_DATA_PIN = 4;

const uint8_t DEFAULT_SSI_SIZE_BITS = 18;
const uint8_t DEFAULT_GRAY_TO_BINARY_ENABLED = 1;

uint32_t encoderPosition = 0;
unsigned long lastSsiReadMs = 0;

uint16_t highRegisterWord(uint32_t value) {
  return (uint16_t)(value >> 16);
}

uint16_t lowRegisterWord(uint32_t value) {
  return (uint16_t)(value & 0xFFFF);
}

uint32_t grayToBinary(uint32_t value) {
  for (uint32_t mask = value >> 1; mask != 0; mask >>= 1) {
    value ^= mask;
  }

  return value;
}

uint32_t readSSI() {
  uint8_t bitCount = DEFAULT_SSI_SIZE_BITS;
  uint32_t value = 0;

  static const char t_us = 2;

  noInterrupts();
  digitalWrite(SSI_CS_PIN, LOW);
  delayMicroseconds(t_us);
  for (uint8_t i = 0; i < bitCount; i++) {
    digitalWrite(SSI_CLK_PIN, LOW);
    delayMicroseconds(t_us);
    digitalWrite(SSI_CLK_PIN, HIGH);
    delayMicroseconds(t_us);
    value = (value << 1) | (digitalRead(SSI_DATA_PIN) ? 1UL : 0UL);
  }
  digitalWrite(SSI_CLK_PIN, HIGH);
  digitalWrite(SSI_CS_PIN, HIGH);
  delayMicroseconds(4 * t_us);
  interrupts();

  return value;
}

void setup() {
  pinMode(SSI_CS_PIN, OUTPUT);
  pinMode(SSI_CLK_PIN, OUTPUT);
  pinMode(SSI_DATA_PIN, INPUT);
  digitalWrite(SSI_CS_PIN, HIGH);
  digitalWrite(SSI_CLK_PIN, HIGH);

  Serial.begin(115200);
  delay(500);
}

void loop() {
  if (millis() - lastSsiReadMs >= 1000) {
    lastSsiReadMs = millis();
    uint32_t package = readSSI(); 
    // 6 status bits
    encoderPosition = package >> 6;
    encoderPosition = grayToBinary(encoderPosition);

    Serial.print(F("position: "));
    Serial.println(encoderPosition);
  }
}
