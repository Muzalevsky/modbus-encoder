#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <SPI.h>

// Arduino Nano + W5500 + SSI encoder + Modbus TCP skeleton.

const uint32_t SETTINGS_MAGIC = 0x4D424553UL; // "MBES"
const uint16_t MODBUS_PORT = 502;

const uint8_t W5500_CS_PIN = 10;
const uint8_t SSI_CLK_PIN = 3;
const uint8_t SSI_DATA_PIN = 2;

const uint8_t DEFAULT_IP[4] = {192, 168, 1, 13};
const uint16_t DEFAULT_MARKS_PER_REV = 4096;
const uint8_t DEFAULT_SSI_SIZE_BYTES = 4;
const uint32_t DEFAULT_SERIAL_NUMBER = 1;
const uint16_t FIRMWARE_VERSION_MAJOR = 1;
const uint16_t FIRMWARE_VERSION_MINOR = 0;

// Holding register map. Multi-word values use high word first.
const uint16_t REG_IP_0 = 0;               // RW: 192
const uint16_t REG_IP_1 = 1;               // RW: 168
const uint16_t REG_IP_2 = 2;               // RW: 1
const uint16_t REG_IP_3 = 3;               // RW: 13
const uint16_t REG_MARKS_PER_REV = 4;      // RW
const uint16_t REG_SSI_SIZE_BYTES = 5;     // RW
const uint16_t REG_SERIAL_HIGH = 6;        // RO
const uint16_t REG_SERIAL_LOW = 7;         // RO
const uint16_t REG_FW_VERSION_MAJOR = 8;   // RO
const uint16_t REG_FW_VERSION_MINOR = 9;   // RO
const uint16_t REG_POSITION_HIGH = 10;     // RO
const uint16_t REG_POSITION_LOW = 11;      // RO
const uint16_t REG_SAVE_SETTINGS = 12;     // WO: write 1 to save RW settings to EEPROM
const uint16_t HOLDING_REG_COUNT = 13;

struct Settings {
  uint32_t magic;
  uint8_t ip[4];
  uint16_t marksPerRev;
  uint8_t ssiSizeBytes;
  uint32_t serialNumber;
  uint16_t firmwareMajor;
  uint16_t firmwareMinor;
};

Settings settings;
EthernetServer ethernetServer(MODBUS_PORT);
ModbusTCPServer modbusServer;
uint32_t encoderPosition = 0;
unsigned long lastSsiReadMs = 0;

uint16_t highRegisterWord(uint32_t value) {
  return (uint16_t)(value >> 16);
}

uint16_t lowRegisterWord(uint32_t value) {
  return (uint16_t)(value & 0xFFFF);
}

void setDefaultSettings() {
  settings.magic = SETTINGS_MAGIC;
  memcpy(settings.ip, DEFAULT_IP, sizeof(settings.ip));
  settings.marksPerRev = DEFAULT_MARKS_PER_REV;
  settings.ssiSizeBytes = DEFAULT_SSI_SIZE_BYTES;
  settings.serialNumber = DEFAULT_SERIAL_NUMBER;
  settings.firmwareMajor = FIRMWARE_VERSION_MAJOR;
  settings.firmwareMinor = FIRMWARE_VERSION_MINOR;
}

void loadSettings() {
  EEPROM.get(0, settings);

  if (settings.magic != SETTINGS_MAGIC || settings.ssiSizeBytes == 0 || settings.ssiSizeBytes > 4) {
    setDefaultSettings();
    EEPROM.put(0, settings);
  }
}

void saveSettings() {
  settings.magic = SETTINGS_MAGIC;
  settings.firmwareMajor = FIRMWARE_VERSION_MAJOR;
  settings.firmwareMinor = FIRMWARE_VERSION_MINOR;
  EEPROM.put(0, settings);
}

IPAddress configuredIp() {
  return IPAddress(settings.ip[0], settings.ip[1], settings.ip[2], settings.ip[3]);
}

void printDiagnostics() {
  Serial.println(F("Modbus SSI encoder boot"));
  Serial.print(F("IP: "));
  Serial.println(configuredIp());
  Serial.print(F("Marks per revolution: "));
  Serial.println(settings.marksPerRev);
  Serial.print(F("SSI frame size, bytes: "));
  Serial.println(settings.ssiSizeBytes);
  Serial.print(F("Serial number: "));
  Serial.println(settings.serialNumber);
  Serial.print(F("Firmware version: "));
  Serial.print(settings.firmwareMajor);
  Serial.print(F("."));
  Serial.println(settings.firmwareMinor);
}

void writeStaticRegisters() {
  modbusServer.holdingRegisterWrite(REG_IP_0, settings.ip[0]);
  modbusServer.holdingRegisterWrite(REG_IP_1, settings.ip[1]);
  modbusServer.holdingRegisterWrite(REG_IP_2, settings.ip[2]);
  modbusServer.holdingRegisterWrite(REG_IP_3, settings.ip[3]);
  modbusServer.holdingRegisterWrite(REG_MARKS_PER_REV, settings.marksPerRev);
  modbusServer.holdingRegisterWrite(REG_SSI_SIZE_BYTES, settings.ssiSizeBytes);
  modbusServer.holdingRegisterWrite(REG_SERIAL_HIGH, highRegisterWord(settings.serialNumber));
  modbusServer.holdingRegisterWrite(REG_SERIAL_LOW, lowRegisterWord(settings.serialNumber));
  modbusServer.holdingRegisterWrite(REG_FW_VERSION_MAJOR, settings.firmwareMajor);
  modbusServer.holdingRegisterWrite(REG_FW_VERSION_MINOR, settings.firmwareMinor);
  modbusServer.holdingRegisterWrite(REG_SAVE_SETTINGS, 0);
}

void writeReadOnlyRegisters() {
  modbusServer.holdingRegisterWrite(REG_SERIAL_HIGH, highRegisterWord(settings.serialNumber));
  modbusServer.holdingRegisterWrite(REG_SERIAL_LOW, lowRegisterWord(settings.serialNumber));
  modbusServer.holdingRegisterWrite(REG_FW_VERSION_MAJOR, settings.firmwareMajor);
  modbusServer.holdingRegisterWrite(REG_FW_VERSION_MINOR, settings.firmwareMinor);
}

void updatePositionRegisters() {
  modbusServer.holdingRegisterWrite(REG_POSITION_HIGH, highRegisterWord(encoderPosition));
  modbusServer.holdingRegisterWrite(REG_POSITION_LOW, lowRegisterWord(encoderPosition));
}

void applyWritableRegisters() {
  uint8_t newIp[4];
  for (uint8_t i = 0; i < 4; i++) {
    int value = modbusServer.holdingRegisterRead(REG_IP_0 + i);
    if (value < 0 || value > 255) {
      return;
    }
    newIp[i] = (uint8_t)value;
  }

  int newMarks = modbusServer.holdingRegisterRead(REG_MARKS_PER_REV);
  int newSsiSize = modbusServer.holdingRegisterRead(REG_SSI_SIZE_BYTES);
  if (newMarks <= 0 || newSsiSize <= 0 || newSsiSize > 4) {
    writeStaticRegisters();
    return;
  }

  memcpy(settings.ip, newIp, sizeof(settings.ip));
  settings.marksPerRev = (uint16_t)newMarks;
  settings.ssiSizeBytes = (uint8_t)newSsiSize;

  if (modbusServer.holdingRegisterRead(REG_SAVE_SETTINGS) == 1) {
    saveSettings();
    modbusServer.holdingRegisterWrite(REG_SAVE_SETTINGS, 0);
    Serial.println(F("Settings saved to EEPROM. Reboot to apply changed IP address."));
  }
}

uint32_t readSSI() {
  uint8_t bitCount = settings.ssiSizeBytes * 8;
  uint32_t value = 0;

  noInterrupts();
  for (uint8_t i = 0; i < bitCount; i++) {
    digitalWrite(SSI_CLK_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SSI_CLK_PIN, HIGH);
    delayMicroseconds(2);
    value = (value << 1) | (digitalRead(SSI_DATA_PIN) ? 1UL : 0UL);
  }
  digitalWrite(SSI_CLK_PIN, HIGH);
  interrupts();

  return value;
}

void setupEthernetAndModbus() {
  byte mac[] = {0x02, 0x4D, 0x42, 0x45, 0x53, 0x01};

  Ethernet.init(W5500_CS_PIN);
  Ethernet.begin(mac, configuredIp());
  ethernetServer.begin();

  if (!modbusServer.begin()) {
    Serial.println(F("Failed to start Modbus TCP server"));
    while (true) {
      delay(1000);
    }
  }

  modbusServer.configureHoldingRegisters(0, HOLDING_REG_COUNT);
  writeStaticRegisters();
  updatePositionRegisters();
}

void setup() {
  pinMode(SSI_CLK_PIN, OUTPUT);
  pinMode(SSI_DATA_PIN, INPUT);
  digitalWrite(SSI_CLK_PIN, HIGH);

  Serial.begin(115200);
  delay(500);

  loadSettings();
  printDiagnostics();
  setupEthernetAndModbus();
}

void loop() {
  EthernetClient client = ethernetServer.available();
  if (client) {
    modbusServer.accept(client);
  }

  modbusServer.poll();
  applyWritableRegisters();
  writeReadOnlyRegisters();

  if (millis() - lastSsiReadMs >= 10) {
    lastSsiReadMs = millis();
    encoderPosition = readSSI();
    updatePositionRegisters();
  }
}
