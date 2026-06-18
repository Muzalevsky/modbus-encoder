#include <ModbusTCPServer.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <SPI.h>

// Arduino Nano + W5500 + SSI encoder + Modbus TCP skeleton.

uint32_t const SETTINGS_MAGIC = 0x4D424531UL; // "MBE1"
uint16_t const MODBUS_PORT = 502;

uint8_t const W5500_CS_PIN = 10;
uint8_t const SSI_CS_PIN = 2;
uint8_t const SSI_CLK_PIN = 3;
uint8_t const SSI_DATA_PIN = 4;

uint8_t const DEFAULT_IP[4] = {192, 168, 1, 95};
uint16_t const DEFAULT_MARKS_PER_REV = 4096;
uint8_t const DEFAULT_SSI_SIZE_BITS = 18;
uint8_t const DEFAULT_GRAY_TO_BINARY_ENABLED = 0;
uint8_t const DEFAULT_POSITION_LSB_BIT = 6;
uint8_t const DEFAULT_POSITION_MSB_BIT = 17;
uint32_t const DEFAULT_SERIAL_NUMBER = 1;
uint16_t const FIRMWARE_VERSION_MAJOR = 1;
uint16_t const FIRMWARE_VERSION_MINOR = 0;

// Holding register map. Multi-word values use high word first.
uint16_t const REG_IP_0 = 0;              // RW: 192
uint16_t const REG_IP_1 = 1;              // RW: 168
uint16_t const REG_IP_2 = 2;              // RW: 1
uint16_t const REG_IP_3 = 3;              // RW: 13
uint16_t const REG_MARKS_PER_REV = 4;     // RW
uint16_t const REG_SSI_SIZE_BITS = 5;     // RW
uint16_t const REG_SERIAL_HIGH = 6;       // RO
uint16_t const REG_SERIAL_LOW = 7;        // RO
uint16_t const REG_FW_VERSION_MAJOR = 8;  // RO
uint16_t const REG_FW_VERSION_MINOR = 9;  // RO
uint16_t const REG_POSITION_HIGH = 10;    // RO
uint16_t const REG_POSITION_LOW = 11;     // RO
uint16_t const REG_SAVE_SETTINGS = 12;    // WO: write 1 to save RW settings to EEPROM
uint16_t const REG_GRAY_TO_BINARY = 13;   // RW: 0 = raw SSI, 1 = Gray to binary
uint16_t const REG_POSITION_LSB_BIT = 14; // RW: bit 0 is frame LSB
uint16_t const REG_POSITION_MSB_BIT = 15; // RW: inclusive
uint16_t const HOLDING_REG_COUNT = 16;

struct Settings
{
    uint32_t magic;
    uint8_t ip[4];
    uint16_t marksPerRev;
    uint8_t ssiSizeBits;
    uint8_t grayToBinaryEnabled;
    uint8_t positionLsbBit;
    uint8_t positionMsbBit;
    uint32_t serialNumber;
    uint16_t firmwareMajor;
    uint16_t firmwareMinor;
};

Settings settings;
EthernetServer ethernetServer(MODBUS_PORT);
EthernetClient modbusClient;
ModbusTCPServer modbusServer;
uint32_t encoderPosition = 0;
unsigned long lastSsiReadMs = 0;

uint16_t highRegisterWord(uint32_t value)
{
    return (uint16_t)(value >> 16);
}

uint16_t lowRegisterWord(uint32_t value)
{
    return (uint16_t)(value & 0xFFFF);
}

void setDefaultSettings()
{
    settings.magic = SETTINGS_MAGIC;
    memcpy(settings.ip, DEFAULT_IP, sizeof(settings.ip));
    settings.marksPerRev = DEFAULT_MARKS_PER_REV;
    settings.ssiSizeBits = DEFAULT_SSI_SIZE_BITS;
    settings.grayToBinaryEnabled = DEFAULT_GRAY_TO_BINARY_ENABLED;
    settings.positionLsbBit = DEFAULT_POSITION_LSB_BIT;
    settings.positionMsbBit = DEFAULT_POSITION_MSB_BIT;
    settings.serialNumber = DEFAULT_SERIAL_NUMBER;
    settings.firmwareMajor = FIRMWARE_VERSION_MAJOR;
    settings.firmwareMinor = FIRMWARE_VERSION_MINOR;
}

void loadSettings()
{
    EEPROM.get(0, settings);

    if (settings.magic != SETTINGS_MAGIC || settings.ssiSizeBits == 0 || settings.ssiSizeBits > 32
        || settings.grayToBinaryEnabled > 1 || settings.positionLsbBit > settings.positionMsbBit
        || settings.positionMsbBit >= settings.ssiSizeBits)
    {
        setDefaultSettings();
        EEPROM.put(0, settings);
    }
}

void saveSettings()
{
    settings.magic = SETTINGS_MAGIC;
    settings.firmwareMajor = FIRMWARE_VERSION_MAJOR;
    settings.firmwareMinor = FIRMWARE_VERSION_MINOR;
    EEPROM.put(0, settings);
}

IPAddress configuredIp()
{
    return IPAddress(settings.ip[0], settings.ip[1], settings.ip[2], settings.ip[3]);
}

void printDiagnostics()
{
    Serial.println(F("Modbus SSI encoder boot"));
    Serial.print(F("IP: "));
    Serial.println(configuredIp());
    Serial.print(F("Marks per revolution: "));
    Serial.println(settings.marksPerRev);
    Serial.print(F("SSI frame size, bits: "));
    Serial.println(settings.ssiSizeBits);
    Serial.print(F("Gray to binary: "));
    Serial.println(settings.grayToBinaryEnabled ? F("enabled") : F("disabled"));
    Serial.print(F("Position bit range: "));
    Serial.print(settings.positionLsbBit);
    Serial.print(F(".."));
    Serial.println(settings.positionMsbBit);
    Serial.print(F("Serial number: "));
    Serial.println(settings.serialNumber);
    Serial.print(F("Firmware version: "));
    Serial.print(settings.firmwareMajor);
    Serial.print(F("."));
    Serial.println(settings.firmwareMinor);
}

void writeStaticRegisters()
{
    modbusServer.holdingRegisterWrite(REG_IP_0, settings.ip[0]);
    modbusServer.holdingRegisterWrite(REG_IP_1, settings.ip[1]);
    modbusServer.holdingRegisterWrite(REG_IP_2, settings.ip[2]);
    modbusServer.holdingRegisterWrite(REG_IP_3, settings.ip[3]);
    modbusServer.holdingRegisterWrite(REG_MARKS_PER_REV, settings.marksPerRev);
    modbusServer.holdingRegisterWrite(REG_SSI_SIZE_BITS, settings.ssiSizeBits);
    modbusServer.holdingRegisterWrite(REG_GRAY_TO_BINARY, settings.grayToBinaryEnabled);
    modbusServer.holdingRegisterWrite(REG_POSITION_LSB_BIT, settings.positionLsbBit);
    modbusServer.holdingRegisterWrite(REG_POSITION_MSB_BIT, settings.positionMsbBit);
    modbusServer.holdingRegisterWrite(REG_SERIAL_HIGH, highRegisterWord(settings.serialNumber));
    modbusServer.holdingRegisterWrite(REG_SERIAL_LOW, lowRegisterWord(settings.serialNumber));
    modbusServer.holdingRegisterWrite(REG_FW_VERSION_MAJOR, settings.firmwareMajor);
    modbusServer.holdingRegisterWrite(REG_FW_VERSION_MINOR, settings.firmwareMinor);
    modbusServer.holdingRegisterWrite(REG_SAVE_SETTINGS, 0);
}

void writeReadOnlyRegisters()
{
    modbusServer.holdingRegisterWrite(REG_SERIAL_HIGH, highRegisterWord(settings.serialNumber));
    modbusServer.holdingRegisterWrite(REG_SERIAL_LOW, lowRegisterWord(settings.serialNumber));
    modbusServer.holdingRegisterWrite(REG_FW_VERSION_MAJOR, settings.firmwareMajor);
    modbusServer.holdingRegisterWrite(REG_FW_VERSION_MINOR, settings.firmwareMinor);
}

uint32_t grayToBinary(uint32_t value)
{
    for (uint32_t mask = value >> 1; mask != 0; mask >>= 1)
    {
        value ^= mask;
    }

    return value;
}

uint32_t extractPositionBits(uint32_t frame)
{
    uint8_t width = settings.positionMsbBit - settings.positionLsbBit + 1;
    uint32_t mask = width >= 32 ? 0xFFFFFFFFUL : ((1UL << width) - 1UL);

    return (frame >> settings.positionLsbBit) & mask;
}

void updatePositionRegisters()
{
    modbusServer.holdingRegisterWrite(REG_POSITION_HIGH, highRegisterWord(encoderPosition));
    modbusServer.holdingRegisterWrite(REG_POSITION_LOW, lowRegisterWord(encoderPosition));
}

void applyWritableRegisters()
{
    uint8_t newIp[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        int value = modbusServer.holdingRegisterRead(REG_IP_0 + i);
        if (value < 0 || value > 255)
        {
            return;
        }
        newIp[i] = (uint8_t)value;
    }

    int newMarks = modbusServer.holdingRegisterRead(REG_MARKS_PER_REV);
    int newSsiSize = modbusServer.holdingRegisterRead(REG_SSI_SIZE_BITS);
    int newGrayToBinary = modbusServer.holdingRegisterRead(REG_GRAY_TO_BINARY);
    int newPositionLsbBit = modbusServer.holdingRegisterRead(REG_POSITION_LSB_BIT);
    int newPositionMsbBit = modbusServer.holdingRegisterRead(REG_POSITION_MSB_BIT);
    if (newMarks <= 0 || newSsiSize <= 0 || newSsiSize > 32 || newGrayToBinary < 0
        || newGrayToBinary > 1 || newPositionLsbBit < 0 || newPositionMsbBit < 0
        || newPositionLsbBit > newPositionMsbBit || newPositionMsbBit >= newSsiSize)
    {
        writeStaticRegisters();
        return;
    }

    memcpy(settings.ip, newIp, sizeof(settings.ip));
    settings.marksPerRev = (uint16_t)newMarks;
    settings.ssiSizeBits = (uint8_t)newSsiSize;
    settings.grayToBinaryEnabled = (uint8_t)newGrayToBinary;
    settings.positionLsbBit = (uint8_t)newPositionLsbBit;
    settings.positionMsbBit = (uint8_t)newPositionMsbBit;

    if (modbusServer.holdingRegisterRead(REG_SAVE_SETTINGS) == 1)
    {
        saveSettings();
        modbusServer.holdingRegisterWrite(REG_SAVE_SETTINGS, 0);
        Serial.println(F("Settings saved to EEPROM. Reboot to apply changed IP address."));
    }
}

uint32_t readSSI()
{
    uint8_t bitCount = settings.ssiSizeBits;
    uint32_t value = 0;

    static char const t_us = 2;

    noInterrupts();
    digitalWrite(SSI_CS_PIN, LOW);
    delayMicroseconds(t_us);
    for (uint8_t i = 0; i < bitCount; i++)
    {
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

void setupEthernetAndModbus()
{
    byte mac[] = {0x02, 0x4D, 0x42, 0x45, 0x53, 0x01};

    Ethernet.init(W5500_CS_PIN);
    Ethernet.begin(mac, configuredIp());
    ethernetServer.begin();

    if (!modbusServer.begin())
    {
        Serial.println(F("Failed to start Modbus TCP server"));
        while (true)
        {
            delay(1000);
        }
    }

    modbusServer.configureHoldingRegisters(0, HOLDING_REG_COUNT);
    writeStaticRegisters();
    updatePositionRegisters();
}

void setup()
{
    pinMode(SSI_CS_PIN, OUTPUT);
    pinMode(SSI_CLK_PIN, OUTPUT);
    pinMode(SSI_DATA_PIN, INPUT);
    digitalWrite(SSI_CS_PIN, HIGH);
    digitalWrite(SSI_CLK_PIN, HIGH);

    Serial.begin(115200);
    delay(500);

    loadSettings();
    printDiagnostics();
    setupEthernetAndModbus();
}

void loop()
{
    if (!modbusClient || !modbusClient.connected())
    {
        modbusClient = ethernetServer.available();
        if (modbusClient)
        {
            Serial.println(F("has client"));
            modbusServer.accept(modbusClient);
            Serial.println(F("accepted client"));
        }
    }

    if (modbusClient && modbusClient.connected())
    {
        modbusServer.poll();
    }
    applyWritableRegisters();
    writeReadOnlyRegisters();

    if (millis() - lastSsiReadMs >= 50)
    {
        lastSsiReadMs = millis();
        uint32_t rawFrame = readSSI();
        encoderPosition = extractPositionBits(rawFrame);
        if (settings.grayToBinaryEnabled)
        {
            encoderPosition = grayToBinary(encoderPosition);
        }

        Serial.print(F("pos: "));
        Serial.println(encoderPosition);
    
        updatePositionRegisters();
    }
}
