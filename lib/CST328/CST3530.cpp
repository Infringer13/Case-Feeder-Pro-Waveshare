// Minimal CST3530 capacitive touch driver. Part of Case Feeder Pro. GPL-3.0.
//
// Protocol from Waveshare's ESP32-S3-Touch-LCD-2.8 V2 demo
// (Arduino Touch_CST3530.cpp and ESP-IDF main/Touch_Driver/CST3530.c):
//   address 0x58
//   data block  0xD0070000 (9 bytes for the first finger)
//   end-of-read 0xD00002AB
//   reset is active-low for 100 ms, then high for 500 ms
#include "CST3530.h"

static constexpr uint8_t  ADDR        = 0x58;
static constexpr uint32_t REG_DATA    = 0xD0070000;
static constexpr uint32_t REG_END     = 0xD00002AB;
static constexpr uint32_t I2C_HZ      = 400000;

CST3530::CST3530(int sda, int scl, int intPin, int rstPin, int width, int height)
    : _sda(sda), _scl(scl), _int(intPin), _rst(rstPin), _width(width), _height(height) {}

bool CST3530::probe() {
    // Waveshare's TOUCH2_Init starts the bus again, then uses its own reset.
    Wire1.begin(_sda, _scl, I2C_HZ);
    if (_int >= 0) pinMode(_int, INPUT);
    if (_rst >= 0) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, LOW);
        delay(100);
        digitalWrite(_rst, HIGH);
        delay(500);
    }
    Wire1.beginTransmission(ADDR);
    if (Wire1.endTransmission(true) != 0) {
        Serial.println("CST3530 not found at I2C address 0x58");
        return false;
    }
    Serial.println("Touch controller CST3530 initialized");
    return true;
}

void CST3530::read() {
    isTouched = false;
    uint8_t buf[9] = {};
    if (!readRegister(REG_DATA, buf, sizeof(buf))) return;

    // No finger, or the status nibble in the last byte says the report is empty.
    const uint8_t fingers = buf[3] & 0x0F;
    if (fingers == 0 || (buf[8] & 0xF0) == 0 || fingers > 5) {
        writeRegister(REG_END, nullptr, 0);
        return;
    }

    // First finger only. Extra fingers live at 0xD0070900; this UI uses one point.
    // Decode matches Touch2_Read_Data / esp_lcd_touch_cst3530_read_data for i = 0.
    points[0].x = (uint16_t)(((buf[7] & 0x0F) << 8) + buf[4]);
    points[0].y = (uint16_t)(((buf[7] & 0xF0) << 4) + buf[5]);
    writeRegister(REG_END, nullptr, 0);
    isTouched = true;
}

bool CST3530::writeRegister(uint32_t reg, const uint8_t* data, uint8_t length) {
    Wire1.beginTransmission(ADDR);
    Wire1.write((uint8_t)((reg >> 24) & 0xFF));
    Wire1.write((uint8_t)((reg >> 16) & 0xFF));
    Wire1.write((uint8_t)((reg >> 8) & 0xFF));
    Wire1.write((uint8_t)(reg & 0xFF));
    for (uint8_t i = 0; i < length; i++) Wire1.write(data[i]);
    return Wire1.endTransmission(true) == 0;
}

bool CST3530::readRegister(uint32_t reg, uint8_t* data, uint8_t length) {
    Wire1.beginTransmission(ADDR);
    Wire1.write((uint8_t)((reg >> 24) & 0xFF));
    Wire1.write((uint8_t)((reg >> 16) & 0xFF));
    Wire1.write((uint8_t)((reg >> 8) & 0xFF));
    Wire1.write((uint8_t)(reg & 0xFF));
    // Repeated start, as in Touch2_I2C_Read (endTransmission(false)).
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)ADDR, (int)length) != length) {
        while (Wire1.available()) Wire1.read();
        return false;
    }
    for (uint8_t i = 0; i < length; i++) data[i] = Wire1.read();
    return true;
}
