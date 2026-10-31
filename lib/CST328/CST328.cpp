// Minimal CST328 capacitive touch driver. Part of Case Feeder Pro. GPL-3.0.
//
// Protocol from Waveshare's ESP32-S3-Touch-LCD-2.8 Arduino demo
// (Touch_CST328.cpp / Touch_CST328.h) and their ESP-IDF CST328 driver:
//   address 0x1A, 16-bit registers, point count at 0xD005, coordinates at 0xD000.
// The touch bus is Wire1 (GPIO1/GPIO3). The IMU and RTC use a different bus.
#include "CST328.h"

static constexpr uint8_t  ADDR           = 0x1A;
static constexpr uint16_t REG_POINTS     = 0xD005;
static constexpr uint16_t REG_XY         = 0xD000;
static constexpr uint16_t REG_NORMAL     = 0xD109;
static constexpr uint32_t I2C_HZ         = 400000;

CST328::CST328(int sda, int scl, int intPin, int rstPin, int width, int height)
    : _sda(sda), _scl(scl), _int(intPin), _rst(rstPin), _width(width), _height(height) {}

void CST328::begin() {
    // Wire1, not Wire: Waveshare's demo keeps the touch controller off the
    // GPIO11/GPIO10 bus shared by the QMI8658 and PCF85063.
    Wire1.begin(_sda, _scl, I2C_HZ);
    if (_int >= 0) pinMode(_int, INPUT);
    if (_rst >= 0) {
        pinMode(_rst, OUTPUT);
        // Waveshare Arduino demo: high, then a short low pulse, then high.
        digitalWrite(_rst, HIGH);
        delay(50);
        digitalWrite(_rst, LOW);
        delay(5);
        digitalWrite(_rst, HIGH);
        delay(50);
    }
    if (!writeRegister(REG_NORMAL, nullptr, 0)) {
        Serial.println("CST328: no reply at 0x1A");
        Serial.println("CST328: V2 boards (label on the back) use a CST3530, which this build does not speak");
    }
}

void CST328::read() {
    isTouched = false;
    uint8_t count = 0;
    if (!readRegister(REG_POINTS, &count, 1)) return;

    // The demo clears 0xD005 after every read so a lifted finger does not stick.
    const uint8_t clear = 0;
    const uint8_t fingers = count & 0x0F;
    if (fingers == 0 || fingers > 5) {
        writeRegister(REG_POINTS, &clear, 1);
        return;
    }

    uint8_t xy[27];
    if (!readRegister(REG_XY, xy, sizeof(xy))) {
        writeRegister(REG_POINTS, &clear, 1);
        return;
    }
    writeRegister(REG_POINTS, &clear, 1);

    // First finger. Waveshare stores the 0xD000 block at buf[1], then takes
    // x from buf[2]/buf[4] and y from buf[3]/buf[4]. xy[0] is that buf[1].
    points[0].x = ((uint16_t)xy[1] << 4) | ((xy[3] & 0xF0) >> 4);
    points[0].y = ((uint16_t)xy[2] << 4) | (xy[3] & 0x0F);
    isTouched = true;
}

bool CST328::writeRegister(uint16_t reg, const uint8_t* data, uint8_t length) {
    Wire1.beginTransmission(ADDR);
    Wire1.write((uint8_t)(reg >> 8));
    Wire1.write((uint8_t)reg);
    for (uint8_t i = 0; i < length; i++) Wire1.write(data[i]);
    return Wire1.endTransmission() == 0;
}

bool CST328::readRegister(uint16_t reg, uint8_t* data, uint8_t length) {
    Wire1.beginTransmission(ADDR);
    Wire1.write((uint8_t)(reg >> 8));
    Wire1.write((uint8_t)reg);
    if (Wire1.endTransmission(true) != 0) return false;
    if (Wire1.requestFrom((int)ADDR, (int)length) != length) {
        while (Wire1.available()) Wire1.read();
        return false;
    }
    for (uint8_t i = 0; i < length; i++) data[i] = Wire1.read();
    return true;
}
