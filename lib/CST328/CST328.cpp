// Minimal CST328 capacitive touch driver. Part of Case Feeder Pro. GPL-3.0.
//
// Protocol from Waveshare's ESP32-S3-Touch-LCD-2.8 V2 Arduino demo
// (Touch_CST328.cpp / Touch_CST328.h):
//   address 0x1A, 16-bit registers, point count at 0xD005, coordinates at 0xD000.
// Identification is the debug-info word 0xCACA, the same test their Touch_Init uses
// before it falls through to the CST3530.
#include "CST328.h"

static constexpr uint8_t  ADDR           = 0x1A;
static constexpr uint16_t REG_POINTS     = 0xD005;
static constexpr uint16_t REG_XY         = 0xD000;
static constexpr uint16_t REG_DEBUG_INFO = 0xD101;
static constexpr uint16_t REG_INFO_BLOCK = 0xD1F4;
static constexpr uint16_t REG_BOOT_TIME  = 0xD1FC;
static constexpr uint16_t REG_NORMAL     = 0xD109;
static constexpr uint16_t SIGNATURE      = 0xCACA;
static constexpr uint32_t I2C_HZ         = 400000;

CST328::CST328(int sda, int scl, int intPin, int rstPin, int width, int height)
    : _sda(sda), _scl(scl), _int(intPin), _rst(rstPin), _width(width), _height(height) {}

bool CST328::probe() {
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
    const uint16_t signature = readSignature();
    if (signature == SIGNATURE) {
        Serial.println("Touch controller CST328 initialized");
        return true;
    }
    Serial.println("CST328 init failed, try CST3530");
    return false;
}

uint16_t CST328::readSignature() {
    // Enter debug-info mode, then read the block their demo checks for 0xCACA.
    // A CST3530 does not answer 0x1A, so the first write fails and we stop.
    if (!writeRegister(REG_DEBUG_INFO, nullptr, 0)) return 0;
    uint8_t boot[4];
    if (!readRegister(REG_BOOT_TIME, boot, sizeof(boot))) return 0;
    if (!readRegister(REG_BOOT_TIME, boot, sizeof(boot))) return 0;
    uint8_t info[24] = {};
    if (!readRegister(REG_INFO_BLOCK, info, sizeof(info))) return 0;
    const uint16_t signature = ((uint16_t)info[11] << 8) | info[10];
    writeRegister(REG_NORMAL, nullptr, 0);
    return signature;
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
