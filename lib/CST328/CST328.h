// Minimal CST328 capacitive touch driver (I2C, first touch point only).
// Register map and reset sequence follow Waveshare's V2 Arduino demo
// (Touch_CST328.cpp): address 0x1A, 16-bit registers. A CST328 answers
// the debug-info read with 0xCACA. Coordinates are raw panel coordinates
// (portrait, matching MADCTL 0x00); lvgl_driver.h maps them.
// Part of Case Feeder Pro. Licensed under GPL-3.0.
#ifndef CST328_H
#define CST328_H

#include <Arduino.h>
#include <Wire.h>

struct TouchPoint {
    uint16_t x;
    uint16_t y;
};

class CST328 {
public:
    CST328(int sda, int scl, int intPin, int rstPin, int width, int height);

    // Starts Wire1, pulses reset the CST328 way, and returns true when the
    // controller's signature is 0xCACA.
    bool probe();
    void read();

    bool       isTouched = false;
    TouchPoint points[1] = {};

private:
    int _sda, _scl, _int, _rst, _width, _height;
    uint16_t readSignature();
    bool writeRegister(uint16_t reg, const uint8_t* data, uint8_t length);
    bool readRegister(uint16_t reg, uint8_t* data, uint8_t length);
};

#endif
