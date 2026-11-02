// Minimal CST3530 capacitive touch driver (I2C, first touch point only).
// Address, reset, and the read sequence follow Waveshare's V2 demo
// (Arduino Touch_CST3530.cpp and ESP-IDF CST3530.c): address 0x58,
// 32-bit registers. Same pins as the CST328 on this board.
// Coordinates are raw panel coordinates (portrait); lvgl_driver.h maps them.
// Part of Case Feeder Pro. Licensed under GPL-3.0.
#ifndef CST3530_H
#define CST3530_H

#include "CST328.h"

class CST3530 {
public:
    CST3530(int sda, int scl, int intPin, int rstPin, int width, int height);

    // Re-starts Wire1, holds reset low for 100 ms, then waits 500 ms, and
    // returns true when address 0x58 ACKs. That is Waveshare's TOUCH2_Init.
    bool probe();
    void read();

    bool       isTouched = false;
    TouchPoint points[1] = {};

private:
    int _sda, _scl, _int, _rst, _width, _height;
    bool writeRegister(uint32_t reg, const uint8_t* data, uint8_t length);
    bool readRegister(uint32_t reg, uint8_t* data, uint8_t length);
};

#endif
