// Minimal CST328 capacitive touch driver (I2C, first touch point only).
// Register map and reset sequence follow Waveshare's Arduino demo for the
// ESP32-S3-Touch-LCD-2.8 (Touch_CST328.cpp, I2C address 0x1A).
// Coordinates are raw panel coordinates (portrait, matching MADCTL 0x00);
// lvgl_driver.h maps them to the screen orientation.
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

    void begin();   // starts I2C on Wire1 and resets the controller
    void read();    // updates isTouched / points[0]

    bool       isTouched = false;
    TouchPoint points[1] = {};

private:
    int _sda, _scl, _int, _rst, _width, _height;
    bool writeRegister(uint16_t reg, const uint8_t* data, uint8_t length);
    bool readRegister(uint16_t reg, uint8_t* data, uint8_t length);
};

#endif
