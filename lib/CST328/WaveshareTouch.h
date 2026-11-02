// Touch front-end for the Waveshare ESP32-S3-Touch-LCD-2.8.
// Waveshare's V2 demo (ST7789.c and Touch_CST328.cpp) tries the CST328
// first and, if that signature is not 0xCACA, initialises the CST3530.
// V1 boards are the CST328; boards labeled V2 or Rev1.1 are the CST3530.
// Both use SDA 1, SCL 3, INT 4, RST 2.
// Part of Case Feeder Pro. Licensed under GPL-3.0.
#ifndef WAVESHARE_TOUCH_H
#define WAVESHARE_TOUCH_H

#include "CST328.h"
#include "CST3530.h"

class WaveshareTouch {
public:
    WaveshareTouch(int sda, int scl, int intPin, int rstPin, int width, int height);

    void begin();
    void read();

    bool       isTouched = false;
    TouchPoint points[1] = {};

private:
    enum class Chip : uint8_t { None, Cst328, Cst3530 };

    CST328 _cst328;
    CST3530 _cst3530;
    Chip _chip = Chip::None;
};

#endif
