// Part of Case Feeder Pro. GPL-3.0.
#include "WaveshareTouch.h"

WaveshareTouch::WaveshareTouch(int sda, int scl, int intPin, int rstPin, int width, int height)
    : _cst328(sda, scl, intPin, rstPin, width, height),
      _cst3530(sda, scl, intPin, rstPin, width, height) {}

void WaveshareTouch::begin() {
    _chip = Chip::None;
    if (_cst328.probe()) {
        _chip = Chip::Cst328;
        return;
    }
    if (_cst3530.probe()) {
        _chip = Chip::Cst3530;
        return;
    }
    Serial.println("Touch: neither CST328 (0x1A) nor CST3530 (0x58) answered");
}

void WaveshareTouch::read() {
    isTouched = false;
    if (_chip == Chip::Cst328) {
        _cst328.read();
        isTouched = _cst328.isTouched;
        points[0] = _cst328.points[0];
    } else if (_chip == Chip::Cst3530) {
        _cst3530.read();
        isTouched = _cst3530.isTouched;
        points[0] = _cst3530.points[0];
    }
}
