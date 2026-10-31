/*
 * display_config.h
 * Screen geometry and capacitive-touch pins for the touchscreen firmware.
 *
 * The display SPI pins are build flags in platformio.ini (TFT_eSPI).
 * Which board those flags describe is selected by the PlatformIO env:
 *   lcd             LCDWiki ES3C28P (ILI9341 + FT6336)
 *   lcd_waveshare   Waveshare ESP32-S3-Touch-LCD-2.8 (ST7789 + CST328)
 *
 * Part of Case Feeder Pro. Licensed under GPL-3.0.
 */
#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

// ===== DISPLAY GEOMETRY =====
// Both panels are 240x320 portrait glass. The UI is landscape.
#define SCREEN_WIDTH    320
#define SCREEN_HEIGHT   240
// TFT_eSPI rotation. On the ES3C28P, rotation 1 puts the USB socket on the
// right and rotation 3 puts it on the left. On the Waveshare the same two
// rotations are 180 degrees apart; which edge faces the operator depends
// on how the board is mounted (the case lid is not drawn for that board).
#define SCREEN_ROTATION         1
#define SCREEN_ROTATION_FLIPPED 3

// Default orientation: build with -DSCREEN_FLIPPED=1 to start with the screen
// turned 180 degrees. Settings > Flip screen changes it on the unit; that
// choice is stored on the LCD and wins over this default.
#ifndef SCREEN_FLIPPED
#define SCREEN_FLIPPED 0
#endif

#if defined(BOARD_WAVESHARE)
// ===== TOUCH PANEL (CST328, I2C 0x1A) =====
// Waveshare wiki "LCD" table and Touch_CST328.h in their Arduino demo.
// This bus is not the 4-pin I2C header (that header is GPIO11/GPIO10).
#define TOUCH_SDA 1
#define TOUCH_SCL 3
#define TOUCH_INT 4
#define TOUCH_RST 2
#else
// ===== TOUCH PANEL (FT6336, I2C - onboard) =====
#define TOUCH_SDA 16
#define TOUCH_SCL 15
#define TOUCH_INT 17
#define TOUCH_RST 18
#endif

#endif // DISPLAY_CONFIG_H
