/**
 * @file lvgl_driver.h
 * LVGL v8 display and touch driver for the 2.8" ESP32-S3 display module.
 * Uses TFT_eSPI for the display and the board's touch controller
 * (FT6336 on the ES3C28P, CST328 on the Waveshare). TFT_eSPI is
 * configured via build flags in platformio.ini.
 * Part of Case Feeder Pro. Licensed under GPL-3.0.
 */

#ifndef LVGL_DRIVER_H
#define LVGL_DRIVER_H

#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#if defined(BOARD_WAVESHARE)
#include <CST328.h>
#else
#include <FT6336.h>
#endif
#include "display_config.h"

// Display buffer size - 10 rows of pixels (~1/24 of the 320x240 frame) per
// buffer, for memory efficiency. LVGL flushes dirty areas in slices this tall.
#define DRAW_BUF_SIZE (SCREEN_WIDTH * 10)

// TFT and touch objects - managed by this driver
static TFT_eSPI* _tft = nullptr;
#if defined(BOARD_WAVESHARE)
static CST328* _touch = nullptr;
#else
static FT6336* _touch = nullptr;
#endif
// Screen turned 180 degrees? Set before lvgl_driver_init() (from the saved
// setting / build default) and changed live with lvgl_set_flipped().
static bool _screenFlipped = (SCREEN_FLIPPED != 0);
static lv_disp_draw_buf_t _draw_buf;
static lv_disp_drv_t _disp_drv;
static lv_indev_drv_t _touch_drv;
static lv_color_t* _buf1 = nullptr;
static lv_color_t* _buf2 = nullptr;

/**
 * Display flush callback - sends pixels to TFT
 */
static void lvgl_display_flush(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
    if (!_tft) return;

    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    _tft->startWrite();
    _tft->setAddrWindow(area->x1, area->y1, w, h);
    _tft->pushColors((uint16_t*)&color_p->full, w * h, true);
    _tft->endWrite();

    lv_disp_flush_ready(disp);
}

/**
 * Touch read callback. Both controllers report portrait panel coordinates
 * that match TFT_eSPI rotation 0 of their own driver:
 *   FT6336 on the ES3C28P, and the CST328 in Waveshare's Arduino demo
 *   (MADCTL 0x00, touch X/Y passed straight through in LVGL_Driver.cpp).
 * Rotation 1 is portrait turned 90 degrees, so landscape X is the panel's
 * Y and landscape Y is the panel's X run backwards. Flip screen is that
 * landscape turned another 180 degrees. The same numbers come out of the
 * ES3C28P's ILI9341 rotation-1 map, which this callback already used.
 */
static void lvgl_touch_read(lv_indev_drv_t* indev, lv_indev_data_t* data) {
    if (!_touch) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    _touch->read();

    if (_touch->isTouched) {
        // Raw touch coordinates, portrait 240 x 320.
        int rawX = _touch->points[0].x;
        int rawY = _touch->points[0].y;

        // Landscape (rotation 1): screen X = touch Y, screen Y = 240 - touch X.
        int screenX = rawY;
        int screenY = SCREEN_HEIGHT - 1 - rawX;
        // Flipped 180 (rotation 3): the same mapping mirrored on both axes.
        if (_screenFlipped) {
            screenX = SCREEN_WIDTH - 1 - screenX;
            screenY = SCREEN_HEIGHT - 1 - screenY;
        }

        // Clamp to screen bounds
        if (screenX < 0) screenX = 0;
        if (screenX >= SCREEN_WIDTH) screenX = SCREEN_WIDTH - 1;
        if (screenY < 0) screenY = 0;
        if (screenY >= SCREEN_HEIGHT) screenY = SCREEN_HEIGHT - 1;

        data->point.x = screenX;
        data->point.y = screenY;
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

#if defined(BOARD_WAVESHARE)
// Waveshare's Arduino demo (Display_ST7789.cpp) and their ST7789T init use
// these power and gamma values. TFT_eSPI's generic ST7789 init does not.
// MADCTL is left to setRotation(); inversion is invertDisplay() (command 0x21).
static void waveshare_cmd(TFT_eSPI* tft, uint8_t cmd, const uint8_t* data, size_t n) {
    tft->writecommand(cmd);
    for (size_t i = 0; i < n; i++) tft->writedata(data[i]);
}

static void waveshare_panel_setup(TFT_eSPI* tft) {
    const uint8_t ramctrl[] = { 0x00, 0xE8 };
    const uint8_t porch[]   = { 0x0C, 0x0C, 0x00, 0x33, 0x33 };
    const uint8_t gate[]    = { 0x75 };
    const uint8_t vcom[]    = { 0x1A };
    const uint8_t lcm[]     = { 0x2C };   // Arduino demo. The ESP-IDF demo writes 0x80 here.
    const uint8_t vdvvrh[]  = { 0x01, 0xFF };
    const uint8_t vrh[]     = { 0x13 };
    const uint8_t vdv[]     = { 0x20 };
    const uint8_t frame[]   = { 0x0F };
    const uint8_t power[]   = { 0xA4, 0xA1 };
    const uint8_t d6[]      = { 0xA1 };
    const uint8_t gammaP[]  = { 0xD0, 0x0D, 0x14, 0x0D, 0x0D, 0x09, 0x38, 0x44,
                                0x4E, 0x3A, 0x17, 0x18, 0x2F, 0x30 };
    const uint8_t gammaN[]  = { 0xD0, 0x09, 0x0F, 0x08, 0x07, 0x14, 0x37, 0x44,
                                0x4D, 0x38, 0x15, 0x16, 0x2C, 0x2E };
    waveshare_cmd(tft, 0xB0, ramctrl, sizeof(ramctrl));
    waveshare_cmd(tft, 0xB2, porch,   sizeof(porch));
    waveshare_cmd(tft, 0xB7, gate,    sizeof(gate));
    waveshare_cmd(tft, 0xBB, vcom,    sizeof(vcom));
    waveshare_cmd(tft, 0xC0, lcm,     sizeof(lcm));
    waveshare_cmd(tft, 0xC2, vdvvrh,  sizeof(vdvvrh));
    waveshare_cmd(tft, 0xC3, vrh,     sizeof(vrh));
    waveshare_cmd(tft, 0xC4, vdv,     sizeof(vdv));
    waveshare_cmd(tft, 0xC6, frame,   sizeof(frame));
    waveshare_cmd(tft, 0xD0, power,   sizeof(power));
    waveshare_cmd(tft, 0xD6, d6,      sizeof(d6));
    waveshare_cmd(tft, 0xE0, gammaP,  sizeof(gammaP));
    waveshare_cmd(tft, 0xE1, gammaN,  sizeof(gammaN));
}
#endif

/**
 * Initialize LVGL with TFT_eSPI display and the board's touch controller.
 * TFT_eSPI is configured via build flags in platformio.ini.
 */
inline void lvgl_driver_init(void) {
    Serial.println("LVGL: Initializing driver...");

    // Create and initialize TFT display
    _tft = new TFT_eSPI();
    _tft->init();
#if defined(BOARD_WAVESHARE)
    waveshare_panel_setup(_tft);
#endif
    // Both panels need inversion on: the ES3C28P's ILI9341, and the
    // Waveshare ST7789 (their demo sends command 0x21). TFT_eSPI's ST7789
    // init already sends it; setting it again leaves it on.
    _tft->invertDisplay(true);
    _tft->setRotation(_screenFlipped ? SCREEN_ROTATION_FLIPPED : SCREEN_ROTATION);
    _tft->fillScreen(TFT_BLACK);

    // Backlight. TFT_BL is GPIO45 on the ES3C28P and GPIO5 on the Waveshare
    // (wiki LCD_BL, demo EXAMPLE_LCD_BK_LIGHT_ON_LEVEL = 1).
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    Serial.println("LVGL: TFT initialized");

    // Initialize LVGL library
    lv_init();
    Serial.println("LVGL: Core initialized");

    // Allocate draw buffers
    _buf1 = (lv_color_t*)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    _buf2 = (lv_color_t*)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);

    if (!_buf1) {
        Serial.println("LVGL: ERROR - Failed to allocate draw buffer!");
        return;
    }

    if (_buf2) {
        lv_disp_draw_buf_init(&_draw_buf, _buf1, _buf2, DRAW_BUF_SIZE);
        Serial.println("LVGL: Using double buffer mode");
    } else {
        lv_disp_draw_buf_init(&_draw_buf, _buf1, nullptr, DRAW_BUF_SIZE);
        Serial.println("LVGL: Using single buffer mode");
    }

    // Initialize display driver
    lv_disp_drv_init(&_disp_drv);
    _disp_drv.hor_res = SCREEN_WIDTH;
    _disp_drv.ver_res = SCREEN_HEIGHT;
    _disp_drv.flush_cb = lvgl_display_flush;
    _disp_drv.draw_buf = &_draw_buf;
    lv_disp_drv_register(&_disp_drv);

    Serial.printf("LVGL: Display driver registered (%dx%d)\n", SCREEN_WIDTH, SCREEN_HEIGHT);

    // Touch controller (pins in display_config.h). Portrait 240 x 320.
#if defined(BOARD_WAVESHARE)
    _touch = new CST328(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, 240, 320);
    _touch->begin();
    Serial.println("LVGL: Touch initialized (CST328 @ 0x1A)");
#else
    _touch = new FT6336(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, 240, 320);
    _touch->begin();
    _touch->setThreshold(40);  // Lower threshold = more sensitive (default ~80)
    Serial.println("LVGL: Touch initialized (FT6336, threshold=40)");
#endif

    // Initialize touch driver
    lv_indev_drv_init(&_touch_drv);
    _touch_drv.type = LV_INDEV_TYPE_POINTER;
    _touch_drv.read_cb = lvgl_touch_read;
    lv_indev_drv_register(&_touch_drv);
    Serial.println("LVGL: Touch driver registered");

    Serial.println("LVGL: Driver initialization complete");
}

/**
 * Get the TFT_eSPI instance (for splash screen or direct drawing if needed)
 */
inline TFT_eSPI* lvgl_get_tft(void) {
    return _tft;
}

/**
 * Choose the 180-degree flip. Before lvgl_driver_init() it just sets the
 * startup orientation; afterwards it rotates the panel and touch mapping live
 * and forces a full redraw (the layout is identical either way).
 */
inline void lvgl_set_flipped(bool flipped) {
    _screenFlipped = flipped;
    if (!_tft) return;
    _tft->setRotation(flipped ? SCREEN_ROTATION_FLIPPED : SCREEN_ROTATION);
    lv_obj_invalidate(lv_scr_act());
    lv_obj_invalidate(lv_layer_top());
}

inline bool lvgl_is_flipped(void) {
    return _screenFlipped;
}

#endif // LVGL_DRIVER_H
