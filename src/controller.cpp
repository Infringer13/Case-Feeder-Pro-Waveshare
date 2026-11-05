// ============================================================
//  Case Feeder Pro - controller firmware
//  BigTreeTech SKR Pico V1.0 (RP2040)
//  Copyright (C) the Case Feeder Pro contributors. Licensed under GPL-3.0.
//
//  Runs a CASE feeder, a BULLET feeder, or both side by side. The MACHINE
//  setting (chosen on the LCD under Settings > Machine, or `K` on serial)
//  says which. Each feeder has fixed ports, so changing the machine never
//  needs rewiring; a feeder that is not part of the machine is held off.
//
//  Each feeder runs independently: its own state machine, beam, jam
//  detection, tuning and settings. A jam, fault or idle shutoff on one never
//  stops the other. The controller owns every real-time and safety
//  behavior; the LCD is optional and the feeders keep running without it.
//
//  Pin map (BTT SKR Pico V1.0):
//    CASE    X driver : STEP 11, DIR 10, EN 12 (TMC UART address 0)
//            DIAG GPIO4  via the X-DIAG jumper onto X-STOP
//            beam GPIO3  on Y-STOP (IO3 / GND / 5V)
//    BULLET  E driver : STEP 14, DIR 13, EN 15 (TMC UART address 3)
//            DIAG GPIO16 via the E0-DIAG jumper onto E0-STOP
//            beam GPIO25 on Z-STOP (IO25 / GND / 5V)
//    GPIO8/9  TMC UART (shared bus)     GPIO0/1  LCD UART (Pi header)
//    GPIO24   status RGB LED            GPIO17   FAN1 (driver fan)
//  Y and Z drivers are unused and held disabled. Jumpers: X-DIAG and E0-DIAG
//  fitted; Y-DIAG and Z-DIAG removed (they would land on the beam inputs).
// ============================================================

#include <Arduino.h>
#include <EEPROM.h>
#include <TMCStepper.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "feeder_protocol.h"

using namespace FeederProtocol;

// ============================================================
//  SHARED PINS / CONSTANTS
// ============================================================
static constexpr uint8_t PIN_TMC_TX = 8;    // UART1
static constexpr uint8_t PIN_TMC_RX = 9;
static constexpr uint8_t PIN_RGB    = 24;
static constexpr uint8_t PIN_FAN    = 17;
static constexpr uint8_t PIN_LCD_TX = 0;    // UART0, Pi header
static constexpr uint8_t PIN_LCD_RX = 1;
static constexpr uint8_t PINS_UNUSED_DRIVER_EN[] = { 7, 2 };            // Y, Z
static constexpr uint8_t PINS_PARKED_LOW[]       = { 21, 23, 18, 20 };  // HB, HE, FAN2, FAN3

static constexpr float    R_SENSE           = 0.11f;
static constexpr uint16_t MICROSTEPS        = 16;
static constexpr uint16_t RMS_CURRENT_MA    = 950;     // the jam thresholds are set at this current
static constexpr float    HOLD_CURRENT_MULT = 1.0f;
static constexpr uint32_t STEALTH_CALIBRATE_MS = 150;  // StealthChop settles before stepping
static constexpr uint32_t TCOOLTHRS_VALUE   = 0xFFFFF;
static constexpr uint32_t SPEED_HZ_CEILING  = 15000;
static constexpr uint32_t RAMP_PERIOD_MS    = 10;
static constexpr uint32_t JAM_PAUSE_MS      = 400;
static constexpr uint32_t STALL_CLEAR_MS    = 150;
static constexpr uint32_t SG_SAMPLE_MS      = 100;
static constexpr uint32_t NO_NEW_CASE_MS    = 8000;   // no counted case at feed speed -> jam
static constexpr uint32_t BEAM_STUCK_MS     = 3000;   // debounced block this long -> jam
static constexpr uint8_t  EMPTY_AFTER_JAMS  = 5;      // then idle shutoff instead of another reverse
static constexpr uint16_t COUNT_CONFIRM_MS  = 40;     // block must last this long to count
static constexpr uint16_t COUNT_REARM_MS    = 50;     // clear this long before the next count
static constexpr uint8_t  TMC2209_VERSION   = 0x21;   // IOIN version; a failed UART read is 0
static constexpr uint16_t SG_RESULT_MASK    = 0x3FF;
static constexpr uint8_t  BEAM_RESET_BREAKS = 2;
static constexpr uint32_t BEAM_RESET_TIMEOUT_MS = 2000;
static constexpr uint32_t STATUS_PERIOD_MS  = 500;
static constexpr uint32_t LCD_TIMEOUT_MS    = 3000;
static constexpr uint32_t DRIVER_RECHECK_MS = 250;
static constexpr uint32_t TELEMETRY_PERIOD_MS = 500;
static constexpr uint32_t LOOP_WDT_MS       = 2000;
static constexpr uint32_t TEMP_SAMPLE_MS    = 1000;
static constexpr uint8_t  LED_BRIGHTNESS    = 40;
static constexpr uint8_t  NUM_FEEDERS       = 2;
static constexpr uint8_t  MAX_BANDS         = 6;

// Jam detection threshold by speed. The TMC2209 flags a stall when
// SG_RESULT <= 2 x SGTHRS. The reading rises with speed, so each speed range
// gets its own threshold. The numbers are the tuned values for these feeders.
// A 51:1 gearbox hides ordinary load, so a real jam often does not walk down
// through the bands: the rotor stops and SG_RESULT reads 0. That 0 is a stall
// (see isStalled). Do not raise these thresholds to "chase" a 0.
struct StallBand {
    uint32_t speedMinHz;
    uint8_t  threshold;   // SGTHRS
};

// ============================================================
//  PER-MACHINE TUNING
// ============================================================
struct FeederConfig {
    const char* name;
    char        tag;              // prefix on serial log lines
    uint8_t     productType;
    uint8_t     gearbox;
    uint8_t     pinStep, pinDir, pinEnN, pinDiag, pinBeam;
    uint8_t     driverAddr;
    uint32_t    speedMinHz, speedMaxHz;
    uint32_t    jamReverseMs, jamReverseHz, jamMinSpeedHz;
    uint32_t    decelHzPerSec;
    uint16_t    beamConfirmMs, beamClearMs;
    const StallBand* bands;
    uint8_t     numBands;
};

// Case feeder, 51:1 gearbox.
static const StallBand CASE_BANDS[] = { { 0, 140 }, { 9298, 150 }, { 10729, 160 } };
static const FeederConfig CASE_CFG = {
    "CASE", 'C', PRODUCT_CASE, 51, 11, 10, 12, 4, 3, 0,
    7868, 12875, 2300, 10729, 4292, 28610, 250, 1500, CASE_BANDS, 3 };

// Bullet feeder, 51:1 gearbox. Bullet jams load the motor less than case
// jams, so its thresholds sit closer to the normal running reading.
static const StallBand BULLET_BANDS[] = {
    { 0, 168 }, { 7438, 173 }, { 8583, 177 }, { 9727, 182 }, { 10871, 186 } };
static const FeederConfig BULLET_CFG = {
    "BULLET", 'B', PRODUCT_BULLET, 51, 14, 13, 15, 16, 25, 3,
    5722, 11444, 2300, 7153, 2504, 21458, 150, 1000, BULLET_BANDS, 5 };

// ============================================================
//  SAVED SETTINGS (EEPROM emulation in flash)
// ============================================================
struct PersistData {
    uint32_t magic;
    uint16_t version;
    uint8_t  machineMask;                  // MachineMask
    uint8_t  beamActiveHigh[NUM_FEEDERS];
    uint8_t  motorReversed[NUM_FEEDERS];
    uint16_t warnSeconds[NUM_FEEDERS];
    uint8_t  speedPercent[NUM_FEEDERS];    // last speed, restored at power-up
};
static constexpr uint8_t  DEFAULT_SPEED_PERCENT = 60;   // a new board; one of the LCD presets
static constexpr uint32_t PERSIST_MAGIC   = 0x31524446u;   // "FDR1"
static constexpr uint16_t PERSIST_VERSION = 2;
static PersistData persist;
static PersistData persistSaved;

static void persistDefaults() {
    memset(&persist, 0, sizeof(persist));
    persist.magic       = PERSIST_MAGIC;
    persist.version     = PERSIST_VERSION;
    persist.machineMask = MACHINE_DUAL;    // a new board shows both; pick on the LCD
    for (uint8_t i = 0; i < NUM_FEEDERS; i++) {
        persist.warnSeconds[i]  = 30;
        persist.speedPercent[i] = DEFAULT_SPEED_PERCENT;
    }
}

// Each write erases a flash sector (~50 ms, interrupts off), so save only
// when a setting actually changes.
static void persistSave() {
    if (memcmp(&persist, &persistSaved, sizeof(persist)) == 0) return;
    EEPROM.put(0, persist);
    EEPROM.commit();
    persistSaved = persist;
}

static void persistLoad() {
    EEPROM.begin(256);
    EEPROM.get(0, persist);
    persistSaved = persist;
    if (persist.magic != PERSIST_MAGIC || persist.version != PERSIST_VERSION ||
        persist.machineMask == 0 || (persist.machineMask & ~MACHINE_DUAL)) {
        persistDefaults();
        persistSave();
    }
}

static const char* stateName(uint8_t s) {
    switch (s) {
        case STATE_RUNNING:      return "RUNNING";
        case STATE_BEAM_BLOCKED: return "BEAM_BLOCKED";
        case STATE_JAM_REVERSE:  return "JAM_REVERSE";
        case STATE_JAM_PAUSE:    return "JAM_PAUSE";
        case STATE_PAUSED:       return "PAUSED";
        case STATE_FAULT:        return "FAULT";
        case STATE_IDLE_SHUTOFF: return "IDLE_SHUTOFF";
        default:                 return "?";
    }
}

// ============================================================
//  FEEDER STATE
// ============================================================
struct Feeder {
    uint8_t             index;
    const FeederConfig* cfg;
    TMC2209Stepper*     drv;
    bool                enabled;      // part of this board's machine

    // runtime tuning (serial-adjustable, not saved)
    uint32_t  speedMinHz, speedMaxHz;
    uint32_t  accelHzPerSec, startupAccelHzPerSec, decelHzPerSec;
    uint32_t  jamReverseHz, jamMinSpeedHz;
    uint16_t  jamRepeatWindowMs, stallConfirmMs, stallSettleMs;
    uint16_t  beamConfirmMs, beamClearMs, countConfirmMs, countClearMs;
    uint32_t  idleWarnMs, idleShutoffMs;
    StallBand bands[MAX_BANDS];
    uint8_t   numBands;

    // motion
    State     state;
    uint32_t  stateSince;
    bool      driverReady, motorPowered, useSlowStart;
    uint32_t  appliedHz, targetHz;
    bool      motorReversed, dirForward;
    int8_t    appliedDirLevel;
    uint32_t  motorEnabledAt, lastRampAt;
    uint      stepSlice, stepChannel;
    bool      runRequested;
    uint8_t   speedPercent;

    // beam / count / idle
    bool      beamActiveLow;
    bool      beamBlocked, rawBeam;
    uint32_t  beamBrokenSince, beamClearedSince, beamClearSince, countBlockSince;
    bool      countArmed, prevBeamRunning;
    uint16_t  count;
    uint32_t  lastFedTime;
    bool      idleWarnActive, idleWarnDismissed;
    uint8_t   beamResetBreaks;
    uint32_t  beamResetStart;
    bool      prevBeamState;
    bool      edgePrev;
    uint32_t  edgeStart, edgeClearStart;

    // stall
    bool      stallActive, jamDetect, diagUsable;
    uint32_t  stallTimer, lastStallEvent, stallSettleStart, lastRecoveryEnd;
    uint16_t  sgResult, sgWindowMin, sgWindowMax;
    int8_t    currentStallBand;
    uint8_t   appliedSgThreshold;
    bool      sgSampleValid;     // last SG_RESULT reply checked out
    bool      sgZeroIsStall;     // that reply was 0 while stepping at speed
    bool      diagFromChip;      // TMC2209 IOIN.DIAG, same sample
    bool      noCaseRecovery;     // beam jam: leave the empty-feeder clock alone
    uint32_t  noNewCaseMs;        // no counted case while at feed speed
    uint32_t  beamStuckMs;        // debounced beam block
    uint32_t  lastCountedCase;    // last clean block-then-clear, or this run's start
    uint32_t  beamBlockedSince;   // debounced block began
    uint32_t  atFeedSince;        // continuous time at feed speed (flickers do not clear it)
    uint8_t   jamStreak;          // recoveries since the last counted case
    uint8_t   emptyAfterJams;     // then stop empty; 0 = keep reversing

    // health / temperature
    uint32_t  lastDriverCheck, lastSgRead;
    uint8_t   resetReads;
    bool      offlineLogged;
    uint8_t   driverTempLevel, driverTempPending;
    uint32_t  lastFaultPrint, lastIdlePrint;
};

static SerialUART& TMCSerial = Serial2;
static SerialUART& LCDSerial = Serial1;
TMC2209Stepper drvCase(&TMCSerial, R_SENSE, CASE_CFG.driverAddr);
TMC2209Stepper drvBullet(&TMCSerial, R_SENSE, BULLET_CFG.driverAddr);
Adafruit_NeoPixel statusPixel(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

static Feeder feeders[NUM_FEEDERS];
static uint8_t selected = 0;           // serial tuning commands apply to this feeder

static uint32_t lastStatusAt = 0, lastLcdRxAt = 0, lastTelemetry = 0, lastTempAt = 0;
static bool     telemetryOn = false, beamLogOn = false;
static float    boardTempC = NAN;

struct RxParser {
    enum Stage : uint8_t { WAIT_SOF, READ_LEN, READ_BODY, CRC_LOW, CRC_HIGH, WAIT_EOF } stage = WAIT_SOF;
    uint8_t length = 0;
    uint8_t index = 0;
    uint8_t body[MAX_PAYLOAD + 1]{};
    uint16_t receivedCrc = 0;
} rx;

static void writeFrame(uint8_t command, const void* payload = nullptr, uint8_t payloadLength = 0);

static void feederInit(Feeder& f, uint8_t index, const FeederConfig* cfg, TMC2209Stepper* drv) {
    memset(&f, 0, sizeof(f));
    f.index = index;
    f.cfg   = cfg;
    f.drv   = drv;
    f.speedMinHz = cfg->speedMinHz;
    f.speedMaxHz = cfg->speedMaxHz;
    f.accelHzPerSec = 143051;
    f.startupAccelHzPerSec = 35763;
    f.decelHzPerSec = cfg->decelHzPerSec;
    f.jamReverseHz  = cfg->jamReverseHz;
    f.jamMinSpeedHz = cfg->jamMinSpeedHz;
    f.jamRepeatWindowMs = 2000;
    f.stallConfirmMs = 450;
    f.stallSettleMs  = 300;
    f.beamConfirmMs  = cfg->beamConfirmMs;
    f.beamClearMs    = cfg->beamClearMs;
    f.countConfirmMs = COUNT_CONFIRM_MS;
    f.countClearMs   = COUNT_REARM_MS;
    f.idleWarnMs     = 30000;
    f.idleShutoffMs  = 120000;
    f.numBands = cfg->numBands;
    for (uint8_t i = 0; i < cfg->numBands; i++) f.bands[i] = cfg->bands[i];
    f.state = STATE_PAUSED;
    f.useSlowStart = true;
    f.dirForward = true;
    f.appliedDirLevel = -1;
    f.speedPercent = DEFAULT_SPEED_PERCENT;
    f.beamActiveLow = true;
    f.countArmed = true;
    f.jamDetect = true;
    f.currentStallBand = -1;
    f.sgWindowMin = 0xFFFF;
    f.noNewCaseMs = NO_NEW_CASE_MS;
    f.beamStuckMs = BEAM_STUCK_MS;
    f.emptyAfterJams = EMPTY_AFTER_JAMS;
}

// ============================================================
//  MOTOR CONTROL - step pulses from a hardware PWM slice
// ============================================================
static void stepPwmInit(Feeder& f) {
    gpio_set_function(f.cfg->pinStep, GPIO_FUNC_PWM);
    f.stepSlice   = pwm_gpio_to_slice_num(f.cfg->pinStep);
    f.stepChannel = pwm_gpio_to_channel(f.cfg->pinStep);
    pwm_set_chan_level(f.stepSlice, f.stepChannel, 0);
    pwm_set_enabled(f.stepSlice, true);
}

static void setStepFrequency(Feeder& f, uint32_t hz) {
    if (hz == f.appliedHz) return;
    f.appliedHz = hz;
    if (hz == 0) { pwm_set_chan_level(f.stepSlice, f.stepChannel, 0); return; }
    const uint32_t sysHz = clock_get_hz(clk_sys);
    uint32_t div = sysHz / (hz * 65536UL) + 1;
    if (div > 255) div = 255;
    uint32_t top = sysHz / (div * hz);
    if (top < 2)     top = 2;
    if (top > 65536) top = 65536;
    pwm_set_clkdiv_int_frac(f.stepSlice, (uint8_t)div, 0);
    pwm_set_wrap(f.stepSlice, (uint16_t)(top - 1));
    pwm_set_chan_level(f.stepSlice, f.stepChannel, (uint16_t)(top / 2));
}

static void motorEnable(Feeder& f, bool enable) {
    if (enable) {
        if (!f.motorPowered) {
            f.motorPowered   = true;
            f.motorEnabledAt = millis();
            f.lastRampAt     = f.motorEnabledAt;
            setStepFrequency(f, 0);
            digitalWrite(f.cfg->pinEnN, LOW);
        }
    } else {
        f.targetHz = 0;
        setStepFrequency(f, 0);
        digitalWrite(f.cfg->pinEnN, HIGH);
        f.motorPowered = false;
    }
}

static void stopImmediately(Feeder& f) {
    motorEnable(f, false);
    f.useSlowStart = true;
}

static void setDirection(Feeder& f, bool forward) {
    f.dirForward = forward;
    const int8_t level = (forward != f.motorReversed) ? HIGH : LOW;
    if (level == f.appliedDirLevel) return;
    f.appliedDirLevel = level;
    digitalWrite(f.cfg->pinDir, level);
}

static uint32_t selectedStepHz(const Feeder& f) {
    return f.speedMinHz + ((f.speedMaxHz - f.speedMinHz) * f.speedPercent) / 100U;
}

// Walk the step rate toward the target: gentle acceleration on a fresh
// start, normal acceleration otherwise, and a fast deceleration.
static void serviceMotorRamp(Feeder& f) {
    if (!f.motorPowered) return;
    const uint32_t now = millis();
    if (now - f.motorEnabledAt < STEALTH_CALIBRATE_MS) { f.lastRampAt = now; return; }
    if (f.appliedHz == f.targetHz) { f.lastRampAt = now; return; }
    uint32_t dt = now - f.lastRampAt;
    if (dt < RAMP_PERIOD_MS) return;
    if (dt > 100) dt = 100;
    f.lastRampAt = now;
    const bool decel = f.targetHz < f.appliedHz;
    const uint32_t rate = decel ? f.decelHzPerSec
                                : (f.useSlowStart ? f.startupAccelHzPerSec : f.accelHzPerSec);
    uint32_t delta = (uint32_t)(((uint64_t)rate * dt) / 1000U);
    if (delta == 0) delta = 1;
    const uint32_t prevHz = f.appliedHz;
    uint32_t next = f.appliedHz;
    if (next < f.targetHz) next += min(delta, f.targetHz - next);
    else                   next -= min(delta, next - f.targetHz);
    setStepFrequency(f, next);
    if (prevHz <= f.jamMinSpeedHz && f.appliedHz > f.jamMinSpeedHz) f.stallSettleStart = now;
    if (f.useSlowStart && f.appliedHz >= f.targetHz) f.useSlowStart = false;
}

static void serviceStallThreshold(Feeder& f) {
    if (!f.driverReady) return;
    int8_t band = 0;
    for (int8_t i = f.numBands - 1; i >= 0; i--) {
        if (f.appliedHz >= f.bands[i].speedMinHz) { band = i; break; }
    }
    if (band == f.currentStallBand && f.bands[band].threshold == f.appliedSgThreshold) return;
    f.currentStallBand   = band;
    f.appliedSgThreshold = f.bands[band].threshold;
    f.drv->SGTHRS(f.appliedSgThreshold);
}

// Live DIAG pin. The TMC2209 DIAG is active-high and open-drain, so the Pico
// pin is pulled up; a stall releases the pin and it reads HIGH.
static bool diagPinHigh(const Feeder& f) {
    return f.diagUsable && digitalRead(f.cfg->pinDiag) == HIGH;
}

// A stall is SG_RESULT <= 2 x SGTHRS, which is the compare the chip uses for
// DIAG. A fully stopped rotor reads 0. That used to be skipped (sgResult > 0),
// so the motor could sit stalled, DIAG could stay low or only pulse, and the
// feeder never reversed. 0 counts only after a real reply taken while steps
// are being issued above the jam speed: a dead UART read is also 0, and a 0
// while ramping is StallGuard with no back-EMF yet, not a jam.
static bool isStalled(const Feeder& f) {
    const uint16_t trip = (uint16_t)f.appliedSgThreshold * 2U;
    if (f.sgZeroIsStall) return true;
    if (f.sgSampleValid && f.sgResult > 0 && f.sgResult <= trip) return true;
    if (f.diagFromChip) return true;
    return diagPinHigh(f);
}

// Read StallGuard. On a 0, check IOIN so a failed UART transaction (also 0)
// is not stored as a stall. Version 0x21 is the TMC2209 identity in that reply.
static void sampleStallGuard(Feeder& f) {
    const uint16_t sg = f.drv->SG_RESULT() & SG_RESULT_MASK;
    if (sg != 0) {
        f.sgResult = sg;
        f.sgSampleValid = true;
        f.sgZeroIsStall = false;
        f.diagFromChip = false;
    } else {
        const uint32_t ioin = f.drv->IOIN();
        const bool alive = ((ioin >> 24) & 0xFF) == TMC2209_VERSION;
        const bool atSpeed = f.appliedHz > f.jamMinSpeedHz;
        if (!alive) {
            f.sgSampleValid = false;
            f.sgZeroIsStall = false;
            f.diagFromChip = false;
            return;
        }
        f.sgResult = 0;
        f.sgSampleValid = true;
        f.sgZeroIsStall = atSpeed;
        f.diagFromChip = atSpeed && ((ioin & (1UL << 4)) != 0);
    }
    if (f.sgResult < f.sgWindowMin) f.sgWindowMin = f.sgResult;
    if (f.sgResult > f.sgWindowMax) f.sgWindowMax = f.sgResult;
}

// Commanded feed rate, ramp finished, and fast enough for jam checks.
static bool runningAtFeedSpeed(const Feeder& f) {
    return f.targetHz > f.jamMinSpeedHz && f.appliedHz >= f.targetHz;
}

// ============================================================
//  BEAM
// ============================================================
static bool readBeamRaw(Feeder& f) {
    const bool low = digitalRead(f.cfg->pinBeam) == LOW;
    return f.beamActiveLow ? low : !low;
}

// Two quick beam breaks (within BEAM_RESET_TIMEOUT_MS) resume a feeder
// from FAULT or idle shutoff without the LCD.
static bool checkBeamResetGesture(Feeder& f) {
    const bool blocked = readBeamRaw(f);
    bool done = false;
    if (blocked && !f.prevBeamState) {
        if (f.beamResetBreaks == 0) f.beamResetStart = millis();
        f.beamResetBreaks++;
        Serial.printf("%c Reset gesture: break %u/%u\n", f.cfg->tag, f.beamResetBreaks, BEAM_RESET_BREAKS);
        if (f.beamResetBreaks >= BEAM_RESET_BREAKS) { f.beamResetBreaks = 0; done = true; }
    }
    f.prevBeamState = blocked;
    if (f.beamResetBreaks > 0 && millis() - f.beamResetStart > BEAM_RESET_TIMEOUT_MS) f.beamResetBreaks = 0;
    return done;
}

// One case: the beam has been blocked through the short confirm. Resets the
// no-case jam clock and the empty streak. The hold's long clear is not required.
static void noteCaseCounted(Feeder& f) {
    f.countArmed = false;
    f.count++;
    f.lastCountedCase = millis();
    f.atFeedSince = 0;
    f.jamStreak = 0;
    f.lastFedTime = millis();
    f.idleWarnActive = false;
    f.idleWarnDismissed = false;
    uint8_t evt[3];
    memcpy(evt, &f.count, 2);
    evt[2] = f.index;
    writeFrame(EVT_COUNT, evt, 3);
    Serial.printf("%c case counted %u\n", f.cfg->tag, f.count);
}

static void serviceBeam(Feeder& f) {
    const bool blocked = readBeamRaw(f);
    if (beamLogOn) {
        if (blocked && !f.edgePrev) {
            f.edgeStart = millis();
            if (f.edgeClearStart) Serial.printf("%c BEAM broken after %lu ms clear\n", f.cfg->tag,
                                                (unsigned long)(millis() - f.edgeClearStart));
            else                  Serial.printf("%c BEAM broken\n", f.cfg->tag);
        } else if (!blocked && f.edgePrev) {
            Serial.printf("%c BEAM cleared after %lu ms\n", f.cfg->tag, (unsigned long)(millis() - f.edgeStart));
            f.edgeClearStart = millis();
        }
    }
    f.edgePrev = blocked;
    f.rawBeam  = blocked;

    // Count on a short confirmed block. A fast case never sits still for the
    // hold's clear time, so that long debounce must not gate the count.
    // Re-arm after a short clear so the next case can count.
    if (blocked) {
        if (f.countBlockSince == 0) f.countBlockSince = millis();
        if (f.countArmed && millis() - f.countBlockSince >= f.countConfirmMs) noteCaseCounted(f);
        f.beamClearSince = 0;
    } else {
        f.countBlockSince = 0;
        if (f.beamClearSince == 0) f.beamClearSince = millis();
        if (!f.countArmed && millis() - f.beamClearSince >= f.countClearMs) f.countArmed = true;
    }
    f.prevBeamRunning = blocked;

    // "Blocked" (hold the motor) only after a sustained break, and released
    // only after a sustained clear. This is separate from counting.
    const bool wasBlocked = f.beamBlocked;
    if (blocked) {
        f.beamClearedSince = 0;
        if (f.beamBrokenSince == 0) f.beamBrokenSince = millis();
        else if (!f.beamBlocked && millis() - f.beamBrokenSince >= f.beamConfirmMs) f.beamBlocked = true;
    } else {
        f.beamBrokenSince = 0;
        if (f.beamBlocked) {
            if (f.beamClearedSince == 0) f.beamClearedSince = millis();
            else if (millis() - f.beamClearedSince >= f.beamClearMs) {
                f.beamBlocked = false;
                f.beamClearedSince = 0;
            }
        }
    }

    if (f.beamBlocked && !wasBlocked) {
        // Clock starts at the beginning of this unbroken block, so a flicker
        // that never confirmed does not count and does not reset it.
        f.beamBlockedSince = f.beamBrokenSince != 0 ? f.beamBrokenSince : millis();
    } else if (!f.beamBlocked && wasBlocked) {
        f.beamBlockedSince = 0;
    }
}

// ============================================================
//  STATUS LED - shows the most urgent feeder
// ============================================================
static uint8_t urgency(const Feeder& f) {
    switch (f.state) {
        case STATE_FAULT:        return 6;
        case STATE_JAM_REVERSE:
        case STATE_JAM_PAUSE:    return 5;
        case STATE_IDLE_SHUTOFF: return 4;
        case STATE_BEAM_BLOCKED: return 3;
        case STATE_RUNNING:      return 2;
        default:                 return 1;
    }
}

static void serviceLed() {
    static uint32_t shown = 0xFFFFFFFFu;
    const uint32_t now = millis();
    const Feeder* worst = nullptr;
    bool driversOk = true;
    for (const Feeder& g : feeders) {
        if (!g.enabled) continue;
        if (!worst || urgency(g) > urgency(*worst)) worst = &g;
        if (!g.driverReady) driversOk = false;
    }
    if (!worst) return;
    uint32_t colour = 0;
    switch (worst->state) {
        case STATE_RUNNING:      colour = 0x00FF00; break;
        case STATE_BEAM_BLOCKED: colour = ((now / 500U) & 1U) ? 0xFFA000 : 0; break;
        case STATE_JAM_REVERSE:
        case STATE_JAM_PAUSE:    colour = ((now / 125U) & 1U) ? 0xFF4000 : 0; break;
        case STATE_IDLE_SHUTOFF: colour = ((now / 250U) & 1U) ? 0x0040FF : 0; break;
        case STATE_FAULT:        colour = ((now / 75U)  & 1U) ? 0xFF0000 : 0; break;
        default:                 colour = 0x303030; break;
    }
    if (!driversOk) colour = ((now / 1000U) & 1U) ? 0xFF0000 : 0;
    if (colour == shown) return;
    shown = colour;
    statusPixel.setPixelColor(0, colour);
    statusPixel.show();
}

// ============================================================
//  LCD LINK
// ============================================================
static void writeFrame(uint8_t command, const void* payload, uint8_t payloadLength) {
    if (payloadLength > MAX_PAYLOAD) return;
    uint8_t frame[MAX_PAYLOAD + 7];
    size_t i = 0;
    frame[i++] = SOF;
    frame[i++] = (uint8_t)(payloadLength + 1);
    frame[i++] = command;
    if (payloadLength && payload) { memcpy(&frame[i], payload, payloadLength); i += payloadLength; }
    const uint16_t crc = crc16(&frame[1], payloadLength + 2);
    frame[i++] = (uint8_t)crc;
    frame[i++] = (uint8_t)(crc >> 8);
    frame[i++] = EOF_MARKER;
    LCDSerial.write(frame, i);
}

static void sendAlert(Feeder& f, AlertType type) {
    const uint8_t payload[2] = { (uint8_t)type, f.index };
    writeFrame(EVT_ALERT, payload, 2);
}

static uint8_t machineFeederCount() {
    uint8_t n = 0;
    for (const Feeder& f : feeders) if (f.enabled) n++;
    return n;
}

static void sendStatus(Feeder& f) {
    StatusPayload s{};
    s.protocolVersion = PROTOCOL_VERSION;
    s.state           = f.state;
    s.speedPercent    = f.speedPercent;
    s.flags = (f.beamBlocked ? FLAG_BEAM_BLOCKED : 0) |
              (f.driverReady ? FLAG_DRIVER_READY : 0) |
              ((millis() - lastLcdRxAt < LCD_TIMEOUT_MS) ? FLAG_LCD_CONNECTED : 0) |
              (f.stallActive ? FLAG_STALL_ACTIVE : 0) |
              (f.idleWarnActive ? FLAG_EMPTY_WARN : 0);
    s.gearboxRatio    = f.cfg->gearbox;
    s.count           = f.count;
    s.stepFrequencyHz = (uint16_t)f.appliedHz;
    s.sgResult        = f.sgResult;
    s.uptimeSeconds   = millis() / 1000U;
    s.warnSeconds     = (uint16_t)(f.idleWarnMs / 1000UL);
    s.boardTempC10    = isnan(boardTempC) ? BOARD_TEMP_UNKNOWN : (int16_t)lroundf(boardTempC * 10.0f);
    s.driverTemp      = f.driverTempLevel;
    s.feederIndex     = f.index;
    s.feederCount     = machineFeederCount();
    s.productType     = f.cfg->productType;
    writeFrame(EVT_STATUS, &s, sizeof(s));
}

static int  firstEnabled();
static bool setMachine(uint8_t mask, const char* source);

static void handleCommand(uint8_t command, const uint8_t* payload, uint8_t length) {
    lastLcdRxAt = millis();
    // Which feeder: an optional trailing byte after the command's own payload.
    // Without it, the first feeder in this machine.
    auto target = [&](uint8_t baseLen) -> int {
        if (length == baseLen) return firstEnabled();
        if (length == baseLen + 1 && payload[baseLen] < NUM_FEEDERS &&
            feeders[payload[baseLen]].enabled) return payload[baseLen];
        return -1;
    };
    switch (command) {
        case CMD_SET_SPEED_PERCENT: {
            const int t = target(1);
            if (t >= 0 && payload[0] <= 100) {
                feeders[t].speedPercent = payload[0];
                persist.speedPercent[t] = payload[0];
                persistSave();          // only writes when the value changed
                writeFrame(RSP_ACK, &command, 1);
            } else writeFrame(RSP_NAK, &command, 1);
            break;
        }
        case CMD_SET_RUN: {
            const int t = target(1);
            if (t < 0) { writeFrame(RSP_NAK, &command, 1); break; }
            Feeder& f = feeders[t];
            f.runRequested = payload[0] != 0;
            if (!f.runRequested) {
                stopImmediately(f);
                f.state = STATE_PAUSED;
                f.stateSince = millis();
            } else if (f.state == STATE_IDLE_SHUTOFF) {
                f.lastFedTime = millis();
            }
            writeFrame(RSP_ACK, &command, 1);
            break;
        }
        case CMD_CLEAR_FAULT: {
            const int t = target(0);
            if (t < 0) { writeFrame(RSP_NAK, &command, 1); break; }
            Feeder& f = feeders[t];
            if (f.state == STATE_FAULT || f.state == STATE_IDLE_SHUTOFF) {
                f.lastRecoveryEnd = 0;
                f.stallActive = false;
                f.stallTimer = 0;
                f.lastFedTime = millis();
                f.idleWarnActive = false;
                f.idleWarnDismissed = false;
                f.useSlowStart = true;
                f.state = f.runRequested ? STATE_RUNNING : STATE_PAUSED;
                f.stateSince = millis();
            }
            writeFrame(RSP_ACK, &command, 1);
            break;
        }
        case CMD_SET_WARN_SECONDS: {
            const int t = target(2);
            if (t < 0) { writeFrame(RSP_NAK, &command, 1); break; }
            Feeder& f = feeders[t];
            uint16_t seconds;
            memcpy(&seconds, payload, 2);
            f.idleWarnMs = (uint32_t)seconds * 1000UL;
            f.idleWarnActive = false;
            f.idleWarnDismissed = false;
            f.lastFedTime = millis();
            persist.warnSeconds[t] = seconds;
            persistSave();
            Serial.printf("%c empty warning = %u s (LCD)\n", f.cfg->tag, seconds);
            writeFrame(RSP_ACK, &command, 1);
            break;
        }
        case CMD_SET_MACHINE:
            if (length == 1 && setMachine(payload[0], "LCD")) writeFrame(RSP_ACK, &command, 1);
            else writeFrame(RSP_NAK, &command, 1);
            break;
        case CMD_GET_STATUS:
            for (Feeder& f : feeders) if (f.enabled) sendStatus(f);
            break;
        case CMD_PING: {
            const uint32_t up = millis();
            writeFrame(RSP_PONG, &up, sizeof(up));
            break;
        }
        default: writeFrame(RSP_NAK, &command, 1); break;
    }
}

static void processLcdByte(uint8_t value) {
    switch (rx.stage) {
        case RxParser::WAIT_SOF: if (value == SOF) rx.stage = RxParser::READ_LEN; break;
        case RxParser::READ_LEN:
            if (value == 0 || value > MAX_PAYLOAD + 1) rx.stage = RxParser::WAIT_SOF;
            else { rx.length = value; rx.index = 0; rx.stage = RxParser::READ_BODY; }
            break;
        case RxParser::READ_BODY:
            rx.body[rx.index++] = value;
            if (rx.index >= rx.length) rx.stage = RxParser::CRC_LOW;
            break;
        case RxParser::CRC_LOW:  rx.receivedCrc = value; rx.stage = RxParser::CRC_HIGH; break;
        case RxParser::CRC_HIGH: rx.receivedCrc |= (uint16_t)value << 8; rx.stage = RxParser::WAIT_EOF; break;
        case RxParser::WAIT_EOF: {
            uint8_t crcData[MAX_PAYLOAD + 2];
            crcData[0] = rx.length;
            memcpy(&crcData[1], rx.body, rx.length);
            if (value == EOF_MARKER && crc16(crcData, rx.length + 1) == rx.receivedCrc) {
                handleCommand(rx.body[0], &rx.body[1], rx.length - 1);
            }
            rx.stage = RxParser::WAIT_SOF;
            break;
        }
    }
}

static void serviceLcd() {
    while (LCDSerial.available()) processLcdByte((uint8_t)LCDSerial.read());
    if (millis() - lastStatusAt >= STATUS_PERIOD_MS) {
        lastStatusAt = millis();
        for (Feeder& f : feeders) if (f.enabled) sendStatus(f);
    }
}

// ============================================================
//  MOTOR DRIVERS (TMC2209 over the shared UART)
// ============================================================
static bool initDriver(Feeder& f) {
    f.driverReady = false;
    f.drv->begin();
    if (f.drv->test_connection() != 0) return false;
    f.drv->pdn_disable(true);
    f.drv->mstep_reg_select(true);
    f.drv->I_scale_analog(false);
    f.drv->rms_current(RMS_CURRENT_MA, HOLD_CURRENT_MULT);
    f.drv->microsteps(MICROSTEPS);
    f.drv->en_spreadCycle(false);     // StealthChop: StallGuard4 works in this mode
    f.drv->pwm_autoscale(true);
    f.drv->pwm_autograd(true);
    f.drv->TCOOLTHRS(TCOOLTHRS_VALUE);
    f.drv->semin(0);
    f.drv->toff(3);
    f.appliedSgThreshold = f.bands[0].threshold;
    f.drv->SGTHRS(f.appliedSgThreshold);
    f.currentStallBand = 0;
    delay(20);
    const uint8_t sgRead = f.drv->SGTHRS();
    if (sgRead != f.appliedSgThreshold) {
        Serial.printf("%c driver verify failed: SGTHRS wrote %u, read %u\n", f.cfg->tag, f.appliedSgThreshold, sgRead);
        return false;
    }
    f.drv->GSTAT(0x07);
    // DIAG idles LOW (the driver sinks the open-drain pin). HIGH at
    // standstill means the jumper is missing and the pull-up is all that's
    // there; jam detection then relies on the polled reading alone.
    const bool diagWas = f.diagUsable;
    f.diagUsable = digitalRead(f.cfg->pinDiag) == LOW;
    if (!f.diagUsable) {
        Serial.printf("%c DIAG reads HIGH at standstill - is the DIAG jumper fitted?\n", f.cfg->tag);
    } else if (!diagWas) {
        Serial.printf("%c DIAG OK (jumper fitted)\n", f.cfg->tag);
    }
    f.driverReady = true;
    Serial.printf("%c motor driver ready\n", f.cfg->tag);
    return true;
}

// Reconfigure a driver that lost its settings (GSTAT reset flag, read twice
// to be sure) or never answered (24V off at boot, for example).
static void checkDriverHealth(Feeder& f) {
    if (millis() - f.lastDriverCheck < DRIVER_RECHECK_MS) return;
    f.lastDriverCheck = millis();
    uint8_t gstat = 0;
    if (f.driverReady) gstat = f.drv->GSTAT();
    if (f.driverReady && (gstat & 0x01)) {
        if (++f.resetReads < 2) return;
    } else {
        f.resetReads = 0;
    }
    if ((gstat & 0x01) || !f.driverReady) {
        f.resetReads = 0;
        if (f.driverReady) {
            Serial.printf("%c motor driver reset - reconfiguring\n", f.cfg->tag);
        } else if (!f.offlineLogged) {
            Serial.printf("%c motor driver not answering (24V on?) - retrying\n", f.cfg->tag);
            f.offlineLogged = true;
        }
        stopImmediately(f);
        f.driverReady = false;
        if (initDriver(f)) f.offlineLogged = false;
    }
}

// ============================================================
//  TEMPERATURES
// ============================================================
static const char* boardTempStr() {
    static char buf[12];
    if (isnan(boardTempC)) return "--";
    const long t10 = lroundf(boardTempC * 10.0f);
    const long a = labs(t10);
    snprintf(buf, sizeof(buf), "%s%ld.%ld", t10 < 0 ? "-" : "", a / 10, a % 10);
    return buf;
}

static const char* driverTempStr(uint8_t level) {
    switch (level) {
        case DRIVER_TEMP_OK:       return "OK (<120C)";
        case DRIVER_TEMP_120:      return ">=120C pre-warning";
        case DRIVER_TEMP_143:      return ">=143C";
        case DRIVER_TEMP_150:      return ">=150C";
        case DRIVER_TEMP_157:      return ">=157C";
        case DRIVER_TEMP_SHUTDOWN: return "OVERTEMP SHUTDOWN";
        default:                   return "?";
    }
}

// The TMC2209 only reports temperature thresholds, not a value. A change
// must read the same twice before it counts.
static void serviceDriverTemp(Feeder& f) {
    if (!f.driverReady) return;
    const uint32_t ds = f.drv->DRV_STATUS();
    uint8_t level = DRIVER_TEMP_OK;
    if      (ds & (1UL << 1))                level = DRIVER_TEMP_SHUTDOWN;
    else if (ds & (1UL << 11))               level = DRIVER_TEMP_157;
    else if (ds & (1UL << 10))               level = DRIVER_TEMP_150;
    else if (ds & (1UL << 9))                level = DRIVER_TEMP_143;
    else if (ds & ((1UL << 8) | (1UL << 0))) level = DRIVER_TEMP_120;
    if (level == f.driverTempLevel) { f.driverTempPending = level; return; }
    if (level != f.driverTempPending) { f.driverTempPending = level; return; }
    Serial.printf("%c driver temperature: %s -> %s (board %s C)\n", f.cfg->tag,
                  driverTempStr(f.driverTempLevel), driverTempStr(level), boardTempStr());
    f.driverTempLevel = level;
}

// Board temperature: the RP2040's own sensor, smoothed.
static void serviceTemps() {
    if (millis() - lastTempAt < TEMP_SAMPLE_MS) return;
    lastTempAt = millis();
    const float t = analogReadTemp();
    boardTempC = isnan(boardTempC) ? t : boardTempC + (t - boardTempC) * 0.2f;
    for (Feeder& f : feeders) if (f.enabled) serviceDriverTemp(f);
}

// ============================================================
//  STATE MACHINE (per feeder)
// ============================================================
static void enterState(Feeder& f, State next) {
    if (f.state == next) return;
    const State prev = f.state;
    f.state = next;
    f.stateSince = millis();
    // Starting a feed (not returning from a jam reverse) opens a new
    // "no case yet" window. A jam return keeps the real case timestamp so
    // telemetry still shows how long it has been since a case was counted.
    if (next == STATE_RUNNING && prev != STATE_JAM_PAUSE) {
        f.lastCountedCase = millis();
        f.atFeedSince = 0;
        if (prev != STATE_BEAM_BLOCKED) f.jamStreak = 0;
    }
}

// Same path as the timed empty shutoff: motor off, IDLE state, the LCD's
// "out of cases" notice and chime.
static void goIdleShutoff(Feeder& f, const char* reason) {
    Serial.printf("%c %s\n", f.cfg->tag, reason);
    stopImmediately(f);
    f.beamResetBreaks = 0;
    f.prevBeamState = readBeamRaw(f);
    f.stallActive = false;
    f.stallTimer = 0;
    f.idleWarnActive = false;
    f.sgZeroIsStall = false;
    f.diagFromChip = false;
    f.atFeedSince = 0;
    enterState(f, STATE_IDLE_SHUTOFF);
    sendAlert(f, ALERT_IDLE_SHUTOFF);
}

static void goFault(Feeder& f) {
    stopImmediately(f);
    f.beamResetBreaks = 0;
    f.prevBeamState = readBeamRaw(f);
    f.stallActive = false;
    f.stallTimer = 0;
    enterState(f, STATE_FAULT);
    const uint8_t payload[2] = { 1, f.index };
    writeFrame(EVT_FAULT, payload, 2);
    sendAlert(f, ALERT_JAM_FAULT);
}

static void startJamReverse(Feeder& f, bool noCase) {
    stopImmediately(f);
    setDirection(f, false);
    motorEnable(f, true);
    f.useSlowStart = false;
    f.targetHz = f.jamReverseHz;
    f.stallActive = false;
    f.stallTimer = 0;
    f.sgZeroIsStall = false;
    f.diagFromChip = false;
    f.atFeedSince = 0;
    f.noCaseRecovery = noCase;
    enterState(f, STATE_JAM_REVERSE);
}

// Shared path for StallGuard and for the beam jam. A second jam inside the
// repeat window is still a FAULT. Several recoveries with no counted case
// in between are an empty hopper: stop and show the empty shutoff instead
// of reversing again.
static void beginJamRecovery(Feeder& f, const char* reason, bool noCase) {
    const uint32_t now = millis();
    if (f.lastRecoveryEnd != 0 && now - f.lastRecoveryEnd < f.jamRepeatWindowMs) {
        Serial.printf("%c %s - jammed again %lu ms after recovery - FAULT\n", f.cfg->tag, reason,
                      (unsigned long)(now - f.lastRecoveryEnd));
        goFault(f);
        return;
    }
    if (f.emptyAfterJams > 0 && f.jamStreak >= f.emptyAfterJams) {
        char why[72];
        snprintf(why, sizeof(why), "empty - %u jam recoveries with no case, stopping", f.jamStreak);
        goIdleShutoff(f, why);
        return;
    }
    f.jamStreak++;
    Serial.printf("%c %s - JAM, reversing to clear (%u/%u)\n", f.cfg->tag, reason,
                  f.jamStreak, f.emptyAfterJams);
    startJamReverse(f, noCase);
}

static void handleRunning(Feeder& f) {
    if (!f.runRequested) { stopImmediately(f); enterState(f, STATE_PAUSED); return; }
    if (!f.driverReady)  { stopImmediately(f); return; }

    // A part waiting at the beam: hold until it is taken.
    if (f.beamBlocked) {
        Serial.printf("%c beam blocked - holding\n", f.cfg->tag);
        stopImmediately(f);
        f.stallActive = false;
        f.stallTimer = 0;
        enterState(f, STATE_BEAM_BLOCKED);
        return;
    }

    setDirection(f, true);
    motorEnable(f, true);
    f.targetHz = selectedStepHz(f);

    if (f.idleWarnMs > 0 && f.lastFedTime != 0 && !f.idleWarnActive && !f.idleWarnDismissed &&
        millis() - f.lastFedTime >= f.idleWarnMs) {
        Serial.printf("%c empty - nothing fed for %lu s (still running)\n",
                      f.cfg->tag, (unsigned long)(f.idleWarnMs / 1000));
        f.idleWarnActive = true;
        sendAlert(f, ALERT_EMPTY_WARNING);
    }

    if (f.idleShutoffMs > 0 && f.lastFedTime != 0 && millis() - f.lastFedTime >= f.idleShutoffMs) {
        char why[40];
        snprintf(why, sizeof(why), "idle %lu s - auto shutoff", (unsigned long)(f.idleShutoffMs / 1000));
        goIdleShutoff(f, why);
        return;
    }

    // Jam detection: only above the minimum speed and after the motor has
    // settled. A stall must persist for stallConfirmMs; a second jam within
    // jamRepeatWindowMs of a recovery is a FAULT.
    if (f.jamDetect && f.targetHz > f.jamMinSpeedHz && f.appliedHz > f.jamMinSpeedHz &&
        millis() - f.stallSettleStart >= f.stallSettleMs) {
        if (isStalled(f)) {
            if (!f.stallActive) {
                f.stallActive = true;
                f.stallTimer = millis();
                f.lastStallEvent = millis();
                Serial.printf("%c stall detected SG=%u diagPin=%d diagChip=%d - confirming\n",
                              f.cfg->tag, f.sgResult, diagPinHigh(f) ? 1 : 0, f.diagFromChip ? 1 : 0);
            } else {
                f.lastStallEvent = millis();
                if (millis() - f.stallTimer >= f.stallConfirmMs) {
                    beginJamRecovery(f, "stall", false);
                    return;
                }
            }
        } else if (f.stallActive && millis() - f.lastStallEvent >= STALL_CLEAR_MS) {
            f.stallActive = false;
            f.stallTimer = 0;
        }
    } else {
        f.stallActive = false;
        f.stallTimer = 0;
    }

    // Primary jam signal on a geared motor. StallGuard often never moves.
    // Time at feed speed with no new debounced case. Raw flicker does not
    // clear atFeedSince; only leaving feed speed, or a counted case, does.
    if (f.jamDetect && f.noNewCaseMs > 0 && runningAtFeedSpeed(f)) {
        const uint32_t now = millis();
        if (f.atFeedSince == 0) f.atFeedSince = now;
        if (now - f.atFeedSince >= f.noNewCaseMs) {
            char why[40];
            snprintf(why, sizeof(why), "no case for %lu ms", (unsigned long)f.noNewCaseMs);
            beginJamRecovery(f, why, true);
            return;
        }
    } else {
        f.atFeedSince = 0;
    }
}

static void resumeFromStop(Feeder& f, bool clearRecovery) {
    if (clearRecovery) f.lastRecoveryEnd = 0;
    f.stallActive = false;
    f.stallTimer = 0;
    f.lastFedTime = millis();
    f.idleWarnActive = false;
    f.idleWarnDismissed = false;
    f.useSlowStart = true;
    enterState(f, f.runRequested ? STATE_RUNNING : STATE_PAUSED);
}

static void serviceStateMachine(Feeder& f) {
    switch (f.state) {
        case STATE_RUNNING: handleRunning(f); break;
        case STATE_BEAM_BLOCKED:
            // A case that stays in the sensor is a jam. Require the beam to
            // still be blocked: the clear debounce after a real departure must
            // not count, and a one-sample flicker does not reset the clock.
            if (f.jamDetect && f.beamStuckMs > 0 && f.beamBlocked && f.rawBeam && f.beamBlockedSince != 0 &&
                millis() - f.beamBlockedSince >= f.beamStuckMs) {
                char why[40];
                snprintf(why, sizeof(why), "beam blocked %lu ms", (unsigned long)f.beamStuckMs);
                beginJamRecovery(f, why, true);
                break;
            }
            if (!f.beamBlocked) {
                if (!f.runRequested) { enterState(f, STATE_PAUSED); break; }
                Serial.printf("%c beam clear - resuming\n", f.cfg->tag);
                f.lastRecoveryEnd = 0;
                f.lastFedTime = millis();
                f.idleWarnActive = false;
                f.idleWarnDismissed = false;
                f.useSlowStart = true;
                enterState(f, STATE_RUNNING);
            }
            break;
        case STATE_JAM_REVERSE:
            if (millis() - f.stateSince >= f.cfg->jamReverseMs) {
                stopImmediately(f);
                enterState(f, STATE_JAM_PAUSE);
            }
            break;
        case STATE_JAM_PAUSE:
            if (millis() - f.stateSince >= JAM_PAUSE_MS) {
                Serial.printf("%c jam cleared - feeding forward\n", f.cfg->tag);
                f.lastRecoveryEnd = millis();
                // A stall recovery expects parts to start moving again. A
                // no-case recovery is "still nothing fed", so the empty
                // warning and idle shutoff keep the time they already had.
                if (!f.noCaseRecovery) {
                    f.lastFedTime = millis();
                    f.idleWarnActive = false;
                    f.idleWarnDismissed = false;
                }
                f.noCaseRecovery = false;
                f.useSlowStart = false;
                setDirection(f, true);
                enterState(f, STATE_RUNNING);
            }
            break;
        case STATE_FAULT:
            stopImmediately(f);
            if (millis() - f.lastFaultPrint >= 5000) {
                f.lastFaultPrint = millis();
                Serial.printf("%c FAULT - clear the jam, then RESUME on the LCD or break the beam %u times\n",
                              f.cfg->tag, BEAM_RESET_BREAKS);
            }
            if (checkBeamResetGesture(f)) {
                Serial.printf("%c resumed (beam gesture)\n", f.cfg->tag);
                resumeFromStop(f, true);
            }
            break;
        case STATE_IDLE_SHUTOFF:
            stopImmediately(f);
            if (millis() - f.lastIdlePrint >= 5000) {
                f.lastIdlePrint = millis();
                Serial.printf("%c IDLE SHUTOFF - reload, then RESUME on the LCD or break the beam %u times\n",
                              f.cfg->tag, BEAM_RESET_BREAKS);
            }
            if (checkBeamResetGesture(f)) {
                Serial.printf("%c resumed (beam gesture)\n", f.cfg->tag);
                resumeFromStop(f, false);
            }
            break;
        case STATE_PAUSED:
            stopImmediately(f);
            if (f.runRequested && f.driverReady) {
                Serial.printf("%c run\n", f.cfg->tag);
                f.lastFedTime = millis();
                f.idleWarnActive = false;
                f.idleWarnDismissed = false;
                f.useSlowStart = true;
                enterState(f, f.beamBlocked ? STATE_BEAM_BLOCKED : STATE_RUNNING);
            }
            break;
    }
}

// ============================================================
//  MACHINE (which feeders this board runs)
// ============================================================
static const char* machineName(uint8_t mask) {
    switch (mask) {
        case MACHINE_CASE:   return "CASE";
        case MACHINE_BULLET: return "BULLET";
        case MACHINE_DUAL:   return "DUAL (case + bullet)";
        default:             return "?";
    }
}

static int firstEnabled() {
    for (const Feeder& f : feeders) if (f.enabled) return f.index;
    return -1;
}

static bool feederStopped(const Feeder& f) {
    return f.state == STATE_PAUSED || f.state == STATE_FAULT || f.state == STATE_IDLE_SHUTOFF;
}

// Switch feeders on/off to match `mask`. Every feeder stops; a newly added
// one has its driver configured now. Refused while any feeder is moving.
static bool setMachine(uint8_t mask, const char* source) {
    if (mask == 0 || (mask & ~MACHINE_DUAL)) return false;
    for (const Feeder& f : feeders) {
        if (f.enabled && !feederStopped(f)) {
            Serial.printf("machine change refused (%s): stop %s first\n", source, f.cfg->name);
            return false;
        }
    }
    for (Feeder& f : feeders) {
        const bool want = (mask >> f.index) & 1U;
        f.runRequested = false;
        stopImmediately(f);
        f.stallActive = false;
        f.stallTimer = 0;
        f.state = STATE_PAUSED;
        f.stateSince = millis();
        if (want && !f.enabled) {
            f.enabled = true;
            f.offlineLogged = false;
            if (!initDriver(f)) Serial.printf("%c motor driver not ready - retrying\n", f.cfg->tag);
            f.rawBeam = f.prevBeamRunning = f.prevBeamState = readBeamRaw(f);
            f.beamBlocked = false;
            f.beamBrokenSince = f.beamClearedSince = f.beamBlockedSince = f.countBlockSince = 0;
            f.lastFedTime = millis();
        } else if (!want) {
            f.enabled = false;
            f.driverReady = false;
        }
    }
    if (!feeders[selected].enabled) selected = (uint8_t)firstEnabled();
    persist.machineMask = mask;
    persistSave();
    Serial.printf("machine set to %s (%s)\n", machineName(mask), source);
    lastStatusAt = 0;   // tell the LCD straight away
    return true;
}

// ============================================================
//  SERIAL CONSOLE (USB, 115200)
//  Setup and tuning commands apply to the SELECTED feeder: `U 0` = case,
//  `U 1` = bullet.
// ============================================================
static void printFeeder(Feeder& f) {
    Serial.printf("--- %s (feeder %u, %u:1) %s ---\n", f.cfg->name, f.index, f.cfg->gearbox,
                  f.index == selected ? "<-- selected (U)" : "");
    Serial.printf("  state:        %s\n", stateName(f.state));
    Serial.printf("  driver:       %s, DIAG %s, temp %s\n", f.driverReady ? "ready" : "NOT READY",
                  f.diagUsable ? "in use" : "NOT used", driverTempStr(f.driverTempLevel));
    Serial.printf("  motor dir:    %s\n", f.motorReversed ? "reversed (M 1)" : "normal (M 0)");
    Serial.printf("  speed:        %u%% -> %lu Hz (range %lu..%lu)\n", f.speedPercent,
                  (unsigned long)selectedStepHz(f), (unsigned long)f.speedMinHz, (unsigned long)f.speedMaxHz);
    Serial.printf("  step Hz now:  %lu (target %lu)\n", (unsigned long)f.appliedHz, (unsigned long)f.targetHz);
    Serial.print("  jam bands:   ");
    for (uint8_t i = 0; i < f.numBands; i++) {
        Serial.printf(" [%u] >=%lu:%u%s", i, (unsigned long)f.bands[i].speedMinHz, f.bands[i].threshold,
                      i == (uint8_t)f.currentStallBand ? "*" : "");
    }
    Serial.println();
    Serial.printf("  StallGuard:   %u (trip <= %u)%s, jam detection %s\n", f.sgResult,
                  f.appliedSgThreshold * 2, f.sgZeroIsStall ? ", full stall" : "",
                  f.jamDetect ? "on" : "OFF");
    Serial.printf("  DIAG:         pin %s, chip %s\n",
                  digitalRead(f.cfg->pinDiag) == HIGH ? "HIGH" : "LOW",
                  f.diagFromChip ? "HIGH" : "low");
    Serial.printf("  jam:          min %lu Hz, reverse %lu Hz for %lu ms, confirm %u ms\n",
                  (unsigned long)f.jamMinSpeedHz, (unsigned long)f.jamReverseHz,
                  (unsigned long)f.cfg->jamReverseMs, f.stallConfirmMs);
    const uint32_t caseAge = f.lastCountedCase ? millis() - f.lastCountedCase : 0;
    Serial.printf("  last case:    %lu ms ago\n", (unsigned long)caseAge);
    Serial.printf("  no-case jam:  %lu ms at feed speed with no counted case%s\n",
                  (unsigned long)f.noNewCaseMs, f.noNewCaseMs == 0 ? " (off)" : "");
    const uint32_t stuckFor = (f.beamBlocked && f.beamBlockedSince) ? millis() - f.beamBlockedSince : 0;
    Serial.printf("  beam stuck:   %lu ms (%lu ms so far)%s\n",
                  (unsigned long)f.beamStuckMs, (unsigned long)stuckFor,
                  f.beamStuckMs == 0 ? " (off)" : "");
    Serial.printf("  empty after:  %u jam recoveries with no case (streak %u)%s\n",
                  f.emptyAfterJams, f.jamStreak, f.emptyAfterJams == 0 ? " (off)" : "");
    Serial.printf("  beam:         %s (raw %s), blocked reads %s, hold confirm/clear %u/%u ms\n",
                  f.beamBlocked ? "BLOCKED" : "clear", f.rawBeam ? "blocked" : "clear",
                  f.beamActiveLow ? "LOW" : "HIGH", f.beamConfirmMs, f.beamClearMs);
    Serial.printf("  count:        confirm %u ms, re-arm %u ms, total %u\n",
                  f.countConfirmMs, f.countClearMs, f.count);
    Serial.printf("  empty warn / shutoff: %lu / %lu ms, count %u, run requested %s\n",
                  (unsigned long)f.idleWarnMs, (unsigned long)f.idleShutoffMs, f.count,
                  f.runRequested ? "yes" : "no");
}

static void printStatus() {
    Serial.println("=== Case Feeder Pro controller ===");
    Serial.printf("  machine:      %s\n", machineName(persist.machineMask));
    Serial.printf("  uptime:       %lu s, board temp %s C, motor current %u mA\n",
                  (unsigned long)(millis() / 1000), boardTempStr(), RMS_CURRENT_MA);
    Serial.printf("  LCD:          %s\n", (millis() - lastLcdRxAt < LCD_TIMEOUT_MS) ? "connected" : "not connected");
    Serial.printf("  telemetry / beam log: %s / %s\n", telemetryOn ? "on" : "off", beamLogOn ? "on" : "off");
    for (Feeder& f : feeders) if (f.enabled) printFeeder(f);
    Serial.println("setup:    ? | K <1 case, 2 bullet, 3 dual> | U <0 case, 1 bullet> | R <0/1> | N <0/1> | M <0/1>");
    Serial.println("advanced: t <0/1> | e <0/1> | j <0/1> | s | v | V | b | B | c | C | w | W | d | n | p | E | i | o | g | r | a | A | D | T | F");
}

static void handleSerial() {
    static char buf[32];
    static size_t len = 0;
    static bool overflow = false;
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == '\r') continue;
        if (c != '\n') {
            if (overflow) continue;
            if (len >= sizeof(buf) - 1) { overflow = true; continue; }
            buf[len++] = c;
            continue;
        }
        if (overflow) { Serial.println("line too long - ignored"); overflow = false; len = 0; continue; }
        buf[len] = '\0';
        if (len > 1 && buf[1] != ' ') { Serial.println("format: <command> <value>. ? for help."); len = 0; continue; }
        if (len > 0) {
            const char cmd = buf[0];
            const long val = (len > 2) ? atol(&buf[2]) : 0;
            Feeder& f = feeders[selected];
            const char tg = f.cfg->tag;
            switch (cmd) {
                case '?': printStatus(); break;
                case 'K':
                    if (!setMachine((uint8_t)val, "serial"))
                        Serial.println("K: 1 = case, 2 = bullet, 3 = dual - stop every feeder first");
                    break;
                case 'U':
                    if (val >= 0 && val < NUM_FEEDERS && feeders[val].enabled) { selected = (uint8_t)val;
                        Serial.printf("selected feeder %ld (%s)\n", val, feeders[selected].cfg->name); }
                    else Serial.println("U: 0 = case, 1 = bullet (must be part of this machine)");
                    break;
                case 'R':
                    if (val != 0) { f.runRequested = true; Serial.printf("%c run (serial)\n", tg); }
                    else { f.runRequested = false; stopImmediately(f); enterState(f, STATE_PAUSED);
                           Serial.printf("%c stopped (serial)\n", tg); }
                    break;
                case 'N':
                    if (val < 0 || val > 1) { Serial.println("N: 0 = blocked reads LOW, 1 = blocked reads HIGH"); break; }
                    f.beamActiveLow = (val == 0);
                    persist.beamActiveHigh[f.index] = f.beamActiveLow ? 0 : 1;
                    persistSave();
                    f.rawBeam = f.prevBeamRunning = f.prevBeamState = readBeamRaw(f);
                    f.beamBrokenSince = f.beamClearedSince = f.beamBlockedSince = f.countBlockSince = 0;
                    f.beamBlocked = false;
                    Serial.printf("%c beam: blocked reads %s\n", tg, f.beamActiveLow ? "LOW" : "HIGH");
                    break;
                case 'M':
                    if (val < 0 || val > 1) { Serial.println("M: 0 = normal, 1 = reversed"); break; }
                    if (f.motorPowered) { Serial.println("M: stop that feeder first"); break; }
                    f.motorReversed = (val == 1);
                    persist.motorReversed[f.index] = f.motorReversed ? 1 : 0;
                    persistSave();
                    setDirection(f, f.dirForward);
                    Serial.printf("%c motor direction %s\n", tg, f.motorReversed ? "reversed" : "normal");
                    break;
                // ---- advanced: diagnostics and jam tuning (not saved) ----
                case 't': telemetryOn = (val != 0); Serial.printf("telemetry %s\n", telemetryOn ? "on" : "off"); break;
                case 'e': beamLogOn = (val != 0);   Serial.printf("beam edge log %s\n", beamLogOn ? "on" : "off"); break;
                case 'j': f.jamDetect = (val != 0); Serial.printf("%c jam detection %s\n", tg, f.jamDetect ? "on" : "off"); break;
                case 's':
                    if (val >= 0 && val <= 255) {
                        for (uint8_t i = 0; i < f.numBands; i++) f.bands[i].threshold = (uint8_t)val;
                        f.currentStallBand = -1;
                        Serial.printf("%c all jam bands = %ld (trip <= %ld)\n", tg, val, val * 2);
                    } else Serial.println("s: 0..255");
                    break;
                case 'v':
                    if (val > (long)f.speedMinHz && val <= (long)SPEED_HZ_CEILING) { f.speedMaxHz = (uint32_t)val;
                        Serial.printf("%c max speed = %ld Hz\n", tg, val); }
                    else Serial.println("v: above the min speed, up to 15000");
                    break;
                case 'V':
                    if (val > 0 && val < (long)f.speedMaxHz) { f.speedMinHz = (uint32_t)val;
                        Serial.printf("%c min speed = %ld Hz\n", tg, val); }
                    else Serial.println("V: above 0, below the max speed");
                    break;
                case 'b': if (val >= 0 && val <= 10000) { f.beamConfirmMs = (uint16_t)val; Serial.printf("%c beam confirm = %ld ms\n", tg, val); } break;
                case 'B': if (val >= 0 && val <= 10000) { f.beamClearMs = (uint16_t)val;   Serial.printf("%c beam clear = %ld ms\n", tg, val); } break;
                case 'c': if (val >= 0 && val <= 2000)  { f.countConfirmMs = (uint16_t)val; Serial.printf("%c count confirm = %ld ms\n", tg, val); } break;
                case 'C': if (val >= 0 && val <= 2000)  { f.countClearMs = (uint16_t)val;   Serial.printf("%c count re-arm = %ld ms\n", tg, val); } break;
                case 'w': if (val >= 100 && val <= 60000) { f.jamRepeatWindowMs = (uint16_t)val; Serial.printf("%c repeat-jam window = %ld ms\n", tg, val); } break;
                case 'W': if (val >= 50 && val <= 5000) { f.stallConfirmMs = (uint16_t)val; Serial.printf("%c stall confirm = %ld ms\n", tg, val); } break;
                case 'd': if (val >= 0 && val <= 5000)  { f.stallSettleMs = (uint16_t)val;  Serial.printf("%c stall settle = %ld ms\n", tg, val); } break;
                case 'n':
                    if (val == 0 || (val >= 500 && val <= 120000)) {
                        f.noNewCaseMs = (uint32_t)val;
                        f.atFeedSince = 0;
                        Serial.printf("%c no-case jam = %ld ms%s\n", tg, val, val == 0 ? " (off)" : "");
                    } else Serial.println("n: 0 = off, or 500..120000 ms");
                    break;
                case 'p':
                    if (val == 0 || (val >= 500 && val <= 120000)) {
                        f.beamStuckMs = (uint32_t)val;
                        Serial.printf("%c beam-stuck jam = %ld ms%s\n", tg, val, val == 0 ? " (off)" : "");
                    } else Serial.println("p: 0 = off, or 500..120000 ms");
                    break;
                case 'E':
                    if (val >= 0 && val <= 20) {
                        f.emptyAfterJams = (uint8_t)val;
                        Serial.printf("%c empty after %ld jam recoveries with no case%s\n", tg, val,
                                      val == 0 ? " (off)" : "");
                    } else Serial.println("E: 0 = off, or 1..20");
                    break;
                case 'i':
                    if (val == 0 || (val >= 1000 && val <= 600000)) { f.idleShutoffMs = (uint32_t)val; f.lastFedTime = millis();
                        Serial.printf("%c idle shutoff = %ld ms\n", tg, val); }
                    break;
                case 'o':
                    if (val == 0 || (val >= 1000 && val <= 600000)) { f.idleWarnMs = (uint32_t)val;
                        f.idleWarnActive = false; f.idleWarnDismissed = false; f.lastFedTime = millis();
                        Serial.printf("%c empty warning = %ld ms\n", tg, val); }
                    break;
                case 'g': if (val >= 0 && val < (long)SPEED_HZ_CEILING) { f.jamMinSpeedHz = (uint32_t)val; Serial.printf("%c jam min speed = %ld Hz\n", tg, val); } break;
                case 'r': if (val >= 500 && val <= (long)SPEED_HZ_CEILING) { f.jamReverseHz = (uint32_t)val; Serial.printf("%c jam reverse speed = %ld Hz\n", tg, val); } break;
                case 'a': if (val >= 1000 && val <= 1000000) { f.accelHzPerSec = (uint32_t)val; Serial.printf("%c accel = %ld Hz/s\n", tg, val); } break;
                case 'A': if (val >= 1000 && val <= 1000000) { f.startupAccelHzPerSec = (uint32_t)val; Serial.printf("%c startup accel = %ld Hz/s\n", tg, val); } break;
                case 'D': if (val >= 1000 && val <= 1000000) { f.decelHzPerSec = (uint32_t)val; Serial.printf("%c decel = %ld Hz/s\n", tg, val); } break;
                case 'T':
                    if (f.driverReady && f.state == STATE_RUNNING) { Serial.printf("%c manual jam test\n", tg); startJamReverse(f, false); }
                    else Serial.println("T: only while running");
                    break;
                case 'F': Serial.printf("%c manual FAULT\n", tg); goFault(f); break;
                default: Serial.println("unknown command. ? for help."); break;
            }
        }
        len = 0;
    }
}

static void updateTelemetry() {
    if (!telemetryOn || millis() - lastTelemetry < TELEMETRY_PERIOD_MS) return;
    lastTelemetry = millis();
    for (Feeder& f : feeders) {
        if (!f.enabled) continue;
        const uint32_t caseAge = f.lastCountedCase ? millis() - f.lastCountedCase : 0;
        const int diag = (diagPinHigh(f) || f.diagFromChip) ? 1 : 0;
        Serial.printf("%c SG:%4u [min %4u max %4u]  trip<=%u  hz:%6lu  target:%6lu  state:%d  beam:%d  diag:%d  case:%lu  empty:%u/%u\n",
                      f.cfg->tag, f.sgResult, f.sgWindowMin == 0xFFFF ? 0 : f.sgWindowMin, f.sgWindowMax,
                      f.appliedSgThreshold * 2, (unsigned long)f.appliedHz, (unsigned long)f.targetHz,
                      (int)f.state, (int)f.beamBlocked, diag, (unsigned long)caseAge,
                      f.jamStreak, f.emptyAfterJams);
        f.sgWindowMin = 0xFFFF;
        f.sgWindowMax = 0;
    }
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
    feederInit(feeders[0], 0, &CASE_CFG, &drvCase);
    feederInit(feeders[1], 1, &BULLET_CFG, &drvBullet);

    // First: every motor off.
    for (Feeder& f : feeders) {
        pinMode(f.cfg->pinEnN, OUTPUT);  digitalWrite(f.cfg->pinEnN, HIGH);
        pinMode(f.cfg->pinStep, OUTPUT); digitalWrite(f.cfg->pinStep, LOW);
        pinMode(f.cfg->pinDir, OUTPUT);  digitalWrite(f.cfg->pinDir, HIGH);
        // Open-drain DIAG idles low and releases on a stall. Without the
        // pull-up the Pico pin floats and a stall never reads HIGH.
        pinMode(f.cfg->pinDiag, INPUT_PULLUP);
        pinMode(f.cfg->pinBeam, INPUT);
    }
    for (uint8_t p : PINS_UNUSED_DRIVER_EN) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
    for (uint8_t p : PINS_PARKED_LOW)       { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    pinMode(PIN_FAN, OUTPUT);
    digitalWrite(PIN_FAN, HIGH);

    statusPixel.begin();
    statusPixel.setBrightness(LED_BRIGHTNESS);
    statusPixel.clear();
    statusPixel.show();

    Serial.begin(115200);
    delay(50);
    Serial.println("\n=== Case Feeder Pro controller ===");

    persistLoad();
    for (Feeder& f : feeders) {
        f.enabled       = (persist.machineMask >> f.index) & 1U;
        f.beamActiveLow = persist.beamActiveHigh[f.index] == 0;
        f.motorReversed = persist.motorReversed[f.index] != 0;
        f.idleWarnMs    = (uint32_t)persist.warnSeconds[f.index] * 1000UL;
        const uint8_t sp = persist.speedPercent[f.index];
        f.speedPercent  = (sp >= 1 && sp <= 100) ? sp : DEFAULT_SPEED_PERCENT;
        setDirection(f, true);
    }

    for (Feeder& f : feeders) stepPwmInit(f);

    LCDSerial.setTX(PIN_LCD_TX);
    LCDSerial.setRX(PIN_LCD_RX);
    LCDSerial.setFIFOSize(256);
    LCDSerial.begin(BAUD);

    TMCSerial.setTX(PIN_TMC_TX);
    TMCSerial.setRX(PIN_TMC_RX);
    TMCSerial.begin(115200);
    delay(20);
    for (Feeder& f : feeders) {
        if (!f.enabled) continue;
        if (!initDriver(f)) Serial.printf("%c motor driver not ready at boot - retrying\n", f.cfg->tag);
        f.rawBeam = f.prevBeamRunning = f.prevBeamState = readBeamRaw(f);
        f.lastFedTime = millis();
        f.stateSince = millis();
        f.state = STATE_PAUSED;
    }
    if (!feeders[selected].enabled) selected = (uint8_t)firstEnabled();

    rp2040.wdt_begin(LOOP_WDT_MS);   // a stuck loop restarts the board with the motors off
    Serial.printf("ready: machine %s, stopped. Start each feeder from the LCD.\n", machineName(persist.machineMask));
    printStatus();
}

void loop() {
    rp2040.wdt_reset();
    serviceLcd();
    for (Feeder& f : feeders) if (f.enabled) serviceBeam(f);
    for (Feeder& f : feeders) if (f.enabled) serviceStateMachine(f);
    for (Feeder& f : feeders) if (f.enabled) serviceMotorRamp(f);
    for (Feeder& f : feeders) if (f.enabled) serviceStallThreshold(f);
    for (Feeder& f : feeders) if (f.enabled) checkDriverHealth(f);

    for (Feeder& f : feeders) {
        if (f.enabled && f.driverReady && f.state == STATE_RUNNING && millis() - f.lastSgRead >= SG_SAMPLE_MS) {
            f.lastSgRead = millis();
            sampleStallGuard(f);
        }
    }

    serviceLed();
    handleSerial();
    serviceTemps();
    updateTelemetry();
}
