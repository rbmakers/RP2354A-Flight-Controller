#pragma once
#include <Arduino.h>

// ================================================================
// RC Channel Calibration
// ================================================================
// Stores per-channel min/mid/max CRSF raw values (172–1811) and
// applies them to produce accurate normalized outputs (-1.0 to +1.0
// for sticks, 0.0 to 1.0 for throttle).
//
// Also applies:
//   - Centre deadband  (removes stick drift near centre)
//   - Expo curve       (softens stick response near centre)
//
// Calibration procedure:
//   1. Call rcCalBegin() from the USB command "RC_CAL_START"
//   2. Move ALL sticks to FULL EXTENTS slowly over 5 seconds
//   3. Call rcCalEnd()   from the USB command "RC_CAL_END"
//   4. Values are stored in-memory (extend with Flash NVM save)
//
// Default values match ELRS/CRSF protocol raw range:
//   Min = 172, Mid = 992, Max = 1811
// ================================================================

#define RC_CAL_CHANNELS  8        // calibrate CH1–CH8

typedef struct
{
    uint16_t rawMin;
    uint16_t rawMid;
    uint16_t rawMax;
    float    deadband;     // normalized, applied at centre (default 0.02)
    float    expo;         // 0.0 = linear, 1.0 = full cubic expo (default 0.20)
} RCCalChannel;

// Initialize with CRSF defaults
void rcCalInit();

// Begin collecting min/max sweep (5 second window)
void rcCalBegin();

// Feed raw CRSF values during calibration sweep
void rcCalFeed(const uint16_t rawCh[RC_CAL_CHANNELS]);

// Finish calibration and lock in values
void rcCalEnd();

bool rcCalActive();

// Apply calibration + deadband + expo to one raw CRSF value.
// chIdx 0-based.  isThrottle=true maps 0.0–1.0 instead of -1.0–1.0.
float rcCalApply(uint8_t chIdx, uint16_t rawValue, bool isThrottle);

// Get/set calibration for one channel (for GUI display/editing)
RCCalChannel rcCalGet(uint8_t ch);
void         rcCalSet(uint8_t ch, RCCalChannel cal);

// Print calibration table to Serial
void rcCalPrint();
