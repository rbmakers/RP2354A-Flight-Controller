#include "rc_calibration.h"
#include <math.h>

// ================================================================
// Default calibration (CRSF raw range)
// ================================================================

static const RCCalChannel DEFAULT_CAL = {
    .rawMin   = 172,
    .rawMid   = 992,
    .rawMax   = 1811,
    .deadband = 0.02f,
    .expo     = 0.20f
};

static RCCalChannel _cal[RC_CAL_CHANNELS];
static bool         _active = false;
static uint32_t     _calStartMs = 0;

// Temporary min/max accumulators during sweep
static uint16_t _sweepMin[RC_CAL_CHANNELS];
static uint16_t _sweepMax[RC_CAL_CHANNELS];

// ================================================================
// Expo helper
// ================================================================
// Standard RC expo: y = x * (1-e) + x^3 * e
// where x is the input (-1..+1) and e is the expo coefficient.

static float applyExpo(float x, float expo)
{
    if (expo <= 0.0f) return x;
    float absX = fabsf(x);
    float sign  = (x >= 0.0f) ? 1.0f : -1.0f;
    return sign * (absX * (1.0f - expo) + absX * absX * absX * expo);
}

// ================================================================
// Deadband helper
// ================================================================
// Collapses the band [-db, +db] to exactly 0, then rescales
// the remaining range to fill -1..+1.

static float applyDeadband(float x, float db)
{
    if (db <= 0.0f) return x;
    if (fabsf(x) < db) return 0.0f;
    float sign  = (x >= 0.0f) ? 1.0f : -1.0f;
    return sign * (fabsf(x) - db) / (1.0f - db);
}

// ================================================================
// Public API
// ================================================================

void rcCalInit()
{
    for (uint8_t i = 0; i < RC_CAL_CHANNELS; i++)
        _cal[i] = DEFAULT_CAL;
    _active = false;
}

void rcCalBegin()
{
    _active     = true;
    _calStartMs = millis();
    for (uint8_t i = 0; i < RC_CAL_CHANNELS; i++)
    {
        _sweepMin[i] = 2047;   // will be lowered
        _sweepMax[i] = 0;      // will be raised
    }
    Serial.println("[RC_CAL] Calibration started — move all sticks to extremes");
}

void rcCalFeed(const uint16_t rawCh[RC_CAL_CHANNELS])
{
    if (!_active) return;
    for (uint8_t i = 0; i < RC_CAL_CHANNELS; i++)
    {
        if (rawCh[i] < _sweepMin[i]) _sweepMin[i] = rawCh[i];
        if (rawCh[i] > _sweepMax[i]) _sweepMax[i] = rawCh[i];
    }
}

void rcCalEnd()
{
    if (!_active) return;
    _active = false;

    for (uint8_t i = 0; i < RC_CAL_CHANNELS; i++)
    {
        // Sanity check: sweep must have covered at least 200 counts
        uint16_t span = _sweepMax[i] - _sweepMin[i];
        if (span < 200)
        {
            Serial.print("[RC_CAL] CH");
            Serial.print(i + 1);
            Serial.println(" sweep too small — keeping defaults");
            continue;
        }

        _cal[i].rawMin = _sweepMin[i];
        _cal[i].rawMax = _sweepMax[i];
        _cal[i].rawMid = (_sweepMin[i] + _sweepMax[i]) / 2;
    }

    rcCalPrint();
}

bool rcCalActive() { return _active; }

float rcCalApply(uint8_t chIdx, uint16_t rawValue, bool isThrottle)
{
    if (chIdx >= RC_CAL_CHANNELS) return 0.0f;
    const RCCalChannel &c = _cal[chIdx];

    float norm;

    if (isThrottle)
    {
        // Map rawMin → 0.0, rawMax → 1.0
        norm = (float)(rawValue - c.rawMin) /
               (float)(c.rawMax - c.rawMin);
        norm = constrain(norm, 0.0f, 1.0f);
        // No deadband/expo on throttle — apply directly
        return norm;
    }
    else
    {
        // Two-sided mapping: rawMid → 0, rawMin → -1, rawMax → +1
        if (rawValue >= c.rawMid)
            norm = (float)(rawValue - c.rawMid) /
                   (float)(c.rawMax   - c.rawMid);
        else
            norm = -(float)(c.rawMid  - rawValue) /
                    (float)(c.rawMid   - c.rawMin);

        norm = constrain(norm, -1.0f, 1.0f);
        norm = applyDeadband(norm, c.deadband);
        norm = applyExpo(norm, c.expo);
        return norm;
    }
}

RCCalChannel rcCalGet(uint8_t ch)
{
    return (ch < RC_CAL_CHANNELS) ? _cal[ch] : DEFAULT_CAL;
}

void rcCalSet(uint8_t ch, RCCalChannel cal)
{
    if (ch < RC_CAL_CHANNELS) _cal[ch] = cal;
}

void rcCalPrint()
{
    Serial.println("[RC_CAL] Channel calibration table:");
    Serial.println("  CH  rawMin  rawMid  rawMax  deadband  expo");
    for (uint8_t i = 0; i < RC_CAL_CHANNELS; i++)
    {
        const RCCalChannel &c = _cal[i];
        Serial.print("  CH"); Serial.print(i + 1);
        Serial.print("  "); Serial.print(c.rawMin);
        Serial.print("   "); Serial.print(c.rawMid);
        Serial.print("   "); Serial.print(c.rawMax);
        Serial.print("   "); Serial.print(c.deadband, 2);
        Serial.print("    "); Serial.println(c.expo, 2);
    }
}
