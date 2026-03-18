#include "altitude_estimator.h"
#include "bmp580_driver.h"
#include <math.h>

// Barometer correction weight per sample.
// Small value → slow baro correction but smooth altitude.
// Tune between 0.005 (smooth) and 0.05 (fast correction).
static const float BARO_ALPHA = 0.02f;

// 1 g in m/s² — used to remove gravity from accel Z
static const float G_MS2 = 9.80665f;

static float _altitude     = 0.0f;   // metres
static float _velocity     = 0.0f;   // m/s  (positive up)
static float _seaLevelPa   = 101325.0f;
static float _baroAlt      = 0.0f;

static float _pressureToAlt(float pa)
{
    if (pa <= 0.0f) return _altitude;
    return 44330.0f * (1.0f - powf(pa / _seaLevelPa, 0.1903f));
}

void altEstInit(float firstPressurePa)
{
    _baroAlt  = _pressureToAlt(firstPressurePa);
    _altitude = _baroAlt;
    _velocity = 0.0f;
}

void altEstUpdate(float accZg, float pressurePa, float dt)
{
    // Convert accel from g to m/s², remove gravity (board level: az ≈ 1 g)
    float accZ_ms2 = (accZg - 1.0f) * G_MS2;

    // Integrate acceleration → velocity → position
    _velocity += accZ_ms2 * dt;
    _altitude += _velocity  * dt;

    // Barometer correction (complementary filter)
    _baroAlt   = _pressureToAlt(pressurePa);
    float altErr = _baroAlt - _altitude;

    // Correct both altitude and velocity (gain-scheduled)
    _altitude += BARO_ALPHA * altErr;
    _velocity += BARO_ALPHA * altErr * (1.0f / dt) * 0.05f;  // gentle vel correction
}

float altEstGetAltitude()      { return _altitude; }
float altEstGetVerticalSpeed() { return _velocity; }
