#include "altitude_hold.h"
#include "altitude_estimator.h"
#include "../config.h"

static float _target   = 0.0f;
static float _integral = 0.0f;
static float _lastErr  = 0.0f;

void altHoldInit()
{
    _target   = altEstGetAltitude();
    _integral = 0.0f;
    _lastErr  = 0.0f;
}

void altHoldSetTarget(float altMetres)
{
    _target   = altMetres;
    _integral = 0.0f;  // reset windup when target changes
    _lastErr  = 0.0f;
}

float altHoldGetTarget() { return _target; }

float altHoldUpdate(float throttleIn, float dt)
{
    float alt = altEstGetAltitude();
    float err = _target - alt;

    _integral += err * dt;
    _integral  = constrain(_integral,
                           -ALT_INTEGRAL_LIMIT, ALT_INTEGRAL_LIMIT);

    float derivative = (err - _lastErr) / dt;
    _lastErr = err;

    float correction = ALT_KP * err
                     + ALT_KI * _integral
                     + ALT_KD * derivative;

    correction = constrain(correction,
                           -ALT_OUTPUT_LIMIT, ALT_OUTPUT_LIMIT);

    return constrain(throttleIn + correction, 0.0f, 1.0f);
}
