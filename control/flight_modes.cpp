#include "flight_modes.h"
#include "../config.h"

static FlightMode _mode  = MODE_ACRO;
static bool       _armed = false;

// Debounce timers
static uint32_t _armDebounceMs    = 0;
static uint32_t _switchDebounceMs = 0;
static uint32_t _modeDebounceMs   = 0;
#define DEBOUNCE_MS  250

void  flightModeSet(FlightMode m) { _mode  = m; }
FlightMode flightModeGet()        { return _mode; }

void motorArm()
{
    _armed = true;
    Serial.println("[FC] ARMED");
    Serial.print("[FC] Inner PID  Kp="); Serial.print(PID_ROLL_KP, 3);
    Serial.print("  Ki=");               Serial.print(PID_ROLL_KI, 3);
    Serial.print("  Kd=");               Serial.print(PID_ROLL_KD, 3);
    Serial.print("  Kff=");              Serial.println(PID_ROLL_KFF, 3);
    Serial.print("[FC] Outer Kp=");      Serial.print(OUTER_LOOP_KP, 1);
    Serial.print("  MaxAngle=");         Serial.print(MAX_ANGLE_DEG, 0);
    Serial.print("°  MaxRate=");         Serial.print(MAX_RATE_DPS, 0);
    Serial.println("°/s");
    Serial.print("[FC] I-Relax cutoff="); Serial.print(ITERM_RELAX_CUTOFF, 2);
    Serial.print(" rad/s  TPA break=");  Serial.print(TPA_BREAKPOINT, 2);
    Serial.print("  TPA rate=");         Serial.print(TPA_RATE, 2);
    Serial.print("  Airmode=");          Serial.println(AIRMODE_ENABLED ? "ON" : "OFF");
}

void motorDisarm()
{
    _armed = false;
    Serial.println("[FC] DISARMED");
}

bool isArmed() { return _armed; }

// ----------------------------------------------------------------
// Acro: stick → rate setpoint (deg/s), converted to rad/s by caller
//
// Expo curve: y = x*(1−e) + x³*e
//   ACRO_EXPO = 0.0 → perfectly linear (default)
//   ACRO_EXPO = 0.3 → softer centre, same max rate
//   Tune in config.h by adding:  #define ACRO_EXPO  0.30f
// ----------------------------------------------------------------
#ifndef ACRO_EXPO
#define ACRO_EXPO  0.0f
#endif

float acroRateSetpoint(float stick)
{
    float x = stick;
    float expo = ACRO_EXPO;

    // Apply cubic expo when enabled
    float shaped = x * (1.0f - expo) + x * x * x * expo;

    return shaped * MAX_RATE_DPS;
}

// ----------------------------------------------------------------
// Angle: stick → rate setpoint via outer P loop
//
// Outer P gain (OUTER_LOOP_KP) is defined in config.h alongside the
// inner-loop gains so all PID values live in one place.
//
// Interpretation: OUTER_LOOP_KP = 6 means a 10° tilt error generates
// a 60 deg/s rate command to the inner loop.
// ----------------------------------------------------------------
float angleModeSetpoint(float stick, float currentAngleDeg)
{
    float targetAngle = stick * MAX_ANGLE_DEG;
    float angleError  = targetAngle - currentAngleDeg;
    return constrain(OUTER_LOOP_KP * angleError, -MAX_RATE_DPS, MAX_RATE_DPS);
}

// ----------------------------------------------------------------
// Stick-based arm / disarm (Betaflight convention)
void checkArmDisarm(float throttleNorm, float yawNorm)
{
    uint32_t now = millis();

    if (!_armed)
    {
        if (throttleNorm < ARM_THROTTLE_MAX && yawNorm > ARM_YAW_THRESHOLD)
        {
            if (now - _armDebounceMs > DEBOUNCE_MS)
            {
                motorArm();
                _armDebounceMs = now;
            }
        }
    }
    else
    {
        if (throttleNorm < ARM_THROTTLE_MAX && yawNorm < -ARM_YAW_THRESHOLD)
        {
            if (now - _armDebounceMs > DEBOUNCE_MS)
            {
                motorDisarm();
                _armDebounceMs = now;
            }
        }
    }
}

// ----------------------------------------------------------------
// Switch-based arm (CH5): >1700 µs = armed, <1300 µs = disarmed
void checkArmSwitch(uint16_t ch5us)
{
    uint32_t now = millis();
    if (now - _switchDebounceMs < DEBOUNCE_MS) return;

    if (ch5us > 1700 && !_armed)  { motorArm();    _switchDebounceMs = now; }
    if (ch5us < 1300 &&  _armed)  { motorDisarm(); _switchDebounceMs = now; }
}

// ----------------------------------------------------------------
// Mode switch (CH6): >1500 µs = ANGLE, ≤1500 µs = ACRO
void checkModeSwitch(uint16_t ch6us)
{
    uint32_t now = millis();
    if (now - _modeDebounceMs < DEBOUNCE_MS) return;
    _modeDebounceMs = now;

    if (ch6us > 1500 && _mode != MODE_ANGLE)
    {
        _mode = MODE_ANGLE;
        Serial.println("[FC] Mode → ANGLE");
    }
    else if (ch6us <= 1500 && _mode != MODE_ACRO)
    {
        _mode = MODE_ACRO;
        Serial.println("[FC] Mode → ACRO");
    }
}


// ----------------------------------------------------------------
// Stick-based arm / disarm (Betaflight convention)
void checkArmDisarm(float throttleNorm, float yawNorm)
{
    uint32_t now = millis();

    if (!_armed)
    {
        if (throttleNorm < ARM_THROTTLE_MAX && yawNorm > ARM_YAW_THRESHOLD)
        {
            if (now - _armDebounceMs > DEBOUNCE_MS)
            {
                motorArm();
                _armDebounceMs = now;
            }
        }
    }
    else
    {
        if (throttleNorm < ARM_THROTTLE_MAX && yawNorm < -ARM_YAW_THRESHOLD)
        {
            if (now - _armDebounceMs > DEBOUNCE_MS)
            {
                motorDisarm();
                _armDebounceMs = now;
            }
        }
    }
}

// ----------------------------------------------------------------
// Switch-based arm (CH5): >1700 µs = armed, <1300 µs = disarmed
void checkArmSwitch(uint16_t ch5us)
{
    uint32_t now = millis();
    if (now - _switchDebounceMs < DEBOUNCE_MS) return;

    if (ch5us > 1700 && !_armed)  { motorArm();    _switchDebounceMs = now; }
    if (ch5us < 1300 &&  _armed)  { motorDisarm(); _switchDebounceMs = now; }
}

// ----------------------------------------------------------------
// Mode switch (CH6): >1500 µs = ANGLE, ≤1500 µs = ACRO
void checkModeSwitch(uint16_t ch6us)
{
    uint32_t now = millis();
    if (now - _modeDebounceMs < DEBOUNCE_MS) return;
    _modeDebounceMs = now;

    if (ch6us > 1500 && _mode != MODE_ANGLE)
    {
        _mode = MODE_ANGLE;
        Serial.println("[FC] Mode → ANGLE");
    }
    else if (ch6us <= 1500 && _mode != MODE_ACRO)
    {
        _mode = MODE_ACRO;
        Serial.println("[FC] Mode → ACRO");
    }
}
