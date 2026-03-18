#include "failsafe.h"
#include "../control/flight_modes.h"   // motorDisarm()

// ================================================================
// Failsafe
// ================================================================

// Link-loss timing thresholds (milliseconds)
#define FS_WARN_MS  100
#define FS_LAND_MS  500
#define FS_KILL_MS 2000

static FailsafeLevel _fsLevel = FS_NONE;
static uint32_t      _linkLostMs = 0;       // millis() when link was last seen
static bool          _everLinked = false;   // don't trigger FS before first link

// Controlled-landing throttle descends over FS_LAND_DURATION_MS
#define FS_LAND_DURATION_MS  8000
static uint32_t _landStartMs = 0;
static float    _landStartThrottle = 0.5f;

void failsafeInit()
{
    _fsLevel    = FS_NONE;
    _linkLostMs = millis();
    _everLinked = false;
}

void failsafeUpdate(bool linkUp, bool batCritical)
{
    uint32_t now = millis();

    // ---- Battery critical → immediate FS_LAND ----
    if (batCritical && isArmed())
    {
        if (_fsLevel < FS_LAND)
        {
            _fsLevel       = FS_LAND;
            _landStartMs   = now;
            Serial.println("[FS] Battery CRITICAL → FS_LAND");
        }
    }

    // ---- RC link watchdog ----
    if (linkUp)
    {
        _everLinked = true;
        _linkLostMs = now;

        // Clear failsafe only if we're in WARN level and link returns
        if (_fsLevel == FS_WARN) _fsLevel = FS_NONE;
    }
    else if (_everLinked)
    {
        uint32_t lostMs = now - _linkLostMs;

        if      (lostMs >= FS_KILL_MS && _fsLevel < FS_KILL)
        {
            _fsLevel = FS_KILL;
            motorDisarm();
            Serial.println("[FS] Link lost >2 s → FS_KILL + DISARM");
        }
        else if (lostMs >= FS_LAND_MS && _fsLevel < FS_LAND)
        {
            _fsLevel       = FS_LAND;
            _landStartMs   = now;
            Serial.println("[FS] Link lost >500 ms → FS_LAND");
        }
        else if (lostMs >= FS_WARN_MS && _fsLevel < FS_WARN)
        {
            _fsLevel = FS_WARN;
            Serial.println("[FS] Link lost >100 ms → FS_WARN");
        }
    }
}

FailsafeLevel failsafeGet()     { return _fsLevel; }
bool          failsafeActive()  { return _fsLevel >= FS_LAND; }

float failsafeGetThrottle(float rcThrottle)
{
    if (_fsLevel == FS_KILL)  return 0.0f;
    if (_fsLevel != FS_LAND)  return rcThrottle;

    // FS_LAND: linearly ramp throttle from landing_start value to 0
    // over FS_LAND_DURATION_MS, then kill.
    uint32_t elapsed = millis() - _landStartMs;

    if (elapsed >= (uint32_t)FS_LAND_DURATION_MS)
    {
        _fsLevel = FS_KILL;
        motorDisarm();
        Serial.println("[FS] Landing complete → FS_KILL");
        return 0.0f;
    }

    float ratio = 1.0f - (float)elapsed / (float)FS_LAND_DURATION_MS;
    return _landStartThrottle * ratio;
}

const char* failsafeName(FailsafeLevel lvl)
{
    switch (lvl) {
        case FS_NONE: return "NONE";
        case FS_WARN: return "WARN";
        case FS_LAND: return "LAND";
        case FS_KILL: return "KILL";
        default:      return "?";
    }
}

// ================================================================
// Motor Test
// ================================================================

static bool    _testActive   = false;
static uint8_t _testMotor    = 0;
static float   _testThrottle = 0.0f;
static uint32_t _testStartMs = 0;

bool motorTestStart(uint8_t motor, float throttle)
{
    if (isArmed())
    {
        Serial.println("[MotorTest] REFUSED — disarm first");
        return false;
    }
    if (motor >= 4)
    {
        Serial.println("[MotorTest] Invalid motor index");
        return false;
    }

    _testMotor    = motor;
    _testThrottle = constrain(throttle, 0.0f, MOTOR_TEST_MAX_THROTTLE);
    _testStartMs  = millis();
    _testActive   = true;

    Serial.print("[MotorTest] Motor ");
    Serial.print(motor + 1);
    Serial.print(" at ");
    Serial.print((int)(_testThrottle * 100));
    Serial.println(" %");

    return true;
}

void motorTestStop()
{
    _testActive = false;
    Serial.println("[MotorTest] Stopped");
}

bool motorTestActive() { return _testActive; }

bool motorTestOverride(float &m1, float &m2, float &m3, float &m4)
{
    if (!_testActive) return false;

    // Auto-timeout
    if (millis() - _testStartMs > (uint32_t)MOTOR_TEST_TIMEOUT_MS)
    {
        motorTestStop();
        m1 = m2 = m3 = m4 = 0.0f;
        return true;
    }

    m1 = m2 = m3 = m4 = 0.0f;

    switch (_testMotor)
    {
        case 0: m1 = _testThrottle; break;
        case 1: m2 = _testThrottle; break;
        case 2: m3 = _testThrottle; break;
        case 3: m4 = _testThrottle; break;
    }

    return true;
}
