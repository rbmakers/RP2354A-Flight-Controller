#include "esc_calibration.h"
#include "../control/flight_modes.h"

// ================================================================
// Timing (milliseconds)
// ================================================================
#define CAL_MAX_HOLD_MS   3000   // hold max throttle 3 s (ESC beeps twice)
#define CAL_MIN_HOLD_MS   2000   // hold min throttle 2 s (ESC stores range)
#define CAL_WAIT_PWR_MS   5000   // user has 5 s to power ESC after FC ready

static ESCCalState _state    = ESC_CAL_IDLE;
static uint32_t    _stateMs  = 0;

// ================================================================
// Public API
// ================================================================

void escCalStart()
{
    if (isArmed())
    {
        Serial.println("[ESC_CAL] REFUSED — disarm first!");
        _state = ESC_CAL_ABORTED;
        return;
    }

    Serial.println("[ESC_CAL] *** REMOVE PROPELLERS NOW ***");
    Serial.println("[ESC_CAL] Power your ESCs within 5 seconds...");
    Serial.println("[ESC_CAL] FC is outputting MAXIMUM throttle.");

    _state   = ESC_CAL_WAIT_PWR;
    _stateMs = millis();
}

void escCalAbort()
{
    _state = ESC_CAL_ABORTED;
    Serial.println("[ESC_CAL] Aborted.");
}

ESCCalState escCalUpdate(float &m1, float &m2, float &m3, float &m4)
{
    uint32_t elapsed = millis() - _stateMs;

    switch (_state)
    {
    case ESC_CAL_WAIT_PWR:
        // Output maximum throttle while waiting for user to power ESC
        m1 = m2 = m3 = m4 = 1.0f;

        if (elapsed >= CAL_WAIT_PWR_MS)
        {
            // Transition: still at max, now in timed phase
            _state   = ESC_CAL_SEND_MAX;
            _stateMs = millis();
            Serial.println("[ESC_CAL] Sending MAX for 3 s — listen for 2 beeps...");
        }
        break;

    case ESC_CAL_SEND_MAX:
        m1 = m2 = m3 = m4 = 1.0f;

        if (elapsed >= CAL_MAX_HOLD_MS)
        {
            _state   = ESC_CAL_SEND_MIN;
            _stateMs = millis();
            Serial.println("[ESC_CAL] Sending MIN for 2 s — listen for confirmation beep...");
        }
        break;

    case ESC_CAL_SEND_MIN:
        m1 = m2 = m3 = m4 = 0.0f;

        if (elapsed >= CAL_MIN_HOLD_MS)
        {
            _state = ESC_CAL_COMPLETE;
            Serial.println("[ESC_CAL] Complete — ESCs calibrated.");
            Serial.println("[ESC_CAL] You may now arm normally.");
        }
        break;

    case ESC_CAL_COMPLETE:
    case ESC_CAL_ABORTED:
        m1 = m2 = m3 = m4 = 0.0f;
        break;

    case ESC_CAL_IDLE:
    default:
        break;
    }

    return _state;
}

bool escCalDone()   { return _state == ESC_CAL_COMPLETE; }
bool escCalActive() { return _state == ESC_CAL_WAIT_PWR ||
                             _state == ESC_CAL_SEND_MAX  ||
                             _state == ESC_CAL_SEND_MIN; }

const char* escCalStateName(ESCCalState s)
{
    switch (s)
    {
    case ESC_CAL_IDLE:     return "IDLE";
    case ESC_CAL_WAIT_PWR: return "WAIT_PWR";
    case ESC_CAL_SEND_MAX: return "SEND_MAX";
    case ESC_CAL_SEND_MIN: return "SEND_MIN";
    case ESC_CAL_COMPLETE: return "COMPLETE";
    case ESC_CAL_ABORTED:  return "ABORTED";
    default:               return "?";
    }
}
