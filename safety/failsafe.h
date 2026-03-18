#pragma once
#include <Arduino.h>

// ================================================================
// Failsafe Handler
// ================================================================
// Monitors RC link quality, battery voltage, and other conditions
// that require emergency motor shutdown or controlled descent.
//
// Failsafe levels (escalating):
//   FS_NONE     — all systems nominal
//   FS_WARN     — link flicker or low battery (flash warning LED)
//   FS_LAND     — sustained link loss or critical battery
//                 → gradually reduce throttle to land in place
//   FS_KILL     — hard kill (immediate motor stop)
//                 → triggered by disarm switch or crash detection
//
// RC link-loss timing (matches ELRS fail-safe defaults):
//   > 100 ms no frame → FS_WARN
//   > 500 ms no frame → FS_LAND
//   > 2000 ms no frame → FS_KILL
//
// Integration: call failsafeUpdate() once per control loop.
// failsafeGetThrottle() returns a throttle override (0.0–1.0)
// that replaces the RC throttle when FS_LAND is active.
// ================================================================

typedef enum {
    FS_NONE  = 0,
    FS_WARN  = 1,
    FS_LAND  = 2,
    FS_KILL  = 3
} FailsafeLevel;

void          failsafeInit();
void          failsafeUpdate(bool linkUp, bool batCritical);
FailsafeLevel failsafeGet();
bool          failsafeActive();         // true if FS_LAND or FS_KILL
float         failsafeGetThrottle(float rcThrottle);
const char*   failsafeName(FailsafeLevel lvl);

// ================================================================
// Motor Test Mode
// ================================================================
// Safely spins individual motors at a low fixed throttle for bench
// testing (confirms motor direction and ESC response).
//
// Safety interlocks:
//   - Armed flag must be CLEARED (disarmed)
//   - All motor outputs set to zero before entering test mode
//   - Maximum test throttle capped at MOTOR_TEST_MAX_THROTTLE
//   - Automatic timeout: motor test stops after MOTOR_TEST_TIMEOUT_MS
//
// Usage:
//   motorTestStart(motorIndex, throttle);   // 0–3, 0.0–1.0
//   motorTestStop();
//   motorTestUpdate(m1, m2, m3, m4);        // call every loop;
//                                           // overrides outputs if active
// ================================================================

#define MOTOR_TEST_MAX_THROTTLE   0.15f      // 15 % max for bench safety
#define MOTOR_TEST_TIMEOUT_MS     5000       // auto-stop after 5 s

bool  motorTestStart(uint8_t motor, float throttle);  // returns false if armed
void  motorTestStop();
bool  motorTestActive();
// Fills m1..m4 with test values if test is active; returns true if overriding.
bool  motorTestOverride(float &m1, float &m2, float &m3, float &m4);
