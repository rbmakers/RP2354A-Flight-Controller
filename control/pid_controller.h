#pragma once
#include <Arduino.h>

// ================================================================
// PID Controller  — with Feedforward, I-term Relax, and Auto-Tune
// ================================================================
// Three independent axis instances: roll, pitch, yaw.
//
// Full output equation:
//   output = Kp·error
//          + Ki·integral          (with I-term relax and anti-windup)
//          + Kd·d(measurement)/dt (derivative on measurement, no kick)
//          + Kff·d(setpoint)/dt   (feedforward — fires on stick movement)
//
// I-term Relax:
//   Suppresses the integrator when the setpoint is changing rapidly.
//   This prevents bounce-back after hard flips or fast stick inputs.
//   relax_factor = 1 − clamp(|setpoint_rate| / ITERM_RELAX_CUTOFF, 0, 1)
//   integral += error × dt × relax_factor
//
// All gain constants (KFF, ITERM_RELAX_CUTOFF) live in config.h
// alongside Kp/Ki/Kd so all PID parameters are in one place.
// ================================================================

typedef struct
{
    // Gains
    float kp, ki, kd, kff;
    // Limits
    float integralLimit;
    float outputLimit;
    // Internal state
    float integral;
    float lastError;
    float lastSetpoint;   // for feedforward derivative and I-term relax
    float lastOutput;
} PIDAxis;

// Initialise one axis.  kff = 0 disables feedforward.
void  pidAxisInit(PIDAxis *pid,
                  float kp, float ki, float kd, float kff,
                  float integralLimit, float outputLimit);

// Compute PID + FF output.  dt in seconds.
// setpoint and measurement must be in the same units (rad/s for rate loop).
float pidAxisUpdate(PIDAxis *pid, float setpoint, float measurement, float dt);

// Reset integrator, derivative state, and setpoint history.
// Call on mode-switch, disarm, or after auto-tune completes.
void  pidAxisReset(PIDAxis *pid);

// ---- Auto-tune (relay-feedback / Ziegler-Nichols) ---------------

typedef struct
{
    bool   active;
    bool   done;
    float  relayAmplitude;
    float  peakPos;
    float  peakNeg;
    float  lastCrossing;
    float  period;
    int    cycleCount;
    float  resultKp, resultKi, resultKd;
} PIDAutoTune;

void  autoTuneStart(PIDAutoTune *at, float relayAmp);
float autoTuneUpdate(PIDAutoTune *at, float error, float dt);
bool  autoTuneDone(PIDAutoTune *at);
void  autoTuneApply(PIDAutoTune *at, PIDAxis *pid);
