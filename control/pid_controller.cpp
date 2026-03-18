#include "pid_controller.h"
#include "../config.h"
#include <math.h>

// ================================================================
// PID Axis
// ================================================================

void pidAxisInit(PIDAxis *pid,
                 float kp, float ki, float kd, float kff,
                 float integralLimit, float outputLimit)
{
    pid->kp            = kp;
    pid->ki            = ki;
    pid->kd            = kd;
    pid->kff           = kff;
    pid->integralLimit = integralLimit;
    pid->outputLimit   = outputLimit;
    pidAxisReset(pid);
}

void pidAxisReset(PIDAxis *pid)
{
    pid->integral    = 0.0f;
    pid->lastError   = 0.0f;
    pid->lastSetpoint = 0.0f;
    pid->lastOutput  = 0.0f;
}

float pidAxisUpdate(PIDAxis *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    // ----------------------------------------------------------------
    // I-term Relax
    // ----------------------------------------------------------------
    // When the setpoint is changing rapidly (fast stick movement),
    // the integrator is suppressed proportionally.  This prevents the
    // integrator from winding up during hard flips and then causing a
    // bounce-back when the stick returns to centre.
    //
    // setpoint_rate is in the same units as setpoint (rad/s for rate loop).
    // ITERM_RELAX_CUTOFF is the rate threshold above which relaxation
    // starts; at ITERM_RELAX_CUTOFF × 1.0 the integrator is fully frozen.
    // ----------------------------------------------------------------
    float setpointRate = fabsf((setpoint - pid->lastSetpoint) / dt);

    float relaxFactor = 1.0f - constrain(setpointRate / ITERM_RELAX_CUTOFF,
                                          0.0f, 1.0f);

    pid->integral += error * dt * relaxFactor;
    pid->integral  = constrain(pid->integral,
                                -pid->integralLimit, pid->integralLimit);

    // ----------------------------------------------------------------
    // Derivative on measurement (avoids "derivative kick" on setpoint step)
    // ----------------------------------------------------------------
    float derivative = -(measurement - (measurement - error) -
                         (measurement - error - pid->lastError)) / dt;
    // Simplified: derivative of error where lastError was based on measurement
    derivative = (error - pid->lastError) / dt;
    pid->lastError = error;

    // ----------------------------------------------------------------
    // Feedforward  (fires immediately on setpoint change)
    // ----------------------------------------------------------------
    // Derivative of the setpoint — non-zero only when the stick is moving.
    // This is the term that eliminates tracking lag during fast manoeuvres.
    // Kff is zero by default; set PID_ROLL_KFF etc. in config.h to enable.
    // ----------------------------------------------------------------
    float ffTerm = pid->kff * (setpoint - pid->lastSetpoint) / dt;
    pid->lastSetpoint = setpoint;

    // ----------------------------------------------------------------
    // Sum all terms
    // ----------------------------------------------------------------
    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative
                 + ffTerm;

    output = constrain(output, -pid->outputLimit, pid->outputLimit);
    pid->lastOutput = output;
    return output;
}

// ================================================================
// Auto-Tune — relay-feedback (Ziegler-Nichols ultimate gain method)
// ================================================================
#define AUTOTUNE_MIN_CYCLES  8

void autoTuneStart(PIDAutoTune *at, float relayAmp)
{
    at->active         = true;
    at->done           = false;
    at->relayAmplitude = relayAmp;
    at->peakPos        = 0.0f;
    at->peakNeg        = 0.0f;
    at->lastCrossing   = 0.0f;
    at->period         = 0.0f;
    at->cycleCount     = 0;
    at->resultKp = at->resultKi = at->resultKd = 0.0f;
}

float autoTuneUpdate(PIDAutoTune *at, float error, float dt)
{
    if (!at->active || at->done) return 0.0f;

    static float timeAcc  = 0.0f;
    static float lastErr  = 0.0f;
    timeAcc += dt;

    if (error > at->peakPos) at->peakPos = error;
    if (error < at->peakNeg) at->peakNeg = error;

    float output = (error >= 0.0f) ? at->relayAmplitude : -at->relayAmplitude;

    if (lastErr < 0.0f && error >= 0.0f)
    {
        if (at->lastCrossing > 0.0f)
        {
            at->period = 2.0f * (timeAcc - at->lastCrossing);
            at->cycleCount++;

            if (at->cycleCount >= AUTOTUNE_MIN_CYCLES)
            {
                float amplitude = (at->peakPos - at->peakNeg) * 0.5f;
                if (amplitude > 1e-4f)
                {
                    float ku = (4.0f * at->relayAmplitude) / ((float)M_PI * amplitude);
                    float pu = at->period;
                    at->resultKp = 0.60f * ku;
                    at->resultKi = 2.0f  * at->resultKp / pu;
                    at->resultKd = at->resultKp * pu / 8.0f;
                }
                at->active = false;
                at->done   = true;
            }
        }
        at->lastCrossing = timeAcc;
        at->peakPos = 0.0f;
        at->peakNeg = 0.0f;
    }
    lastErr = error;
    return output;
}

bool autoTuneDone(PIDAutoTune *at) { return at->done; }

void autoTuneApply(PIDAutoTune *at, PIDAxis *pid)
{
    if (!at->done) return;
    pid->kp = at->resultKp;
    pid->ki = at->resultKi;
    pid->kd = at->resultKd;
    // kff is not changed by auto-tune — set manually in config.h
    pidAxisReset(pid);
}
