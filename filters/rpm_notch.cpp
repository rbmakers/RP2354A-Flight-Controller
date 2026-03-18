#include "rpm_notch.h"
#include "../config.h"

static MotorRPMFilter _mf[RPM_FILTER_MOTORS];
static float          _fs = 2000.0f;
static uint16_t       _rpm[RPM_FILTER_MOTORS] = {0};

void rpmFilterInit(float sampleRateHz)
{
    _fs = sampleRateHz;
    for (int m = 0; m < RPM_FILTER_MOTORS; m++)
        for (int h = 0; h < RPM_FILTER_HARMONICS; h++)
            biquadReset(&_mf[m].notch[h]);
}

// electricalRPM = mechanical_RPM * pole_pairs
void rpmFilterUpdateRPM(uint8_t motor, uint16_t electricalRPM)
{
    if (motor >= RPM_FILTER_MOTORS) return;
    if (_rpm[motor] == electricalRPM)  return;   // no change — skip
    _rpm[motor] = electricalRPM;

    for (int h = 0; h < RPM_FILTER_HARMONICS; h++)
    {
        float freq = electricalRPM * (h + 1) / 60.0f;

        if (freq > 5.0f && freq < _fs * 0.48f)
        {
            // Q=2.5: moderately narrow notch — tune to motor noise width
            biquadInitNotch(&_mf[motor].notch[h], freq, _fs, 2.5f);
        }
        else
        {
            // Frequency out of range: reset to pass-through
            biquadReset(&_mf[motor].notch[h]);
        }
    }
}

float rpmFilterApply(float sample)
{
    for (int m = 0; m < RPM_FILTER_MOTORS; m++)
        for (int h = 0; h < RPM_FILTER_HARMONICS; h++)
            sample = biquadApply(&_mf[m].notch[h], sample);
    return sample;
}
