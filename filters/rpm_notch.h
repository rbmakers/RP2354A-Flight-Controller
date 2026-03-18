#pragma once
#include <Arduino.h>
#include "biquad_filter.h"

// ================================================================
// RPM-based Notch Filter
// ================================================================
// Places narrow notch filters at the fundamental and 2nd harmonic
// of each motor's electrical frequency.
// Requires ESC telemetry RPM data (DShot bidirectional or BLHeli
// telemetry).  Without RPM feedback the filters are disabled.
//
// Enable with USE_RPM_FILTER in config.h.
// ================================================================

#define RPM_FILTER_MOTORS      4
#define RPM_FILTER_HARMONICS   2    // fundamental + 2nd harmonic

typedef struct
{
    BiquadFilter notch[RPM_FILTER_HARMONICS];
} MotorRPMFilter;

void  rpmFilterInit(float sampleRateHz);
void  rpmFilterUpdateRPM(uint8_t motor, uint16_t electricalRPM);
float rpmFilterApply(float gyroSample);   // apply all motor notches
