#pragma once
#include <Arduino.h>

// ================================================================
// Biquad Digital Filter — second-order IIR
// Supports Butterworth low-pass and notch designs.
// ================================================================

typedef struct
{
    // Coefficients
    float a0, a1, a2;    // feedforward (numerator)
    float b1, b2;        // feedback    (denominator, note b0=1 normalised)
    // State (Direct Form II transposed)
    float w1, w2;
} BiquadFilter;

// Initialise as 2nd-order Butterworth low-pass
// cutoffHz / sampleRateHz should be < 0.45
void   biquadInitLPF(BiquadFilter *f, float cutoffHz, float sampleRateHz);

// Initialise as notch filter
// q: quality factor — higher = narrower notch (typical 1.0–5.0)
void   biquadInitNotch(BiquadFilter *f, float centerHz,
                       float sampleRateHz, float q);

// Apply filter to one sample; returns filtered output
float  biquadApply(BiquadFilter *f, float x);

// Reset state (zero initial conditions)
void   biquadReset(BiquadFilter *f);
