#include "biquad_filter.h"
#include <math.h>

void biquadReset(BiquadFilter *f)
{
    f->w1 = f->w2 = 0.0f;
}

void biquadInitLPF(BiquadFilter *f, float cutoffHz, float sampleRateHz)
{
    // Butterworth 2nd-order low-pass (bilinear transform)
    float omega  = 2.0f * (float)M_PI * cutoffHz / sampleRateHz;
    float sn     = sinf(omega);
    float cs     = cosf(omega);
    float alpha  = sn / sqrtf(2.0f);   // Q = 1/sqrt(2) for Butterworth

    float b0 = (1.0f - cs) * 0.5f;
    float b1 =  1.0f - cs;
    float b2 = (1.0f - cs) * 0.5f;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 =  1.0f - alpha;

    f->a0 = b0 / a0;
    f->a1 = b1 / a0;
    f->a2 = b2 / a0;
    f->b1 = a1 / a0;
    f->b2 = a2 / a0;
    biquadReset(f);
}

void biquadInitNotch(BiquadFilter *f, float centerHz,
                     float sampleRateHz, float q)
{
    if (centerHz <= 0.0f || centerHz >= sampleRateHz * 0.5f) return;
    if (q < 0.1f) q = 0.1f;

    float omega = 2.0f * (float)M_PI * centerHz / sampleRateHz;
    float cs    = cosf(omega);
    float alpha = sinf(omega) / (2.0f * q);

    float b0 =  1.0f;
    float b1 = -2.0f * cs;
    float b2 =  1.0f;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 =  1.0f - alpha;

    f->a0 = b0 / a0;
    f->a1 = b1 / a0;
    f->a2 = b2 / a0;
    f->b1 = a1 / a0;
    f->b2 = a2 / a0;
    biquadReset(f);
}

float biquadApply(BiquadFilter *f, float x)
{
    // Direct Form II transposed (numerically stable)
    float y  = f->a0 * x + f->w1;
    f->w1    = f->a1 * x - f->b1 * y + f->w2;
    f->w2    = f->a2 * x - f->b2 * y;
    return y;
}
