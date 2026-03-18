#include "fft_analyzer.h"
#include "../config.h"
#include <math.h>

// ================================================================
// DFT with Hann window
// Computed only when the circular buffer is full (every FFT_SIZE
// samples at 2 kHz = every 128 ms ≈ 7.8 Hz update rate).
// ================================================================

static float  _inputBuf[FFT_SIZE];   // circular sample buffer
static int    _bufIdx    = 0;
static bool   _ready     = false;
static float  _peakHz    = 0.0f;
static bool   _computing = false;    // guard against re-entrancy

// Hann window coefficient
static inline float hann(int n, int N)
{
    return 0.5f * (1.0f - cosf(2.0f * (float)M_PI * n / (N - 1)));
}

// Single-bin DFT magnitude (Goertzel-style loop is more efficient;
// this DFT loop is simpler and adequate at 256 points on Cortex-M33).
static float dftMagnitude(int k)
{
    float re = 0.0f, im = 0.0f;
    float w  = 2.0f * (float)M_PI * k / FFT_SIZE;
    for (int n = 0; n < FFT_SIZE; n++)
    {
        float h = hann(n, FFT_SIZE);
        float s = _inputBuf[(n + _bufIdx) % FFT_SIZE] * h;
        re += s * cosf(w * n);
        im -= s * sinf(w * n);
    }
    return sqrtf(re * re + im * im);
}

static void computeFFT()
{
    if (_computing) return;
    _computing = true;

    float maxMag = 0.0f;
    int   maxBin = 1;

    // Only check bins 1 to N/2−1 (DC excluded, Nyquist excluded).
    // Frequency resolution: IMU_RATE_HZ / FFT_SIZE Hz per bin.
    for (int k = 1; k < FFT_SIZE / 2; k++)
    {
        float mag = dftMagnitude(k);
        if (mag > maxMag) { maxMag = mag; maxBin = k; }
    }

    _peakHz = (float)maxBin * (float)IMU_RATE_HZ / (float)FFT_SIZE;
    _ready  = true;
    _computing = false;
}

// ================================================================
// Public API
// ================================================================

void fftInit()
{
    for (int i = 0; i < FFT_SIZE; i++) _inputBuf[i] = 0.0f;
    _bufIdx = 0;
    _ready  = false;
}

void fftAddSample(float gyroDps)
{
    _inputBuf[_bufIdx] = gyroDps;
    _bufIdx = (_bufIdx + 1) % FFT_SIZE;

    // Compute a new spectrum when we have filled one complete window
    if (_bufIdx == 0) computeFFT();
}

float fftGetPeakFrequencyHz() { return _peakHz; }
bool  fftResultReady()        { return _ready; }

void fftPrintSpectrum()
{
    if (!_ready) return;
    Serial.println("[FFT] Spectrum (bin Hz : magnitude)");
    for (int k = 1; k < FFT_SIZE / 2; k++)
    {
        float hz  = (float)k * (float)IMU_RATE_HZ / FFT_SIZE;
        float mag = dftMagnitude(k);
        Serial.print(hz, 1);
        Serial.print(" : ");
        Serial.println(mag, 2);
    }
}
