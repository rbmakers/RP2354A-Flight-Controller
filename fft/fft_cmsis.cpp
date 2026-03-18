#include "fft_cmsis.h"
#include "../config.h"
#include <math.h>
#include <string.h>

// ================================================================
// Attempt to include CMSIS-DSP
// ================================================================
// arduino-pico ≥ 3.x ships CMSIS-DSP; older versions may not.
// We use a soft-detect: if the header exists, use the fast FFT,
// otherwise fall through to the Goertzel fallback.

#if __has_include(<CMSIS_DSP.h>)
  #include <CMSIS_DSP.h>
  #define HAVE_CMSIS_DSP 1
#else
  #define HAVE_CMSIS_DSP 0
#endif

// ================================================================
// Shared state
// ================================================================

static float   _inputBuf[FFT_SIZE_PRO];   // circular sample buffer
static int     _bufIdx   = 0;
static bool    _ready    = false;
static float   _peakHz   = 0.0f;

// Output spectrum (magnitude, one-sided: FFT_SIZE_PRO/2 bins)
static float   _spectrum[FFT_SIZE_PRO / 2];
static uint16_t _specLen = FFT_SIZE_PRO / 2;

// Frequency resolution = IMU_RATE_HZ / FFT_SIZE_PRO
static const float BIN_HZ = (float)IMU_RATE_HZ / (float)FFT_SIZE_PRO;

// ================================================================
// Hann window (pre-computed at init)
// ================================================================
static float _hannWin[FFT_SIZE_PRO];

static void buildHannWindow()
{
    for (int n = 0; n < FFT_SIZE_PRO; n++)
        _hannWin[n] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * n
                                           / (FFT_SIZE_PRO - 1)));
}

// ================================================================
// PATH A: CMSIS-DSP arm_rfft_fast_f32
// ================================================================
#if HAVE_CMSIS_DSP

static arm_rfft_fast_instance_f32 _fftInst;

static void computeFFT_CMSIS(const float *windowedData)
{
    // arm_rfft_fast_f32 operates in-place.
    // Output is in CCS (complex conjugate symmetric) format:
    //   buf[0]         = DC bin (real only)
    //   buf[1]         = Nyquist bin (real only)
    //   buf[2k],buf[2k+1] = real, imag of bin k  (k=1..N/2-1)
    static float fftBuf[FFT_SIZE_PRO];
    memcpy(fftBuf, windowedData, sizeof(fftBuf));

    arm_rfft_fast_f32(&_fftInst, fftBuf, fftBuf, 0);

    // Compute magnitude for bins 1..N/2-1 (skip DC)
    _spectrum[0] = fabsf(fftBuf[0]);   // DC
    float maxMag = 0.0f;
    int   maxBin = 1;

    for (int k = 1; k < FFT_SIZE_PRO / 2; k++)
    {
        float re  = fftBuf[2 * k];
        float im  = fftBuf[2 * k + 1];
        float mag = sqrtf(re * re + im * im);

        _spectrum[k] = mag;

        // Ignore bins below 20 Hz (likely vibration-unrelated)
        if (mag > maxMag && (k * BIN_HZ) >= 20.0f)
        {
            maxMag = mag;
            maxBin = k;
        }
    }

    _peakHz = maxBin * BIN_HZ;
    _ready  = true;
}

void fftProInit()
{
    buildHannWindow();
    arm_rfft_fast_init_f32(&_fftInst, FFT_SIZE_PRO);
    memset(_spectrum, 0, sizeof(_spectrum));
    Serial.println("[FFT] CMSIS-DSP arm_rfft_fast_f32 initialised");
}

// ================================================================
// PATH B: Goertzel algorithm fallback
// ================================================================
// Evaluates only a selected set of K frequency bins instead of all
// N/2 bins, giving O(N*K) cost.  For motor vibration monitoring we
// only care about ~30 bins covering 20 Hz – 500 Hz.
#else   // !HAVE_CMSIS_DSP

// Number of Goertzel bins to evaluate (covers 20 Hz–512 Hz at 8 Hz steps)
#define GOERTZEL_BINS  62

static int _goertzelBinIndices[GOERTZEL_BINS];   // bin numbers (k values)
static int _numGoertzelBins = 0;

static float goertzelMagnitude(const float *buf, int N, int k)
{
    float omega  = 2.0f * (float)M_PI * k / N;
    float coeff  = 2.0f * cosf(omega);
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;

    for (int n = 0; n < N; n++)
    {
        s0 = buf[n] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    // Power = s1² + s2² - s1*s2*coeff
    return sqrtf(s1 * s1 + s2 * s2 - s1 * s2 * coeff);
}

static void computeFFT_Goertzel(const float *windowedData)
{
    float maxMag = 0.0f;
    int   maxBin = _goertzelBinIndices[0];

    for (int i = 0; i < _numGoertzelBins; i++)
    {
        int   k   = _goertzelBinIndices[i];
        float mag = goertzelMagnitude(windowedData, FFT_SIZE_PRO, k);

        _spectrum[k] = mag;

        if (mag > maxMag)
        {
            maxMag = mag;
            maxBin = k;
        }
    }

    _peakHz = maxBin * BIN_HZ;
    _ready  = true;
}

void fftProInit()
{
    buildHannWindow();
    memset(_spectrum, 0, sizeof(_spectrum));

    // Build list of bin indices from 20 Hz to 550 Hz in steps of ~BIN_HZ
    // (one step per bin, so every integer bin in that range)
    _numGoertzelBins = 0;
    for (int k = 1; k < FFT_SIZE_PRO / 2 && _numGoertzelBins < GOERTZEL_BINS; k++)
    {
        float hz = k * BIN_HZ;
        if (hz >= 20.0f && hz <= 550.0f)
            _goertzelBinIndices[_numGoertzelBins++] = k;
    }

    Serial.print("[FFT] Goertzel fallback, ");
    Serial.print(_numGoertzelBins);
    Serial.println(" bins (20–550 Hz)");
}

#endif  // HAVE_CMSIS_DSP

// ================================================================
// Common: sample ingestion and window application
// ================================================================

void fftProAddSample(float gyroDps)
{
    _inputBuf[_bufIdx] = gyroDps;
    _bufIdx = (_bufIdx + 1) % FFT_SIZE_PRO;

    // When the circular buffer completes one full revolution, compute.
    if (_bufIdx == 0)
    {
        // Apply Hann window in a temporary buffer
        static float windowed[FFT_SIZE_PRO];
        for (int n = 0; n < FFT_SIZE_PRO; n++)
            windowed[n] = _inputBuf[n] * _hannWin[n];

#if HAVE_CMSIS_DSP
        computeFFT_CMSIS(windowed);
#else
        computeFFT_Goertzel(windowed);
#endif
    }
}

float fftProGetPeakHz()       { return _peakHz; }
bool  fftProResultReady()     { return _ready;  }

void fftProGetSpectrum(float *magnitudes, uint16_t *count)
{
    memcpy(magnitudes, _spectrum, sizeof(float) * _specLen);
    *count = _specLen;
}
