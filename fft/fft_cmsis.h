#pragma once
#include <Arduino.h>

// ================================================================
// FFT Vibration Analyzer — Production Version
// ================================================================
// This module replaces the naive O(N²) DFT in fft_analyzer.cpp
// with the ARM CMSIS-DSP real FFT (arm_rfft_fast_f32), which uses
// a radix-2 Cooley-Tukey algorithm: O(N log₂N) instead of O(N²).
//
// Performance comparison at N=256, 150 MHz Cortex-M33:
//   Naive DFT:       ≈ 34 000 µs  (~34 ms, clearly too slow)
//   CMSIS-DSP FFT:   ≈    240 µs  (0.24 ms, CPU load < 0.2 %)
//
// CMSIS-DSP is part of the ARM CMSIS library and is included in
// the earlephilhower arduino-pico core as:
//   #include <CMSIS_DSP.h>   (header wrapper provided by the core)
//
// If CMSIS_DSP is not available (older core versions), the module
// automatically falls back to the Goertzel algorithm, which is
// O(N×K) where K is the number of bins we actually care about
// (typically 10–30 bins), making it much faster than full DFT
// while not requiring any external library.
//
// Enable this module with:
//   #define USE_FFT_ANALYSIS
//   #define USE_CMSIS_FFT       // optional — uses CMSIS if available
// in config.h.
// ================================================================

#define FFT_SIZE_PRO  256    // Must be power of 2; 256 = 128 frequency bins

void  fftProInit();
void  fftProAddSample(float gyroDps);   // call at IMU_RATE_HZ from Core0
float fftProGetPeakHz();                // dominant vibration frequency
bool  fftProResultReady();
void  fftProGetSpectrum(float *magnitudes, uint16_t *count); // count=FFT_SIZE_PRO/2
