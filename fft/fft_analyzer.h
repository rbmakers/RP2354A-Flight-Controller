#pragma once
#include <Arduino.h>

// ================================================================
// Optional FFT Vibration Analyzer
// ================================================================
// Enable with  #define USE_FFT_ANALYSIS  in config.h
//
// Runs a 256-point DFT on the gyro data accumulated on Core 1.
// Identifies the dominant frequency peak and feeds it to the
// dynamic notch filter.
//
// CPU cost at 2 kHz input, 256-pt DFT, computed ~8× per second:
//   ≈ 8 % of one core.
//
// In production, replace the DFT with an ARM CMSIS-DSP FFT
// (available for Cortex-M33 on RP2354A) for ~10× lower CPU cost.
// ================================================================

#define FFT_SIZE  256

void  fftInit();
void  fftAddSample(float gyroDps);          // called from Core0 ISR path
float fftGetPeakFrequencyHz();              // peak vibration frequency
bool  fftResultReady();                     // true after first full window
void  fftPrintSpectrum();                   // debug: print to Serial
