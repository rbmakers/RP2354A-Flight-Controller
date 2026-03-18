#include "battery.h"
#include "../config.h"

// ================================================================
// RP2354A ADC setup
// ================================================================
// The RP2350 ADC input range is 0–3.3 V (VREF = AVDD = 3.3 V).
// Arduino analogRead() on the arduino-pico core returns 0–4095
// for a 12-bit read when analogReadResolution(12) is set.

// Internal state
static float   _voltFiltered = (float)BAT_FULL_MV;
static float   _currFiltered = 0.0f;
static bool    _initialized  = false;

// Auto-detected cell count (set once at startup from first reading)
static uint8_t _cellCount = 4;

// ================================================================
// Cell count auto-detection
// ================================================================
// Reads the raw voltage once at startup and guesses cell count.
// Works reliably only when pack is reasonably charged (>3.5 V/cell).

static uint8_t detectCellCount(uint16_t voltMV)
{
    if      (voltMV > 17500) return 5;   // 5S: 17.5–21.0 V
    else if (voltMV > 13500) return 4;   // 4S: 13.5–16.8 V
    else if (voltMV > 10000) return 3;   // 3S: 10.0–12.6 V
    else if (voltMV >  6500) return 2;   // 2S:  6.5– 8.4 V
    else                     return 1;   // 1S: assume single cell
}

// ================================================================
// Raw ADC read — averaged over 8 samples for noise reduction
// ================================================================

static uint16_t readADC_mV(uint8_t pin)
{
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) sum += analogRead(pin);
    float adc_lsb = sum / 8.0f;
    return (uint16_t)(adc_lsb * BAT_ADC_MV_PER_LSB);
}

// ================================================================
// Public API
// ================================================================

void batInit()
{
    analogReadResolution(12);   // 12-bit: 0–4095

    pinMode(BAT_VOLTAGE_PIN, INPUT);
#ifdef BAT_CURRENT_PIN
    pinMode(BAT_CURRENT_PIN, INPUT);
#endif

    // Take initial reading
    uint16_t adc_mV   = readADC_mV(BAT_VOLTAGE_PIN);
    uint16_t volt_mV  = (uint16_t)(adc_mV * BAT_DIVIDER_RATIO);
    _voltFiltered     = (float)volt_mV;
    _cellCount        = detectCellCount(volt_mV);

    Serial.print("[BAT] Init: ");
    Serial.print(volt_mV);
    Serial.print(" mV, ");
    Serial.print(_cellCount);
    Serial.println("S LiPo detected");

    _initialized = true;
}

void batUpdate()
{
    if (!_initialized) return;

    // ---- Voltage ----
    uint16_t adc_mV  = readADC_mV(BAT_VOLTAGE_PIN);
    float    volt_mV = adc_mV * BAT_DIVIDER_RATIO;
    _voltFiltered = _voltFiltered * (1.0f - BAT_LPF_ALPHA)
                  + volt_mV      *  BAT_LPF_ALPHA;

    // ---- Current (optional) ----
#ifdef BAT_CURRENT_PIN
    uint16_t curr_adc_mV = readADC_mV(BAT_CURRENT_PIN);
    float curr_A = ((float)curr_adc_mV - CURRENT_ZERO_MV)
                   / CURRENT_SENSOR_MV_PER_A;
    // Convert to centi-Amps; clamp to 0 (sensor may go slightly negative at 0 A)
    float curr_cA = curr_A * 100.0f;
    if (curr_cA < 0.0f) curr_cA = 0.0f;
    _currFiltered = _currFiltered * (1.0f - BAT_LPF_ALPHA)
                  + curr_cA       *  BAT_LPF_ALPHA;
#endif

    // Low-voltage console warning
    if (batIsCritical())
        Serial.println("[BAT] CRITICAL voltage — initiating forced landing");
    else if (batIsLow())
        Serial.println("[BAT] LOW voltage — return to home");
}

uint16_t batGetVoltage_mV()
{
    return (uint16_t)_voltFiltered;
}

int16_t batGetCurrent_cA()
{
    return (int16_t)_currFiltered;
}

uint8_t batGetPercent()
{
    // Linear mapping from BAT_CRITICAL_MV (0 %) to BAT_FULL_MV (100 %)
    float v = _voltFiltered;
    float pct = (v - BAT_CRITICAL_MV) /
                (float)(BAT_FULL_MV - BAT_CRITICAL_MV) * 100.0f;
    return (uint8_t)constrain((int)pct, 0, 100);
}

bool batIsLow()      { return _voltFiltered <= (float)BAT_WARN_MV;     }
bool batIsCritical() { return _voltFiltered <= (float)BAT_CRITICAL_MV; }
