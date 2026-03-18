// ================================================================
// RP2354A Flight Controller  — Complete Flight Stack v5
// ================================================================
// Board    : RB-RP2354A  (RP2354A Cortex-M33, 150 MHz)
// IMU      : BMI088      (SPI,  DRDY interrupt @ 2 kHz)
// Baro     : BMP580      (I2C0, 50 Hz)
// Receiver : ExpressLRS  (UART0 CRSF, 420 kBaud)
// Motors   : Brushed PWM (GPIO29/11/18/25)  or
//            BLDC DShot600 PIO (GPIO1/4/10/14)
// Flash    : W25Q128 16 MiB SPI blackbox
// Battery  : ADC1 voltage
//
// ---- Core 0 -----------------------------------------------
//   Boot: self-test → IMU calibration → flight loop
//   1 kHz flight loop:
//     ISR: BMI088 DRDY → RPM notch → biquad LPF → Madgwick AHRS
//     Failsafe + battery monitor
//     Alt estimation + hold
//     Flight mode (Acro / Angle) + ARM/DISARM
//     TPA: scale Kp/Kd by throttle (Betaflight-style)
//     3-axis PID with Feedforward + I-term Relax
//     Quad-X mixer + Airmode desaturation
//     ESC calibration / motor test override
//     Motor output (Brushed PWM or PIO DShot600)
//     Blackbox write
//     MSP + CSV telemetry at 100 Hz
//
// ---- Core 1 -----------------------------------------------
//   ELRS/CRSF parser (async)
//   FFT vibration analysis → dynamic notch (optional)
//
// IDE: Arduino 2.3.8 + earlephilhower/arduino-pico >= 3.x
// ================================================================

#include <Arduino.h>
#include "pico/multicore.h"

#include "config.h"

// --- IMU ---
#include "imu/bmi088_driver.h"
#include "imu/imu_calibration.h"
#include "imu/madgwick_ahrs.h"

// --- Altitude ---
#include "altitude/bmp580_driver.h"
#include "altitude/altitude_estimator.h"
#include "altitude/altitude_hold.h"

// --- Filters ---
#include "filters/biquad_filter.h"
#include "filters/rpm_notch.h"

// --- Motor drivers ---
#if defined(MOTOR_TYPE_BRUSHED)
  #include "motors/brushed_pwm.h"
#else
  #include "motors/dshot_pio.h"
  #include "motors/esc_telemetry.h"
#endif
#include "motors/esc_calibration.h"

// --- Control ---
#include "control/pid_controller.h"
#include "control/flight_modes.h"

// --- RC calibration ---
#include "receiver_cal/rc_calibration.h"

// --- Receiver ---
#include "receiver/elrs_receiver.h"

// --- Safety ---
#include "safety/failsafe.h"

// --- Battery ---
#ifdef USE_BATTERY_MON
  #include "battery/battery.h"
#endif

// --- Blackbox ---
#ifdef USE_BLACKBOX
  #include "blackbox/blackbox.h"
  #include "blackbox/bb_dump.h"
#endif

// --- Telemetry ---
#include "telemetry/usb_telemetry.h"

// --- MSP ---
#include "msp/msp_protocol.h"

// --- Self-test ---
#include "system/self_test.h"

// --- Optional FFT ---
#ifdef USE_FFT_ANALYSIS
  #include "fft/fft_cmsis.h"
#endif

// ================================================================
// Global objects (some extern'd by telemetry / MSP modules)
// ================================================================

PIDAxis pidRoll, pidPitch, pidYaw;

// Motor outputs accessed by MSP module via extern
float m1Out=0, m2Out=0, m3Out=0, m4Out=0;

static BiquadFilter lpfGx, lpfGy, lpfGz;

// ================================================================
// Loop counters and state
// ================================================================

static uint32_t lastLoopUs     = 0;
static uint32_t loopTime       = 1000;
static uint32_t baroCounter    = 0;
static uint32_t telemCounter   = 0;
static uint32_t batCounter     = 0;
static float    lastPressure   = 101325.0f;
static bool     escCalRunning  = false;
static bool     bbDumpRunning  = false;

// RC calibration sweep feed
static bool     rcCalSweepActive = false;

// ================================================================
// Core 1 — ELRS parser + optional FFT
// ================================================================

void core1Task()
{
    crsfInit();

#ifdef USE_FFT_ANALYSIS
    fftProInit();
#endif

    while (true)
    {
        crsfTask();

#ifdef USE_FFT_ANALYSIS
        IMURaw imu = bmi088GetLatest();
        fftProAddSample(imu.gx * RAD_TO_DEG);

        static uint32_t fftMs = 0;
        if (millis() - fftMs > 50 && fftProResultReady())
        {
            fftMs = millis();
            float hz = fftProGetPeakHz();
            if (hz > 20.0f && hz < 900.0f)
                rpmFilterUpdateRPM(0,
                    (uint16_t)(hz * 60.0f / MOTOR_POLE_PAIRS));
        }
        delayMicroseconds(400);
#endif
    }
}

// ================================================================
// setup() — Core 0
// ================================================================

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[RP2354A-FC v4] =====================");

    // ---- Motor driver ----------------------------------------
#if defined(MOTOR_TYPE_BRUSHED)
    brushedInit();
    Serial.println("[FC] Motor: BRUSHED PWM");
#else
    dshotPioInit();
    escTelemetryInit();
    Serial.println("[FC] Motor: BLDC DShot600 PIO");
#endif

    // ---- Failsafe + battery ----------------------------------
    failsafeInit();

#ifdef USE_BATTERY_MON
    batInit();
#endif

    // ---- IMU -------------------------------------------------
    bmi088Init();

    // ---- AHRS ------------------------------------------------
    madgwickInit(MADGWICK_RATE_HZ, MADGWICK_BETA);

    // ---- Gyro filters ----------------------------------------
    biquadInitLPF(&lpfGx, GYRO_LPF_CUTOFF_HZ, (float)IMU_RATE_HZ);
    biquadInitLPF(&lpfGy, GYRO_LPF_CUTOFF_HZ, (float)IMU_RATE_HZ);
    biquadInitLPF(&lpfGz, GYRO_LPF_CUTOFF_HZ, (float)IMU_RATE_HZ);

#ifdef USE_RPM_FILTER
    rpmFilterInit((float)IMU_RATE_HZ);
#endif

    // ---- PID -------------------------------------------------
    pidAxisInit(&pidRoll,  PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  PID_ROLL_KFF,
                PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    pidAxisInit(&pidPitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_KFF,
                PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    pidAxisInit(&pidYaw,   PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   PID_YAW_KFF,
                PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);

    // ---- Barometer -------------------------------------------
    if (!bmp580Init())
        Serial.println("[WARN] BMP580 not found");
    lastPressure = bmp580ReadPressure();
    altEstInit(lastPressure);
    altHoldInit();

    // ---- RC calibration defaults ----------------------------
    rcCalInit();

    // ---- Blackbox --------------------------------------------
#ifdef USE_BLACKBOX
    if (!bbInit())
        Serial.println("[WARN] Blackbox flash not found");
#endif

    // ---- MSP -------------------------------------------------
    mspInit();

    // ---- Safety: start disarmed ------------------------------
    motorDisarm();

    // ---- IMU calibration (2 s, keep still) ------------------
    imuCalibStart();
    Serial.println("[FC] IMU calibration — keep board still for 2 s...");
    // Run calibration synchronously here (blocks 2 s before flight loop)
    {
        uint32_t calEnd = millis() + 2100;
        while (millis() < calEnd || imuCalibRunning())
        {
            IMURaw r = bmi088GetLatest();
            imuCalibUpdate(r.gx, r.gy, r.gz, r.ax, r.ay, r.az);
            delay(1);
        }
    }
    Serial.println("[FC] Calibration done");

    // ---- Self-test ------------------------------------------
    selfTestRun();
    if (!selfTestAllPassed())
        Serial.println("[WARN] Self-test failures — check hardware before flight!");

    // ---- Launch Core 1 --------------------------------------
    multicore_launch_core1(core1Task);

    lastLoopUs = micros();
    Serial.println("[FC] Flight loop running");
    Serial.println("[FC] =====================");
}

// ================================================================
// loop() — Core 0 @ 1 kHz
// ================================================================

void loop()
{
    // ---- Blackbox dump (bench only — blocks flight loop) --------
#ifdef USE_BLACKBOX
    if (bbDumpActive())
    {
        bbDumpUpdate();
        return;
    }
#endif

    // ---- ESC calibration override (bench only) ------------------
    if (escCalActive())
    {
        escCalUpdate(m1Out, m2Out, m3Out, m4Out);
#if defined(MOTOR_TYPE_BRUSHED)
        brushedWriteMotors(m1Out, m2Out, m3Out, m4Out);
#else
        dshotPioWriteMotors(m1Out, m2Out, m3Out, m4Out);
#endif
        return;
    }

    // ---- 1 kHz timing gate --------------------------------------
    uint32_t now = micros();
    if (now - lastLoopUs < (uint32_t)FLIGHT_LOOP_US) return;
    loopTime   = now - lastLoopUs;
    lastLoopUs = now;
    float dt   = loopTime * 1e-6f;

    // ==============================================================
    // 1. IMU + calibration correction
    // ==============================================================
    IMURaw imuRaw = bmi088GetLatest();
    IMUCalibration cal = imuCalibGet();

    float gx = imuRaw.gx - cal.gyroBias[0];
    float gy = imuRaw.gy - cal.gyroBias[1];
    float gz = imuRaw.gz - cal.gyroBias[2];
    float ax = imuRaw.ax - cal.accelBias[0];
    float ay = imuRaw.ay - cal.accelBias[1];
    float az = imuRaw.az - cal.accelBias[2];

    // ==============================================================
    // 2. Gyro filter pipeline
    // ==============================================================
#ifdef USE_RPM_FILTER
    gx = rpmFilterApply(gx);
    gy = rpmFilterApply(gy);
    gz = rpmFilterApply(gz);
#endif
    gx = biquadApply(&lpfGx, gx);
    gy = biquadApply(&lpfGy, gy);
    gz = biquadApply(&lpfGz, gz);

    // ==============================================================
    // 3. Madgwick AHRS
    // ==============================================================
    madgwickUpdate(gx, gy, gz, ax, ay, az);
    float rollDeg, pitchDeg, yawDeg;
    madgwickGetEuler(&rollDeg, &pitchDeg, &yawDeg);

    // ==============================================================
    // 4. Barometer + altitude (50 Hz)
    // ==============================================================
    if (++baroCounter >= (uint32_t)BARO_DIVIDER)
    {
        baroCounter  = 0;
        lastPressure = bmp580ReadPressure();
    }
    altEstUpdate(az, lastPressure, dt);
    float altitude = altEstGetAltitude();

    // ==============================================================
    // 5. Battery monitor (50 Hz)
    // ==============================================================
    uint16_t volt_mV = 0;
    int16_t  curr_cA = 0;
    uint8_t  bat_pct = 100;
#ifdef USE_BATTERY_MON
    if (++batCounter >= 20) { batCounter = 0; batUpdate(); }
    volt_mV = batGetVoltage_mV();
    curr_cA = batGetCurrent_cA();
    bat_pct = batGetPercent();
#endif

    // ==============================================================
    // 6. Failsafe
    // ==============================================================
    bool batCrit = false;
#ifdef USE_BATTERY_MON
    batCrit = batIsCritical();
#endif
    failsafeUpdate(crsfIsLinkUp(), batCrit);

    // ==============================================================
    // 7. RC input with calibration
    // ==============================================================
    CRSFData rc = crsfGet();

    // Feed RC cal sweep if active
    if (rcCalActive() && rc.valid)
        rcCalFeed(rc.raw);

    // Apply per-channel calibration
    float throttle   = rcCalApply(2, rc.raw[2], true);   // CH3: 0..1
    float rollStick  = rcCalApply(0, rc.raw[0], false);   // CH1
    float pitchStick = rcCalApply(1, rc.raw[1], false);   // CH2
    float yawStick   = rcCalApply(3, rc.raw[3], false);   // CH4

    // Failsafe throttle override
    if (failsafeActive())
        throttle = failsafeGetThrottle(throttle);

    // ARM/DISARM + mode
    checkArmDisarm(throttle, yawStick);
    checkArmSwitch(crsfCH5());
    checkModeSwitch(crsfCH6());

    // Altitude hold: CH7 > 1500 µs
    bool altHoldActive = (crsfCH7() > 1500) && isArmed();
    if (altHoldActive)
        throttle = altHoldUpdate(throttle, dt);

    // ==============================================================
    // 8. Flight mode + rate/angle setpoints
    // ==============================================================
    float rollRateSP, pitchRateSP;
    FlightMode mode = flightModeGet();

    if (mode == MODE_ACRO)
    {
        rollRateSP  = acroRateSetpoint(rollStick)  * DEG_TO_RAD;
        pitchRateSP = acroRateSetpoint(pitchStick) * DEG_TO_RAD;
    }
    else
    {
        rollRateSP  = angleModeSetpoint(rollStick,  rollDeg)  * DEG_TO_RAD;
        pitchRateSP = angleModeSetpoint(pitchStick, pitchDeg) * DEG_TO_RAD;
    }
    float yawRateSP = acroRateSetpoint(yawStick) * DEG_TO_RAD;

    // ==============================================================
    // 9. TPA — Throttle PID Attenuation
    // ==============================================================
    // Scale Kp and Kd down above TPA_BREAKPOINT to prevent high-
    // throttle oscillation on BLDC builds where motors become faster
    // and more responsive at high RPM.
    // Yaw is intentionally excluded — it needs full authority always.
    // tpaFactor: 1.0 below breakpoint, ramps to (1 − TPA_RATE) at full throttle.
    float tpaFactor = 1.0f;
    if (throttle > TPA_BREAKPOINT)
        tpaFactor = 1.0f - TPA_RATE *
                    constrain((throttle - TPA_BREAKPOINT) / (1.0f - TPA_BREAKPOINT),
                               0.0f, 1.0f);

    // Temporarily scale Kp/Kd for this loop tick only
    float savedRollKp  = pidRoll.kp;   float savedRollKd  = pidRoll.kd;
    float savedPitchKp = pidPitch.kp;  float savedPitchKd = pidPitch.kd;
    pidRoll.kp  *= tpaFactor;  pidRoll.kd  *= tpaFactor;
    pidPitch.kp *= tpaFactor;  pidPitch.kd *= tpaFactor;

    // ==============================================================
    // 10. PID  (Feedforward + I-term Relax built into pidAxisUpdate)
    // ==============================================================
    float rollOut  = pidAxisUpdate(&pidRoll,  rollRateSP,  gx, dt);
    float pitchOut = pidAxisUpdate(&pidPitch, pitchRateSP, gy, dt);
    float yawOut   = pidAxisUpdate(&pidYaw,   yawRateSP,   gz, dt);

    // Restore Kp/Kd (TPA scaling is per-tick, not permanent)
    pidRoll.kp  = savedRollKp;   pidRoll.kd  = savedRollKd;
    pidPitch.kp = savedPitchKp;  pidPitch.kd = savedPitchKd;

    // ==============================================================
    // 11. Safety gate + Quad-X mixer
    // ==============================================================
    if (!isArmed() || failsafeGet() == FS_KILL)
    {
        m1Out = m2Out = m3Out = m4Out = 0.0f;
        if (!isArmed())
        {
            pidAxisReset(&pidRoll);
            pidAxisReset(&pidPitch);
            pidAxisReset(&pidYaw);
        }
    }
    else
    {
        // Quad-X: M1=FR, M2=RR, M3=RL, M4=FL
        m1Out = throttle + rollOut + pitchOut - yawOut;
        m2Out = throttle - rollOut - pitchOut - yawOut;
        m3Out = throttle - rollOut + pitchOut + yawOut;
        m4Out = throttle + rollOut - pitchOut + yawOut;

        // ------------------------------------------------------------
        // Airmode desaturation
        // ------------------------------------------------------------
        // When any motor output is below zero, shift all four motors
        // upward by the same amount so the minimum becomes zero.
        // This preserves the differential between motors (control
        // authority) without changing the relative thrust balance,
        // giving the pilot full PID authority even at zero throttle.
        //
        // Without Airmode: motors clamp at 0.0, authority is lost.
        // With Airmode:    the craft remains fully controllable.
        // Disable for beginners (AIRMODE_ENABLED = 0 in config.h).
#if AIRMODE_ENABLED
        float minOut = min(min(m1Out, m2Out), min(m3Out, m4Out));
        if (minOut < 0.0f)
        {
            m1Out -= minOut;
            m2Out -= minOut;
            m3Out -= minOut;
            m4Out -= minOut;
        }
#endif

        // Clamp upper bound after Airmode shift
        m1Out = constrain(m1Out, 0.0f, 1.0f);
        m2Out = constrain(m2Out, 0.0f, 1.0f);
        m3Out = constrain(m3Out, 0.0f, 1.0f);
        m4Out = constrain(m4Out, 0.0f, 1.0f);
    }

    // Motor test overrides all (bench, disarmed only)
    motorTestOverride(m1Out, m2Out, m3Out, m4Out);

    // ==============================================================
    // 12. Motor output
    // ==============================================================
#if defined(MOTOR_TYPE_BRUSHED)
    brushedWriteMotors(m1Out, m2Out, m3Out, m4Out);
#else
    dshotPioWriteMotors(m1Out, m2Out, m3Out, m4Out);
    escTelemetryUpdate();
#endif

    // ==============================================================
    // 13. Blackbox write on arm edge
    // ==============================================================
#ifdef USE_BLACKBOX
    static bool wasArmed = false;
    if (isArmed() && !wasArmed) bbSessionStart();
    if (!isArmed() && wasArmed) bbSessionEnd();
    wasArmed = isArmed();

    if (isArmed() && bbReady())
    {
        BBFrame bbf;
        bbf.timestamp_us = micros();
        bbf.roll_x10     = (int16_t)(rollDeg  * 10);
        bbf.pitch_x10    = (int16_t)(pitchDeg * 10);
        bbf.yaw_x10      = (int16_t)(yawDeg   * 10);
        bbf.gx_x100      = (int16_t)(gx * RAD_TO_DEG * 100);
        bbf.gy_x100      = (int16_t)(gy * RAD_TO_DEG * 100);
        bbf.gz_x100      = (int16_t)(gz * RAD_TO_DEG * 100);
        bbf.m1 = (uint16_t)(m1Out*1000); bbf.m2 = (uint16_t)(m2Out*1000);
        bbf.m3 = (uint16_t)(m3Out*1000); bbf.m4 = (uint16_t)(m4Out*1000);
        bbf.altitude_cm  = (uint16_t)constrain((int)(altitude*100), 0, 65535);
        bbf.voltage_mV   = volt_mV;
        bbf.armed_mode   = (uint8_t)((isArmed()?0x80:0)|((uint8_t)mode&0x03));
        bbf.flags        = (uint8_t)((altHoldActive?0x01:0));
        bbf.loop_time_us = (uint16_t)constrain((int)loopTime, 0, 65535);
        bbWrite(&bbf);
    }
#endif

    // ==============================================================
    // 14. Telemetry at 100 Hz (MSP + CSV)
    // ==============================================================
    if (++telemCounter >= (uint32_t)TELEMETRY_DIVIDER)
    {
        telemCounter = 0;

        // MSP handles its own incoming bytes (Betaflight Configurator)
        mspUpdate();

        // CSV command parser (Processing GUI)
        usbTelemetryParseCmd();

        uint16_t r1=0, r2=0, r3=0, r4=0;
#if !defined(MOTOR_TYPE_BRUSHED)
        r1=escGetRPM(0); r2=escGetRPM(1);
        r3=escGetRPM(2); r4=escGetRPM(3);
#endif

        usbTelemetrySend(
            rollDeg, pitchDeg, yawDeg,
            m1Out, m2Out, m3Out, m4Out,
            pidRoll.kp, pidRoll.ki, pidRoll.kd,
            altitude,
            isArmed(), (int)flightModeGet(),
            volt_mV, curr_cA, bat_pct,
            r1, r2, r3, r4,
            loopTime, (uint8_t)failsafeGet()
        );
    }
}
