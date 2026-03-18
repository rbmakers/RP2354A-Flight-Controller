#include "msp_protocol.h"
#include "../imu/bmi088_driver.h"
#include "../imu/madgwick_ahrs.h"
#include "../control/pid_controller.h"
#include "../control/flight_modes.h"
#include "../altitude/altitude_estimator.h"
#include "../receiver/elrs_receiver.h"
#include "../telemetry/usb_telemetry.h"

#ifdef USE_BATTERY_MON
  #include "../battery/battery.h"
#endif
#ifdef USE_BLACKBOX
  #include "../blackbox/blackbox.h"
#endif

// ================================================================
// MSP command IDs
// ================================================================
#define MSP_IDENT      100
#define MSP_STATUS     101
#define MSP_RAW_IMU    102
#define MSP_MOTOR      104
#define MSP_RC         105
#define MSP_ATTITUDE   108
#define MSP_ALTITUDE   109
#define MSP_ANALOG     110
#define MSP_SET_PID    202
#define MSP_PID        112

// ================================================================
// External PID axes (defined in main .ino)
// ================================================================
extern PIDAxis pidRoll, pidPitch, pidYaw;

// ================================================================
// Frame parser state machine
// ================================================================
static enum {
    MSP_IDLE,
    MSP_HEADER_M,
    MSP_HEADER_ARROW,
    MSP_HEADER_SIZE,
    MSP_HEADER_CMD,
    MSP_PAYLOAD,
    MSP_CHECKSUM
} _mspState = MSP_IDLE;

#define MSP_MAX_PAYLOAD  64
static uint8_t  _mspSize      = 0;
static uint8_t  _mspCmd       = 0;
static uint8_t  _mspBuf[MSP_MAX_PAYLOAD];
static uint8_t  _mspBufIdx    = 0;
static uint8_t  _mspChecksum  = 0;

// ================================================================
// Frame builder helpers
// ================================================================

static void mspBeginResponse(uint8_t cmd, uint8_t payloadSize)
{
    uint8_t header[5] = {
        '$', 'M', '>',
        payloadSize,
        cmd
    };
    Serial.write(header, 5);
}

static void mspWriteU8 (uint8_t v)  { Serial.write(&v, 1); }
static void mspWriteU16(uint16_t v) { Serial.write((uint8_t*)&v, 2); }
static void mspWriteU32(uint32_t v) { Serial.write((uint8_t*)&v, 4); }
static void mspWriteS16(int16_t v)  { Serial.write((uint8_t*)&v, 2); }

// Write checksum byte = XOR of (size, cmd, payload[0..n-1])
// We track it manually as we write each byte.
static uint8_t _txChecksum = 0;

static void mspStartTx(uint8_t cmd, uint8_t payloadSize)
{
    _txChecksum = payloadSize ^ cmd;
    mspBeginResponse(cmd, payloadSize);
}

static void mspTxU8(uint8_t v)
{
    _txChecksum ^= v;
    mspWriteU8(v);
}

static void mspTxU16(uint16_t v)
{
    _txChecksum ^= (uint8_t)(v & 0xFF);
    _txChecksum ^= (uint8_t)(v >> 8);
    mspWriteU16(v);
}

static void mspTxS16(int16_t v)
{
    mspTxU16((uint16_t)v);
}

static void mspTxU32(uint32_t v)
{
    _txChecksum ^= (uint8_t)(v & 0xFF);
    _txChecksum ^= (uint8_t)((v >> 8)  & 0xFF);
    _txChecksum ^= (uint8_t)((v >> 16) & 0xFF);
    _txChecksum ^= (uint8_t)((v >> 24) & 0xFF);
    mspWriteU32(v);
}

static void mspEndTx()
{
    mspWriteU8(_txChecksum);
}

// ================================================================
// MSP response builders
// ================================================================

static void sendMSP_IDENT()
{
    // version=250, multiType=3 (QUADX), mspVersion=2, capability=0
    mspStartTx(MSP_IDENT, 7);
    mspTxU8(250);          // version
    mspTxU8(3);            // multitype: QUADX
    mspTxU8(2);            // MSP version
    mspTxU32(0);           // capability flags
    mspEndTx();
}

static void sendMSP_STATUS()
{
    // cycleTime, i2cErrors, sensorPresent flags, boxActivation, currentProfile
    uint16_t sensors = (1 << 0) |  // ACC
                       (1 << 1) |  // BARO
                       (1 << 2);   // MAG (not present but flag for GUI compat)

    uint32_t boxActivation = isArmed() ? (1 << 0) : 0;
    // bit 1 = ANGLE mode, bit 0 = ARM
    if (flightModeGet() == MODE_ANGLE) boxActivation |= (1 << 1);

    mspStartTx(MSP_STATUS, 11);
    mspTxU16(1000);          // cycleTime µs
    mspTxU16(0);             // i2cErrorCount
    mspTxU16(sensors);       // sensor flags
    mspTxU32(boxActivation); // activation flags
    mspTxU8(0);              // currentProfile
    mspEndTx();
}

static void sendMSP_RAW_IMU()
{
    IMURaw imu = bmi088GetLatest();
    // Configurator expects: acc in units of 512/g, gyro in units of (1/4.096) °/s
    int16_t ax = (int16_t)(imu.ax * 512.0f);
    int16_t ay = (int16_t)(imu.ay * 512.0f);
    int16_t az = (int16_t)(imu.az * 512.0f);
    int16_t gx = (int16_t)(imu.gx * RAD_TO_DEG * 4.096f);
    int16_t gy = (int16_t)(imu.gy * RAD_TO_DEG * 4.096f);
    int16_t gz = (int16_t)(imu.gz * RAD_TO_DEG * 4.096f);

    mspStartTx(MSP_RAW_IMU, 18);
    mspTxS16(ax); mspTxS16(ay); mspTxS16(az);   // acc
    mspTxS16(gx); mspTxS16(gy); mspTxS16(gz);   // gyro
    mspTxS16(0);  mspTxS16(0);  mspTxS16(0);    // mag (not present)
    mspEndTx();
}

static void sendMSP_MOTOR()
{
    // 8 motors, 1000–2000, unused motors = 0
    // We only have 4; pad with 0 for slots 5–8
    // Motor values from last loop cycle (extern from main sketch)
    // Accessed via a getter — we read them from blackbox frame if available,
    // otherwise report zero for motors 5–8.
    mspStartTx(MSP_MOTOR, 16);
    // Motors stored as global in main.ino — accessed via a simple extern trick
    extern float m1Out, m2Out, m3Out, m4Out;
    mspTxU16((uint16_t)(1000 + m1Out * 1000));
    mspTxU16((uint16_t)(1000 + m2Out * 1000));
    mspTxU16((uint16_t)(1000 + m3Out * 1000));
    mspTxU16((uint16_t)(1000 + m4Out * 1000));
    mspTxU16(0); mspTxU16(0); mspTxU16(0); mspTxU16(0);
    mspEndTx();
}

static void sendMSP_RC()
{
    // 8 channels in µs (1000–2000)
    // Map CRSF raw (172–1811) → µs (1000–2000)
    CRSFData rc = crsfGet();
    mspStartTx(MSP_RC, 16);
    for (int i = 0; i < 8; i++)
    {
        uint16_t raw = (i < CRSF_CHANNELS) ? rc.raw[i] : 992;
        // CRSF: 172=min, 992=mid, 1811=max → map to 1000–2000
        uint16_t us = (uint16_t)(1000 + (raw - 172) * 1000 / (1811 - 172));
        us = constrain(us, 1000, 2000);
        mspTxU16(us);
    }
    mspEndTx();
}

static void sendMSP_ATTITUDE()
{
    float roll, pitch, yaw;
    madgwickGetEuler(&roll, &pitch, &yaw);
    // Configurator expects: roll/pitch in degrees×10, yaw in degrees
    mspStartTx(MSP_ATTITUDE, 6);
    mspTxS16((int16_t)(roll  * 10));
    mspTxS16((int16_t)(pitch * 10));
    mspTxS16((int16_t)(yaw));
    mspEndTx();
}

static void sendMSP_ALTITUDE()
{
    int32_t alt_cm  = (int32_t)(altEstGetAltitude()      * 100);
    int16_t vario   = (int16_t)(altEstGetVerticalSpeed()  * 100);
    mspStartTx(MSP_ALTITUDE, 6);
    mspTxU32((uint32_t)alt_cm);
    mspTxS16(vario);
    mspEndTx();
}

static void sendMSP_ANALOG()
{
#ifdef USE_BATTERY_MON
    uint8_t  vBat  = (uint8_t)(batGetVoltage_mV() / 100);  // 0.1 V units
    uint16_t mAh   = 0;    // consumption tracking not implemented
    uint16_t rssi  = crsfIsLinkUp() ? 1000 : 0;            // 0–1023
    uint16_t curr  = (uint16_t)(batGetCurrent_cA() * 10);  // 0.01 A units
#else
    uint8_t  vBat  = 168;  // 16.8 V = full 4S (nominal)
    uint16_t mAh   = 0;
    uint16_t rssi  = crsfIsLinkUp() ? 1000 : 0;
    uint16_t curr  = 0;
#endif

    mspStartTx(MSP_ANALOG, 7);
    mspTxU8(vBat);
    mspTxU16(mAh);
    mspTxU16(rssi);
    mspTxS16((int16_t)curr);
    mspEndTx();
}

static void sendMSP_PID()
{
    // Betaflight PID format: each axis = {P, I, D} as uint8 (0–255)
    // We scale: P/10, I*100, D*100 as rough mapping to BF units
    auto toP = [](float v) -> uint8_t { return (uint8_t)constrain(v * 10, 0, 255); };
    auto toI = [](float v) -> uint8_t { return (uint8_t)constrain(v * 1000, 0, 255); };
    auto toD = [](float v) -> uint8_t { return (uint8_t)constrain(v * 1000, 0, 255); };

    mspStartTx(MSP_PID, 30);  // 10 × 3 bytes
    // Roll
    mspTxU8(toP(pidRoll.kp));  mspTxU8(toI(pidRoll.ki));  mspTxU8(toD(pidRoll.kd));
    // Pitch
    mspTxU8(toP(pidPitch.kp)); mspTxU8(toI(pidPitch.ki)); mspTxU8(toD(pidPitch.kd));
    // Yaw
    mspTxU8(toP(pidYaw.kp));   mspTxU8(toI(pidYaw.ki));   mspTxU8(toD(pidYaw.kd));
    // Alt, Pos, PosR, NavR, Level, Mag, Vel — all zeros for now
    for (int i = 0; i < 7 * 3; i++) mspTxU8(0);
    mspEndTx();
}

// ================================================================
// MSP_SET_PID handler — receive PID gains from Configurator
// ================================================================
static void handleMSP_SET_PID(const uint8_t *payload, uint8_t size)
{
    if (size < 9) return;

    // Inverse of sendMSP_PID scaling
    pidRoll.kp  = payload[0] / 10.0f;
    pidRoll.ki  = payload[1] / 1000.0f;
    pidRoll.kd  = payload[2] / 1000.0f;

    pidPitch.kp = payload[3] / 10.0f;
    pidPitch.ki = payload[4] / 1000.0f;
    pidPitch.kd = payload[5] / 1000.0f;

    pidYaw.kp   = payload[6] / 10.0f;
    pidYaw.ki   = payload[7] / 1000.0f;
    pidYaw.kd   = payload[8] / 1000.0f;

    Serial.print("[MSP] PID updated: roll P=");
    Serial.print(pidRoll.kp, 3);
    Serial.print(" I="); Serial.print(pidRoll.ki, 3);
    Serial.print(" D="); Serial.println(pidRoll.kd, 3);

    // Send acknowledgment (empty payload response)
    mspStartTx(MSP_SET_PID, 0);
    mspEndTx();
}

// ================================================================
// Dispatch a complete received frame
// ================================================================
static void mspDispatch(uint8_t cmd, const uint8_t *payload, uint8_t size)
{
    switch (cmd)
    {
    case MSP_IDENT:     sendMSP_IDENT();    break;
    case MSP_STATUS:    sendMSP_STATUS();   break;
    case MSP_RAW_IMU:   sendMSP_RAW_IMU(); break;
    case MSP_MOTOR:     sendMSP_MOTOR();    break;
    case MSP_RC:        sendMSP_RC();       break;
    case MSP_ATTITUDE:  sendMSP_ATTITUDE(); break;
    case MSP_ALTITUDE:  sendMSP_ALTITUDE(); break;
    case MSP_ANALOG:    sendMSP_ANALOG();   break;
    case MSP_PID:       sendMSP_PID();      break;
    case MSP_SET_PID:   handleMSP_SET_PID(payload, size); break;
    default:
        // Unknown command — send error frame
        Serial.write('$'); Serial.write('M'); Serial.write('!');
        mspWriteU8(0); mspWriteU8(cmd); mspWriteU8(cmd);
        break;
    }
}

// ================================================================
// Public API
// ================================================================

void mspInit()
{
    _mspState = MSP_IDLE;
    Serial.println("[MSP] v1 protocol ready (Betaflight Configurator compatible)");
}

void mspUpdate()
{
    while (Serial.available())
    {
        uint8_t b = (uint8_t)Serial.read();

        switch (_mspState)
        {
        case MSP_IDLE:
            if (b == '$') _mspState = MSP_HEADER_M;
            else
            {
                // Not MSP — forward to CSV command parser
                // Rebuild char and push to a small buffer for usbTelemetryParseCmd
                // (In practice: both parsers are called and CSV commands start
                //  with ASCII letters, never '$', so no collision occurs.)
            }
            break;

        case MSP_HEADER_M:
            _mspState = (b == 'M') ? MSP_HEADER_ARROW : MSP_IDLE;
            break;

        case MSP_HEADER_ARROW:
            if (b == '<')
            {
                _mspChecksum = 0;
                _mspState    = MSP_HEADER_SIZE;
            }
            else _mspState = MSP_IDLE;
            break;

        case MSP_HEADER_SIZE:
            _mspSize     = b;
            _mspChecksum = b;
            _mspBufIdx   = 0;
            _mspState    = MSP_HEADER_CMD;
            break;

        case MSP_HEADER_CMD:
            _mspCmd       = b;
            _mspChecksum ^= b;
            _mspState     = (_mspSize > 0) ? MSP_PAYLOAD : MSP_CHECKSUM;
            break;

        case MSP_PAYLOAD:
            if (_mspBufIdx < MSP_MAX_PAYLOAD)
                _mspBuf[_mspBufIdx++] = b;
            _mspChecksum ^= b;
            if (_mspBufIdx >= _mspSize) _mspState = MSP_CHECKSUM;
            break;

        case MSP_CHECKSUM:
            if (b == _mspChecksum)
                mspDispatch(_mspCmd, _mspBuf, _mspSize);
            else
            {
                Serial.print("[MSP] Checksum error on cmd ");
                Serial.println(_mspCmd);
            }
            _mspState = MSP_IDLE;
            break;
        }
    }
}
