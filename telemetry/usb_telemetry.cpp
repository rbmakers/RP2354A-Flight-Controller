#include "usb_telemetry.h"
#include "../control/pid_controller.h"
#include "../control/flight_modes.h"
#include "../safety/failsafe.h"
#include "../imu/imu_calibration.h"
#include "../receiver_cal/rc_calibration.h"
#include "../motors/esc_calibration.h"
#include "../system/self_test.h"

#ifdef USE_BLACKBOX
  #include "../blackbox/bb_dump.h"
  #include "../blackbox/blackbox.h"
#endif

// External PID axes and motor outputs declared in main .ino
extern PIDAxis pidRoll, pidPitch, pidYaw;

// ================================================================
// Send one 22-field telemetry packet
// ================================================================
void usbTelemetrySend(
    float roll, float pitch, float yaw,
    float m1, float m2, float m3, float m4,
    float kp, float ki, float kd,
    float altitude,
    bool  armed, int mode,
    uint16_t volt_mV, int16_t curr_cA, uint8_t bat_pct,
    uint16_t r1rpm, uint16_t r2rpm, uint16_t r3rpm, uint16_t r4rpm,
    uint32_t loop_us, uint8_t fs_level)
{
    // Skip telemetry during blackbox dump to avoid corrupting binary stream
#ifdef USE_BLACKBOX
    if (bbDumpActive()) return;
#endif

    auto mi = [](float v) -> int { return (int)(constrain(v, 0.0f, 1.0f) * 1000); };

    // Angles × 10 as integers — smaller than float strings, no precision loss
    Serial.print((int)(roll  * 10)); Serial.print(',');
    Serial.print((int)(pitch * 10)); Serial.print(',');
    Serial.print((int)(yaw   * 10)); Serial.print(',');

    Serial.print(mi(m1)); Serial.print(',');
    Serial.print(mi(m2)); Serial.print(',');
    Serial.print(mi(m3)); Serial.print(',');
    Serial.print(mi(m4)); Serial.print(',');

    Serial.print(kp, 3); Serial.print(',');
    Serial.print(ki, 3); Serial.print(',');
    Serial.print(kd, 3); Serial.print(',');

    Serial.print((int)(altitude * 100)); Serial.print(',');  // cm

    Serial.print(armed ? 1 : 0); Serial.print(',');
    Serial.print(mode);          Serial.print(',');

    Serial.print(volt_mV);  Serial.print(',');
    Serial.print(curr_cA);  Serial.print(',');
    Serial.print(bat_pct);  Serial.print(',');

    Serial.print(r1rpm); Serial.print(',');
    Serial.print(r2rpm); Serial.print(',');
    Serial.print(r3rpm); Serial.print(',');
    Serial.print(r4rpm); Serial.print(',');

    Serial.print(loop_us);  Serial.print(',');
    Serial.println(fs_level);
}

// ================================================================
// Receive and dispatch GUI commands
// ================================================================
static char    _cmdBuf[80];
static uint8_t _cmdIdx = 0;

void usbTelemetryParseCmd()
{
    while (Serial.available())
    {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r')
        {
            _cmdBuf[_cmdIdx] = '\0';
            String cmd = String(_cmdBuf);
            _cmdIdx = 0;
            if (cmd.length() == 0) continue;

            // ---- PID gains (roll axis; pitch/yaw follow same P,I,D) ----
            if (cmd.startsWith("SET_KP:"))
            {
                float v = cmd.substring(7).toFloat();
                pidRoll.kp = pidPitch.kp = v;
                Serial.print("[GUI] KP="); Serial.println(v, 3);
            }
            else if (cmd.startsWith("SET_KI:"))
            {
                float v = cmd.substring(7).toFloat();
                pidRoll.ki = pidPitch.ki = v;
                Serial.print("[GUI] KI="); Serial.println(v, 3);
            }
            else if (cmd.startsWith("SET_KD:"))
            {
                float v = cmd.substring(7).toFloat();
                pidRoll.kd = pidPitch.kd = v;
                Serial.print("[GUI] KD="); Serial.println(v, 3);
            }
            // SET_YAW_KP/KI/KD for independent yaw tuning
            else if (cmd.startsWith("SET_YAW_KP:"))
            {
                pidYaw.kp = cmd.substring(11).toFloat();
            }
            else if (cmd.startsWith("SET_YAW_KI:"))
            {
                pidYaw.ki = cmd.substring(11).toFloat();
            }
            else if (cmd.startsWith("SET_YAW_KD:"))
            {
                pidYaw.kd = cmd.substring(11).toFloat();
            }

            // ---- ARM / DISARM ----
            else if (cmd == "ARM")    motorArm();
            else if (cmd == "DISARM") motorDisarm();

            // ---- Flight modes ----
            else if (cmd == "MODE_ACRO")  flightModeSet(MODE_ACRO);
            else if (cmd == "MODE_ANGLE") flightModeSet(MODE_ANGLE);

            // ---- IMU calibration ----
            else if (cmd == "CALIB")
            {
                if (!isArmed())
                {
                    imuCalibStart();
                    Serial.println("[GUI] IMU calibration started — keep still");
                }
                else
                {
                    Serial.println("[GUI] CALIB refused — disarm first");
                }
            }

            // ---- RC calibration ----
            else if (cmd == "RC_CAL_START")
            {
                if (!isArmed())
                {
                    rcCalBegin();
                    Serial.println("[GUI] RC calibration started — move all sticks to extremes");
                }
                else Serial.println("[GUI] RC_CAL refused — disarm first");
            }
            else if (cmd == "RC_CAL_END")
            {
                rcCalEnd();
                Serial.println("[GUI] RC calibration complete");
            }
            else if (cmd == "RC_CAL_PRINT") { rcCalPrint(); }

            // ---- RC expo / deadband per channel ----
            // Format: SET_EXPO:<ch(0-7)>:<val(0.00-1.00)>
            else if (cmd.startsWith("SET_EXPO:"))
            {
                int sep = cmd.indexOf(':', 9);
                if (sep > 0)
                {
                    uint8_t ch  = (uint8_t)cmd.substring(9, sep).toInt();
                    float   val = cmd.substring(sep + 1).toFloat();
                    RCCalChannel c = rcCalGet(ch);
                    c.expo = constrain(val, 0.0f, 1.0f);
                    rcCalSet(ch, c);
                    Serial.print("[GUI] CH"); Serial.print(ch+1);
                    Serial.print(" expo="); Serial.println(val, 2);
                }
            }
            // Format: SET_DEADBAND:<ch(0-7)>:<val(0.00-0.20)>
            else if (cmd.startsWith("SET_DEADBAND:"))
            {
                int sep = cmd.indexOf(':', 13);
                if (sep > 0)
                {
                    uint8_t ch  = (uint8_t)cmd.substring(13, sep).toInt();
                    float   val = cmd.substring(sep + 1).toFloat();
                    RCCalChannel c = rcCalGet(ch);
                    c.deadband = constrain(val, 0.0f, 0.20f);
                    rcCalSet(ch, c);
                }
            }

            // ---- ESC calibration (removes props first!) ----
            else if (cmd == "ESC_CAL_START") { escCalStart(); }
            else if (cmd == "ESC_CAL_ABORT") { escCalAbort(); }

            // ---- Motor test ----
            // Format: MOTOR_TEST:<motor(0-3)>:<throttle_pct(0-15)>
            else if (cmd.startsWith("MOTOR_TEST:"))
            {
                int sep = cmd.indexOf(':', 11);
                if (sep > 0)
                {
                    uint8_t motor = (uint8_t)cmd.substring(11, sep).toInt();
                    float   pct   = cmd.substring(sep + 1).toFloat();
                    motorTestStart(motor, pct / 100.0f);
                }
            }
            else if (cmd == "MOTOR_TEST_STOP") { motorTestStop(); }
            else if (cmd == "MOTOR_TEST_ALL")  
            {
                // Spin all four at 10 % simultaneously
                for (uint8_t m = 0; m < 4; m++)
                    motorTestStart(m, 0.10f);
            }

            // ---- Auto-tune ----
            else if (cmd == "AUTOTUNE_START")
            {
                Serial.println("[GUI] AUTOTUNE not yet connected to flight loop");
                Serial.println("[GUI] Use manual PID sliders for now");
            }

            // ---- Self-test ----
            else if (cmd == "SELF_TEST")
            {
                selfTestRun();
                // Print results as structured lines for GUI to parse
                const SelfTestResult *r = selfTestGetResults();
                Serial.println("SELF_TEST_BEGIN");
                for (int i = 0; i < SELF_TEST_COUNT; i++)
                {
                    Serial.print("ST:");
                    Serial.print(r[i].passed ? "PASS" : "FAIL");
                    Serial.print(':');
                    Serial.print(r[i].name);
                    Serial.print(':');
                    Serial.println(r[i].detail);
                }
                Serial.println("SELF_TEST_END");
            }

            // ---- Blackbox ----
            else if (cmd == "ERASE_BB")
            {
#ifdef USE_BLACKBOX
                if (!isArmed())
                {
                    Serial.println("[GUI] Erasing blackbox flash...");
                    bbEraseAll();
                }
                else Serial.println("[GUI] ERASE_BB refused — disarm first");
#else
                Serial.println("[GUI] USE_BLACKBOX not enabled");
#endif
            }
            else if (cmd == "DUMP_BB")
            {
#ifdef USE_BLACKBOX
                if (!isArmed())
                    bbDumpStart();
                else
                    Serial.println("[GUI] DUMP_BB refused — disarm first");
#else
                Serial.println("[GUI] USE_BLACKBOX not enabled");
#endif
            }

            else
            {
                Serial.print("[GUI] Unknown cmd: "); Serial.println(cmd);
            }
        }
        else if (_cmdIdx < (uint8_t)(sizeof(_cmdBuf) - 1))
        {
            _cmdBuf[_cmdIdx++] = c;
        }
        else
        {
            _cmdIdx = 0;   // buffer overflow — reset
        }
    }
}
