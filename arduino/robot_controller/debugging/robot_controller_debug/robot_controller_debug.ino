#include "movement_debug.h"
#include "sensors_debug.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  Serial protocol
//
//  Pi → Arduino:
//    V,<vd>,<wd>\n   — set desired linear [m/s] and angular [rad/s] velocity
//    S\n             — emergency stop (zeroes setpoints, clears integrators)
//
//  Arduino → Pi (NEW DEBUG PROTOCOL):
//  [DEBUG] The O packet now transmits absolute ticks instead of velocity.
//    O,<timestamp_ms>,<absolute_ticksL>,<absolute_ticksR>\n
// ═══════════════════════════════════════════════════════════════════════════════

static String cmdBuf = "";

static const float VD_CHANGE_THRESH = 0.05f;   // [m/s]
static const float WD_CHANGE_THRESH = 0.10f;   // [rad/s]

void processCommand(const String& cmd)
{
    if (cmd.length() == 0) return;

    char prefix = cmd.charAt(0);

    if (prefix == 'V')
    {
        int c1 = cmd.indexOf(',');
        int c2 = cmd.indexOf(',', c1 + 1);

        if (c1 < 0 || c2 < 0) return;

        float new_vd = cmd.substring(c1 + 1, c2).toFloat();
        float new_wd = cmd.substring(c2 + 1).toFloat();

        if (fabs(new_vd - vd) > VD_CHANGE_THRESH ||
            fabs(new_wd - wd) > WD_CHANGE_THRESH)
        {
            reset_integrators();
        }

        vd = new_vd;
        wd = new_wd;
    }
    else if (prefix == 'S')
    {
        vd = 0.0f;
        wd = 0.0f;
        reset_integrators();
    }
}

void setup()
{
    Serial.begin(115200);

    delay(500);
    while (Serial.available()) Serial.read();

    movement_setup();
    sensors_setup();

    Serial.println("READY");
}

void loop()
{
    PID_update();
    sensors_update();

    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n') {
            processCommand(cmdBuf);
            cmdBuf = "";
        } else if (c != '\r') {
            if (cmdBuf.length() < 64) {
                cmdBuf += c;
            }
        }
    }
}
