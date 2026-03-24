#include "movement.h"
#include "sensors.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  Serial protocol
//
//  Pi → Arduino:
//    V,<vd>,<wd>\n   — set desired linear [m/s] and angular [rad/s] velocity
//    S\n             — emergency stop (zeroes setpoints, clears integrators)
//
//  Arduino → Pi:
//    O,<v>,<w>\n     — measured odometry at 1/T Hz  (only output)
//    READY\n         — sent once on boot
// ═══════════════════════════════════════════════════════════════════════════════

static String cmdBuf = "";

// Change threshold: if a new setpoint differs by more than this, wipe the
// integrators so the controller responds instantly rather than unwinding.
static const float VD_CHANGE_THRESH = 0.05f;   // [m/s]
static const float WD_CHANGE_THRESH = 0.10f;   // [rad/s]

// ─────────────────────────────────────────────────────────────────────────────

void processCommand(const String& cmd)
{
    if (cmd.length() == 0) return;

    char prefix = cmd.charAt(0);

    // ── V,vd,wd ──────────────────────────────────────────────────────────────
    if (prefix == 'V')
    {
        int c1 = cmd.indexOf(',');
        int c2 = cmd.indexOf(',', c1 + 1);

        if (c1 < 0 || c2 < 0) return;  // malformed

        float new_vd = cmd.substring(c1 + 1, c2).toFloat();
        float new_wd = cmd.substring(c2 + 1).toFloat();

        // If the setpoint changes enough, reset the integrators so the motors
        // respond immediately rather than fighting against built-up windup.
        if (fabs(new_vd - vd) > VD_CHANGE_THRESH ||
            fabs(new_wd - wd) > WD_CHANGE_THRESH)
        {
            reset_integrators();
        }

        vd = new_vd;
        wd = new_wd;
        // No ACK sent — ACK traffic was polluting the serial stream.
    }

    // ── S (stop) ──────────────────────────────────────────────────────────────
    else if (prefix == 'S')
    {
        vd = 0.0f;
        wd = 0.0f;
        reset_integrators();
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);

    // Brief pause then flush any garbage bytes accumulated during reset
    delay(500);
    while (Serial.available()) Serial.read();

    movement_setup();
    sensors_setup();

    Serial.println("READY");
}

void loop()
{
    // Run the PID (applies motor commands, updates speed_L / speed_R)
    PID_update();

    // Transmit odometry at the configured interval
    sensors_update();

    // Drain and parse incoming serial commands
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n') {
            processCommand(cmdBuf);
            cmdBuf = "";
        } else if (c != '\r') {
            if (cmdBuf.length() < 64) {   // guard against buffer overflow
                cmdBuf += c;
            }
        }
    }
}
