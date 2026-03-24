"""
joystick_controller.py  —  Logitech F710 (D-mode) → Arduino serial

Arcade-drive layout:
  Left  Stick Y  →  linear  velocity (push forward = positive)
  Right Stick X  →  angular velocity (push right   = positive)
  Left Trigger   →  EMERGENCY STOP
  A / X button   →  quit

Key improvements over the original:
  • MAX_VD is set to a realistic value (tune it to your robot's actual top speed)
  • Velocity ramping: commands slew toward the stick target at a bounded rate
    so sudden stick movements don't spike the PID and wind up the integrator
  • Deadzone rescaling: no velocity jump at the deadzone boundary
  • Baud rate matches the Arduino (115200)
"""

import sys
import time
import threading

import pygame
import serial


# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION  ← tune these
# ═══════════════════════════════════════════════════════════════════════════════

SERIAL_PORT  = '/dev/ttyACM0'
BAUD_RATE    = 115200            # Must match Serial.begin() in .ino

# Physical top speeds — set these to what your robot can actually do.
# If the robot barely moves at vd=0.3, lower MAX_VD. If it feels sluggish,
# raise it.  Wrong values here → PID saturation → wd has no effect.
MAX_VD = 0.4   # [m/s]   linear  — ~0.3–0.5 is typical for a small robot
MAX_WD = 2.0   # [rad/s] angular — ~1.5–2.5 is typical

# Slew-rate limiter: how fast the commanded velocity can change per second.
# Lower = smoother but more sluggish response.  Higher = snappier but can
# cause the integrator to wind up if moved too fast.
VD_SLEW = 0.8   # [m/s  per second]
WD_SLEW = 4.0   # [rad/s per second]

DEADZONE     = 0.12   # Fraction of full stick travel to ignore
CONTROL_HZ   = 30     # Loop rate [Hz]

# Gamepad axis indices (Logitech F710, D-mode, pygame)
AXIS_LEFT_Y  = 1      # Forward/back
AXIS_RIGHT_X = 2      # Turn left/right
BTN_ESTOP    = 6      # Left trigger
BTN_QUIT_A   = 0      # A
BTN_QUIT_X   = 2      # X


# ═══════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def apply_deadzone(value: float, dz: float) -> float:
    """Rescale axis output so there is no velocity discontinuity at the edge."""
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)


def slew(current: float, target: float, rate: float, dt: float) -> float:
    """Move current toward target at most rate * dt per step."""
    delta = target - current
    max_delta = rate * dt
    if abs(delta) <= max_delta:
        return target
    return current + (max_delta if delta > 0 else -max_delta)


def drain_serial(ser: serial.Serial):
    """Background thread: discard all incoming bytes so the serial buffer
    doesn't fill up and cause delays."""
    while True:
        try:
            if ser.in_waiting > 0:
                ser.readline()
        except Exception:
            break


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick found. Connect your Logitech F710 and retry.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller: {joystick.get_name()}")

    # ── Serial ────────────────────────────────────────────────────────────────
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Serial: {SERIAL_PORT} @ {BAUD_RATE}")
        time.sleep(1.5)  # Let the Arduino finish its reset
        drain_t = threading.Thread(target=drain_serial, args=(ser,), daemon=True)
        drain_t.start()
    except Exception as e:
        print(f"Serial failed: {e} — running in print-only mode.")

    print("\n── Controls ───────────────────────────────────────────────────")
    print(f"  Left Stick Y   : Forward / Reverse  (max {MAX_VD} m/s)")
    print(f"  Right Stick X  : Turn               (max {MAX_WD} rad/s)")
    print("  Left Trigger   : EMERGENCY STOP")
    print("  A / X button   : Quit")
    print("────────────────────────────────────────────────────────────────\n")

    clock   = pygame.time.Clock()
    axes    = {}
    e_stop  = False
    running = True

    # Ramp state — current commanded velocities
    cur_vd  = 0.0
    cur_wd  = 0.0

    try:
        while running:
            dt = 1.0 / CONTROL_HZ   # nominal dt; close enough for slew calc

            # ── Event handling ────────────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

                elif event.type == pygame.JOYAXISMOTION:
                    axes[event.axis] = event.value

                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_ESTOP:
                        e_stop = True
                    elif event.button in (BTN_QUIT_A, BTN_QUIT_X):
                        running = False

                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == BTN_ESTOP:
                        e_stop = False

            # ── Velocity computation ──────────────────────────────────────────
            if e_stop:
                # Emergency stop: snap to zero immediately, reset ramp state
                cur_vd = 0.0
                cur_wd = 0.0
                cmd    = "S\n"
            else:
                # Raw stick values
                ly = apply_deadzone(axes.get(AXIS_LEFT_Y,  0.0), DEADZONE)
                rx = apply_deadzone(axes.get(AXIS_RIGHT_X, 0.0), DEADZONE)

                # Target velocities from stick position
                tgt_vd = -ly * MAX_VD   # Pygame Y is inverted (up = -1)
                tgt_wd =  rx * MAX_WD

                # Slew-rate limit: smooth out sudden stick movements.
                # This keeps the PID error small so the integrator stays tidy.
                cur_vd = slew(cur_vd, tgt_vd, VD_SLEW, dt)
                cur_wd = slew(cur_wd, tgt_wd, WD_SLEW, dt)

                cmd = f"V,{cur_vd:.3f},{cur_wd:.3f}\n"

            # ── Transmit ──────────────────────────────────────────────────────
            if ser:
                try:
                    ser.write(cmd.encode('utf-8'))
                except Exception as e:
                    print(f"Write error: {e}")

            print(f"\r{cmd.strip()}   ", end='', flush=True)

            clock.tick(CONTROL_HZ)

    except KeyboardInterrupt:
        pass

    finally:
        print()
        if ser:
            try:
                ser.write(b"S\n")   # Always send a stop on exit
                ser.close()
            except Exception:
                pass
        pygame.quit()
        print("Exited.")


if __name__ == "__main__":
    main()
