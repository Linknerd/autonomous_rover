import pygame
import serial
import time
import sys
import threading

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

MAX_VD = 2.0    # Max linear velocity (m/s)
MAX_WD = 4.0    # Max angular velocity (rad/s)
DEADZONE = 0.15 # Fraction of stick travel to ignore

def apply_deadzone(value, deadzone):
    """Rescales axis output so there's no velocity jump at the deadzone edge."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)

def drain_serial(ser):
    while True:
        try:
            if ser.in_waiting > 0:
                ser.readline()
        except Exception:
            break

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick/controller found. Connect your Logitech F710 and retry.")
        sys.exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to: {joystick.get_name()}")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
        time.sleep(2)  # Wait for Arduino reset
    except Exception as e:
        print(f"Failed to connect to {SERIAL_PORT}: {e}")
        print("Running in simulation mode (printing commands only).")
        ser = None

    if ser:
        drain_thread = threading.Thread(target=drain_serial, args=(ser,), daemon=True)
        drain_thread.start()

    print("\n--- Controls (Logitech F710 — D Mode) ---")
    print("Left Stick Y         : Forward (+up) / Reverse (+down)")
    print("Right Stick X        : Turn Right (+right) / Turn Left (+left)")
    print("  Combined sticks    : Arc turns, reverse turns — all work naturally")
    print("Left Trigger (btn 6) : EMERGENCY STOP (overrides everything)")
    print("A / X button         : Quit\n")

    clock = pygame.time.Clock()
    axes = {}
    e_stop = False
    running = True

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

                elif event.type == pygame.JOYAXISMOTION:
                    axes[event.axis] = event.value

                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 6:           # Left Trigger — E-STOP
                        e_stop = True
                    elif event.button in (0, 2):    # A (DInput) or X (XInput) — quit
                        running = False

                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == 6:
                        e_stop = False              # Release E-STOP

            # ── EMERGENCY STOP has absolute priority over all motion ──
            if e_stop:
                cmd = "S\n"
            else:
                # Arcade Drive: Left Stick Y = linear, Right Stick X = angular
                # Pygame axis convention: Y axis is -1 (up/forward), +1 (down/reverse)
                ly = apply_deadzone(axes.get(1, 0.0), DEADZONE)
                lx = apply_deadzone(axes.get(2, 0.0), DEADZONE)

                vd = -ly * MAX_VD   # Flip Y so pushing up = positive velocity
                wd =  lx * MAX_WD   # Right = positive angular velocity

                cmd = f"V,{vd:.2f},{wd:.2f}\n"

            if ser:
                ser.write(cmd.encode('utf-8'))
            print(f"Sent: {cmd.strip()}")

            clock.tick(20)  # 20 Hz control loop

    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            try:
                ser.write(b"S\n")   # Always stop rover on exit
                ser.close()
            except Exception:
                pass
        print("\nExiting.")
        pygame.quit()

if __name__ == "__main__":
    main()
