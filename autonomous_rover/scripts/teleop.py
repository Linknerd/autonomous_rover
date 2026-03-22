import pygame
import serial
import time
import sys
import threading

# Windows usually uses COM ports (e.g., 'COM3'). Update this to your Arduino's port.
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Rover maximum velocities
MAX_VD = 1.0  # Max linear velocity (example: 1.0 m/s)
MAX_WD = 2.0  # Max angular velocity (example: 1.0 rad/s)

def drain_serial(ser):
    while True:
        try:
            if ser.in_waiting > 0:
                ser.readline()
        except:
            break

def main():
    # Initialize Pygame and Joystick
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick/controller found.")
        print("Please connect your Logitech controller and try again.")
        sys.exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to: {joystick.get_name()}")

    # Initialize Serial connection to Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
        # Wait a bit for Arduino to reset
        time.sleep(2)
    except Exception as e:
        print(f"Failed to connect to Serial Port {SERIAL_PORT}: {e}")
        print("Running in simulation mode (printing commands only).")
        ser = None

    if ser:
        drain_thread = threading.Thread(target=drain_serial, args=(ser,), daemon=True)
        drain_thread.start()

    print("\n--- Controls ---")
    print("Right Trigger : Movement (Forward)")
    print("Left Joystick : Turn (Left/Right)")
    print("Left Trigger  : Force Break (Stop)\n")

    clock = pygame.time.Clock()

    axes = {}
    left_trigger_pressed = False
    right_trigger_pressed = False
    running = True

    try:
        while running:
            # Handle Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_x:
                        running = False
                elif event.type == pygame.JOYAXISMOTION:
                    axes[event.axis] = event.value
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 6:
                        left_trigger_pressed = True
                    elif event.button == 7:
                        right_trigger_pressed = True
                    # Button 0 (DInput X) or 2 (XInput X) to stop
                    elif event.button in (0, 2):
                        running = False
                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == 6:
                        left_trigger_pressed = False
                    elif event.button == 7:
                        right_trigger_pressed = False

            # Define deadzones to prevent drift
            DEADZONE = 0.2
            lx = axes.get(0, 0.0)
            ly = axes.get(1, 0.0)
            rx = axes.get(2, 0.0)
            ry = axes.get(3, 0.0)

            # 1. Force Break (Left Trigger)
            if left_trigger_pressed:
                cmd = "S\n"
            else:
                vd = 0.0
                wd = 0.0

                # 2. Forward (Right Trigger)
                if right_trigger_pressed:
                    vd = 2.0
                    wd = 0.0
                else:
                    # 3. Joystick logic
                    # Right Joystick: diagonal movement
                    if abs(ry) > DEADZONE and abs(rx) > DEADZONE:
                        if ry < -DEADZONE: # Forward diagonally
                            vd = 2.0
                        elif ry > DEADZONE: # Reverse diagonally
                            vd = -2.0
                        wd = rx * MAX_WD
                    # Left Joystick: Rotate on spot (only left/right)
                    elif abs(lx) > DEADZONE and abs(ly) <= DEADZONE:
                        vd = 0.0
                        wd = lx * MAX_WD
                    # Left Joystick: Forward/Back (only up/down)
                    elif abs(ly) > DEADZONE and abs(lx) <= DEADZONE:
                        if ly < -DEADZONE: # Forward
                            vd = 2.0
                        elif ly > DEADZONE: # Reverse
                            vd = -2.0
                        wd = 0.0

                # Format Command
                cmd = f"V,{vd:.2f},{wd:.2f}\n"

            # Send Command to Arduino
            if ser:
                ser.write(cmd.encode('utf-8'))
            print(f"Sent: {cmd.strip()}")

            # 20 Hz loop
            clock.tick(20)

    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            try:
                ser.write(b"S\n")  # Stop rover before quitting
                ser.close()
            except:
                pass
        print("\nExiting...")
        pygame.quit()

if __name__ == "__main__":
    main()
