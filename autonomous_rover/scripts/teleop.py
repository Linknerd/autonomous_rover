import pygame
import serial
import time
import sys

# Windows usually uses COM ports (e.g., 'COM3'). Update this to your Arduino's port.
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# Rover maximum velocities
MAX_VD = 1.0  # Max linear velocity (example: 1.0 m/s)
MAX_WD = 1.0  # Max angular velocity (example: 1.0 rad/s)

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

    print("\n--- Controls ---")
    print("Right Trigger : Movement (Forward)")
    print("Left Joystick : Turn (Left/Right)")
    print("Left Trigger  : Force Break (Stop)\n")

    clock = pygame.time.Clock()

    left_stick_x = 0.0
    left_trigger_pressed = False
    right_trigger_pressed = False

    try:
        while True:
            # Handle Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                elif event.type == pygame.JOYAXISMOTION:
                    if event.axis == 0:
                        left_stick_x = event.value
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 6:
                        left_trigger_pressed = True
                    elif event.button == 7:
                        right_trigger_pressed = True
                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == 6:
                        left_trigger_pressed = False
                    elif event.button == 7:
                        right_trigger_pressed = False

            # Define deadzones to prevent drift
            DEADZONE = 0.1
            clean_lx = left_stick_x if abs(left_stick_x) >= DEADZONE else 0.0

            # 1. Force Break (Left Trigger)
            if left_trigger_pressed:
                cmd = "S\n"
            else:
                # 2. Movement and Turning
                # Right trigger controls forward linear velocity 'vd'
                vd = MAX_VD if right_trigger_pressed else 0.0
                
                # Left joystick controls angular velocity 'wd'
                # Assuming right = positive wd, left = negative wd
                wd = clean_lx * MAX_WD
                
                # Format: "V,vd,wd"
                cmd = f"V,{vd:.2f},{wd:.2f}\n"

            # Send Command to Arduino
            if ser:
                ser.write(cmd.encode('utf-8'))
            print(f"Sent: {cmd.strip()}")

            # 20 Hz loop
            clock.tick(20)

    except KeyboardInterrupt:
        if ser:
            ser.write(b"S\n")  # Stop rover before quitting
            ser.close()
        print("\nExiting...")
        pygame.quit()

if __name__ == "__main__":
    main()
