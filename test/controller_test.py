import pygame
import time
import sys

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick/controller found.")
        sys.exit()

    joystick = pygame.joystick.Joystick(0)
    print(f"Connected to: {joystick.get_name()}")
    
    num_axes = joystick.get_numaxes()
    num_buttons = joystick.get_numbuttons()
    num_hats = joystick.get_numhats()

    print(f"Axes: {num_axes}, Buttons: {num_buttons}, Hats: {num_hats}")
    
    # Read the controller state for 3 seconds to print out which buttons and axes are active
    print("Testing controller mapping for 10 seconds. Move the sticks and triggers!")
    start_time = time.time()
    
    while time.time() - start_time < 10:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
                
            if event.type == pygame.JOYAXISMOTION:
                if abs(event.value) > 0.2: # filter deadzone
                    print(f"Axis {event.axis} moved to {event.value:.2f}")
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")
            if event.type == pygame.JOYHATMOTION:
                print(f"Hat {event.hat} moved to {event.value}")
        time.sleep(0.1)
        
    print("Test complete.")

if __name__ == "__main__":
    main()
