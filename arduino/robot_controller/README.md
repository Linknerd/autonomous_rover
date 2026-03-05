# Robot Controller

This directory contains the low-level Arduino code for the robot.

## Hardware Connections

### Motor Driver (L298N or similar)
- `EA` (Left PWM) -> Pin 9
- `EB` (Right PWM) -> Pin 11
- `I1`, `I2` (Left Dir) -> Pins 8, 10
- `I3`, `I4` (Right Dir) -> Pins 13, 12

### Encoders
- `SIGNAL_A` (Left) -> Pin 2
- `SIGNAL_B` (Left) -> Pin 3
- `SIGNAL_C` (Right) -> Pin 4
- `SIGNAL_D` (Right) -> Pin 5

## Structure
- `robot_controller.ino`: Main control loop and Serial command processor.
- `movement.cpp` / `.h`: Handles motor PWM and direction control.
- `sensors.cpp` / `.h`: Handles encoder interrupts, odometry calculation, and IMU data.
