#include "movement.h"
#include "sensors.h"

byte u = 0; // default motor command

void setup() {
  Serial.begin(9600); // Changed to 9600 to match user code
  initMovement();
  initSensors();
  Serial.print("Program initialized.\n");
}

void loop() {
  // 1. Calculate speeds and handle encooder timings
  updateSensors();

  // 2. Read from serial if ROS2 sends commands
  if (Serial.available()) {
    // Process incoming ROS2 commands (e.g., speed, turn rate)
    // For now we will just read them out to clear the buffer
    String cmd = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(cmd);
  }

  // 3. For testing right now, just drive motors at a constant PWM like in the
  // original code u = 128; moveForward(u);
}
