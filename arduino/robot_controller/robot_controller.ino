#include "movement.h"
#include "sensors.h"

void setup() {
  Serial.begin(115200);
  initMovement();
  initSensors();
}

void loop() {
  // Main control loop
  updateSensors();
  
  // Read from serial if ROS2 sends commands
  if (Serial.available()) {
    // Process commands
  }
}
