#include "movement.h"
#include "sensors.h"

byte u = 0; // default motor command

void setup() {
  Serial.begin(9600); // Changed to 9600 to match user code
  initMovement();
  initSensors();
  Serial.println("I,Program initialized.");
}

void loop() {
  // 1. Calculate speeds and handle encoder timings, outputs via CSV
  updateSensors();

  // 2. Read from serial if ROS2 sends commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); // Read line until newline

    // Protocol Ex: M,left_pwm,right_pwm\n
    if (cmd.startsWith("M,")) {
      int firstComma = cmd.indexOf(',');
      int secondComma = cmd.indexOf(',', firstComma + 1);

      if (firstComma != -1 && secondComma != -1) {
        String leftPWM_str = cmd.substring(firstComma + 1, secondComma);
        String rightPWM_str = cmd.substring(secondComma + 1);

        int leftPWM = leftPWM_str.toInt();
        int rightPWM = rightPWM_str.toInt();

        move_motors(leftPWM, rightPWM);
      }
    } else if (cmd.startsWith("S\n")) {
      // Maybe Stop command
      stopMovement();
    }
  }

  // Delay tightly for stability like original IMU code indicated
  delay(10);
}
