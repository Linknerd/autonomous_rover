#include "movement.h"
#include "sensors.h"

void setup() {
  Serial.begin(9600);
  movement_setup();
  initSensors();
  Serial.println("I,Program initialized.");
}

void loop() {
  updateSensors();
  PID();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Protocol: V,vd,wd\n
    if (cmd.startsWith("V,")) {
      int firstComma  = cmd.indexOf(',');
      int secondComma = cmd.indexOf(',', firstComma + 1);

      if (firstComma != -1 && secondComma != -1) {
        vd = cmd.substring(firstComma + 1, secondComma).toFloat();
        wd = cmd.substring(secondComma + 1).toFloat();
      }
    } else if (cmd == "S") {
      vd = 0.0;
      wd = 0.0;
    }
  }
}
