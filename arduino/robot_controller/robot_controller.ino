#include "movement.h"
#include "sensors.h"

static String serialBuffer = "";

void setup() {
  Serial.begin(115200);
  delay(2000);
  while(Serial.available()) Serial.read();
  movement_setup();
  initSensors();
  Serial.println("I,Program initialized.");
}

void processCommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("V,")) {
    int firstComma  = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);

    if (firstComma != -1 && secondComma != -1) {
      vd = cmd.substring(firstComma + 1, secondComma).toFloat();
      wd = cmd.substring(secondComma + 1).toFloat();
      Serial.print("ACK_V, vd:");
      Serial.print(vd);
      Serial.print(" wd:");
      Serial.println(wd);
    }
  } else if (cmd == "S") {
    vd = 0.0;
    wd = 0.0;
    Serial.println("ACK_STOP");
  }
}

void loop() {
  updateSensors();
  PID();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(serialBuffer);
      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
    }
  }
}
