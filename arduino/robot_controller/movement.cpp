#include "movement.h"
#include <Arduino.h>

// Wheel PWM pins
const int EA = 9;
const int EB = 11;

// Wheel direction digital pins
const int I1 = 8;
const int I2 = 10;
const int I3 = 13;
const int I4 = 12;

void initMovement() {
  // Set the pin modes for the motor driver
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
}

void move_motors(int leftPWM, int rightPWM) {
  if (leftPWM > 0) {
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
  } else if (leftPWM < 0) {
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
  } else {
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
  }

  if (rightPWM > 0) {
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
  } else if (rightPWM < 0) {
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  } else {
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);
  }

  // Write to motor
  analogWrite(EA, abs(leftPWM));
  analogWrite(EB, abs(rightPWM));
}

void moveForward(int speed) { move_motors(speed, speed); }

void stopMovement() { move_motors(0, 0); }

void turnLeft(int speed) { move_motors(-speed, speed); }

void turnRight(int speed) { move_motors(speed, -speed); }
