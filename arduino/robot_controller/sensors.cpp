#include "sensors.h"
#include <Arduino.h>
#include <Arduino_LSM6DS3.h> // Keep if IMU is used

// Left wheel encoder digital pins
const byte SIGNAL_A = 2;
const byte SIGNAL_B = 3;
const byte SIGNAL_C = 4;
const byte SIGNAL_D = 5;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;
const double ELL = 0.2775;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksL = 0;
volatile long encoder_ticksR = 0;

// Variables to store estimated rates
double omega_L = 0.0;
double omega_R = 0.0;
double speed_L = 0.0;
double speed_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;
long t_now = 0;
long t_last = 0;

// Interrupt Service Routines
void decodeLEncoderTicks() {
  if (digitalRead(SIGNAL_B) == LOW) {
    encoder_ticksL++;
  } else {
    encoder_ticksL--;
  }
}

void decodeREncoderTicks() {
  if (digitalRead(SIGNAL_D) == LOW) {
    encoder_ticksR--;
  } else {
    encoder_ticksR++;
  }
}

void initSensors() {
  // Set the pin modes for the encoders
  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_C, INPUT);
  pinMode(SIGNAL_D, INPUT);
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeLEncoderTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeREncoderTicks, RISING);
}

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double s_L, double s_R) {
  return 0.5 * (s_L + s_R);
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double s_L, double s_R) {
  return 1.0 / ELL * (s_R - s_L);
}

void updateSensors() {
  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)encoder_ticksL / (double)TPR) * 1000.0 /
              (double)(t_now - t_last);
    omega_R = 2.0 * PI * ((double)encoder_ticksR / (double)TPR) * 1000.0 /
              (double)(t_now - t_last);
    speed_L = omega_L * RHO;
    speed_R = omega_R * RHO;

    // Record the current time [ms]
    t_last = t_now;

    Serial.print("Average Speed: ");
    Serial.print(compute_vehicle_speed(speed_L, speed_R));
    Serial.print("\t");
    Serial.print("Average Rotation: ");
    Serial.print(compute_vehicle_rate(speed_L, speed_R));
    Serial.print("\n");

    // Reset the encoder ticks counters
    encoder_ticksL = 0;
    encoder_ticksR = 0;
  }
}
