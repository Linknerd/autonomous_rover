#include "sensors.h"
#include <Adafruit_SCD30.h>
#include <Arduino.h>
#include <Arduino_LSM6DS3.h> // Keep if IMU is used


// --- ENCODER setup ---
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

// --- SCD30 setup ---
Adafruit_SCD30 scd;

// --- SHARP SENSOR setup ---
const byte SENSOR_PINS[] = {A0, A1, A3};
const int NUM_SENSORS = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);

const float AD_COEFF = 27.86;
const float AD_EXPONENT = -1.15;
const float ADC_TO_VOLTS = 5.0 / 1023.0;

float getDistance(byte pin) {
  int rawValue = analogRead(pin);
  float voltage = rawValue * ADC_TO_VOLTS;

  // Handle edge case: Avoid pow(0, negative) which results in inf/nan
  if (voltage <= 0.01)
    return 80.0;

  return AD_COEFF * pow(voltage, AD_EXPONENT);
}

// --- Interrupt Service Routines ---
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
  // 1. Set the pin modes for the encoders
  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_C, INPUT);
  pinMode(SIGNAL_D, INPUT);
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeLEncoderTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeREncoderTicks, RISING);

  // 2. Initialize SCD30
  if (!scd.begin()) {
    Serial.println("SCD30 Sensor not found :(");
    // We will not block here forever, just let ROS know via serial eventually
  } else {
    // Set the measurement interval [2-1800 s]
    scd.setMeasurementInterval(2);
  }

  // (Sharp sensors just need analogRead, no init requried)
}

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double s_L, double s_R) {
  return 0.5 * (s_L + s_R);
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double s_L, double s_R) {
  return 1.0 / ELL * (s_R - s_L);
}

void readSCD30() {
  // The SCD30 will report when it is ready.
  if (scd.dataReady()) {
    if (!scd.read()) {
      Serial.println("Error reading SCD30 sensor data");
      return;
    }

    Serial.print("SCD30 Temp: ");
    Serial.print(scd.temperature);
    Serial.print(" dC\t");
    Serial.print("RH: ");
    Serial.print(scd.relative_humidity);
    Serial.print(" %\t");
    Serial.print("CO2: ");
    Serial.print(scd.CO2, 3);
    Serial.println(" ppm");
  }
}

void readSharpSensors() {
  Serial.print("Sharp array: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    float distance = getDistance(SENSOR_PINS[i]);
    Serial.print("[S");
    Serial.print(i);
    Serial.print(":");
    Serial.print(distance);
    Serial.print("cm] ");
  }
  Serial.println();
}

void updateSensors() {
  // Get the elapsed time [ms]
  t_now = millis();

  // Encoders are read every T milliseconds
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

    Serial.print("Encoders -> Speed: ");
    Serial.print(compute_vehicle_speed(speed_L, speed_R));
    Serial.print("\t");
    Serial.print("Rotation: ");
    Serial.print(compute_vehicle_rate(speed_L, speed_R));
    Serial.print("\n");

    // Reset the encoder ticks counters
    encoder_ticksL = 0;
    encoder_ticksR = 0;

    // Also print Sharp sensors at this interval
    readSharpSensors();
  }

  // SCD30 checks if data is ready on its own polling cycle
  readSCD30();
}
