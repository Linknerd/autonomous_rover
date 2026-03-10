#include "sensors.h"
#include "movement.h"
#include <Adafruit_SCD30.h>
#include <Arduino.h>
#include <Arduino_LSM6DS3.h> // IMU library


long t_now = 0;
long t_last = 0;

// --- IMU LSM6DS3 setup ---
float omega_x, omega_y, omega_z;
float a_x, a_y, a_z;
float a_f, g_f;

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


void initSensors() {
  // 2. Initialize SCD30
  if (!scd.begin()) {
    Serial.println("E,SCD30 Sensor not found");
    // We will not block here forever, just let ROS know via serial eventually
  } else {
    // Set the measurement interval [2-1800 s]
    scd.setMeasurementInterval(2);
  }

  // 3. Initialize IMU
  if (!IMU.begin()) {
    Serial.println("E,Failed to initialize IMU");
  } else {
    a_f = IMU.accelerationSampleRate();
    g_f = IMU.gyroscopeSampleRate();
    // Removed IMU sample rate print statements
  }

  // (Sharp sensors just need analogRead, no init requried)
}

void readSCD30() {
  // The SCD30 will report when it is ready.
  if (scd.dataReady()) {
    if (!scd.read()) {
      Serial.println("E,Error reading SCD30");
      return;
    }

    // Format: C,temp,humidity,co2
    Serial.print("C,");
    Serial.print(scd.temperature);
    Serial.print(",");
    Serial.print(scd.relative_humidity);
    Serial.print(",");
    Serial.println(scd.CO2, 3);
  }
}

void readSharpSensors() {
  // Format: S,dist0,dist1,dist2
  Serial.print("S");
  for (int i = 0; i < NUM_SENSORS; i++) {
    float distance = getDistance(SENSOR_PINS[i]);
    Serial.print(",");
    Serial.print(distance);
  }
  Serial.println();
}

void readIMU() {
  // Read from the accelerometer and gyroscope
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(a_x, a_y, a_z);
    IMU.readGyroscope(omega_x, omega_y, omega_z);

    // Format: I,ax,ay,az,gx,gy,gz
    Serial.print("I,");
    Serial.print(a_x + 0.01);
    Serial.print(",");
    Serial.print(a_y + 0.02);
    Serial.print(",");
    Serial.print(a_z - 0.01);
    Serial.print(",");
    Serial.print(omega_x - 0.06);
    Serial.print(",");
    Serial.print(omega_y + 0.06);
    Serial.print(",");
    Serial.println(omega_z - 0.12);
  }
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

    // Format: O,speed,rotation
    Serial.print("O,");
    Serial.print(compute_vehicle_speed(speed_L, speed_R));
    Serial.print(",");
    Serial.println(compute_vehicle_rate(speed_L, speed_R));

    // Reset the encoder ticks counters
    encoder_ticksL = 0;
    encoder_ticksR = 0;

    // Also print Sharp sensors at this interval
    readSharpSensors();
  }

  // SCD30 checks if data is ready on its own polling cycle
  readSCD30();

  readIMU();
}
