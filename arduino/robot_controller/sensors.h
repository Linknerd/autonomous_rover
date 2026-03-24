/*
#ifndef SENSORS_H
#define SENSORS_H

void initSensors();
void updateSensors();
void readSCD30();
void readSharpSensors();
void readIMU();

#endif
*/
#ifndef SENSORS_H
#define SENSORS_H

// Initialises any sensor hardware (currently a no-op; extend if you re-add
// the IMU or other peripherals later).
void sensors_setup();

// Call every main-loop iteration.  Outputs one O,v,w line over Serial every
// T milliseconds — the same rate as the PID loop.
void sensors_update();

#endif  // SENSORS_H
