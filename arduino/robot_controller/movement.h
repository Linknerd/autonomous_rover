#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>


// Wheel PWM pins
const int EA = 9;
const int EB = 11;

// Wheel direction digital pins
const int I1 = 8;
const int I2 = 10;
const int I3 = 13;
const int I4 = 12;

const byte SIGNAL_A = 2;  // Left wheel  - interrupt pin
const byte SIGNAL_B = 3;  // Right wheel - interrupt pin
const byte SIGNAL_C = 4;  // Left wheel  - direction pin
const byte SIGNAL_D = 5;  // Right wheel - direction pin


extern double omega_L;
extern double omega_R;
extern double speed_L;
extern double speed_R;

const int TPR = 3000;

const double RHO = 0.0625;

const double ELL = 0.2775;

const int T = 50;

const int DEADZONE = 40;

const short k_P = 200;
const short k_I = 0.5 * k_P;

extern float vd;   
extern float wd;   

extern volatile long encoder_ticksL;
extern volatile long encoder_ticksR;

void movement_setup();

double compute_vehicle_speed(double speed_L, double speed_R);
double compute_vehicle_rate(double speed_L, double speed_R);
double compute_leftd();
double compute_rightd();

short PI_controller(double e_now, double e_int, double kP, double kI);
void  motor_write(short left, short right);
void  PID();


void ISR_encoderL();
void ISR_encoderR();

#endif 
