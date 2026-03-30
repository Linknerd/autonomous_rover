/*
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

const int T = 100;

const int DEADZONE = 40;

const short k_P = 200;
const short k_I = 100;

extern float vd;
extern float wd;

extern volatile long encoder_ticksL;
extern volatile long encoder_ticksR;

void movement_setup();
void reset_integrators();

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
*/
#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  PIN ASSIGNMENTS
// ═══════════════════════════════════════════════════════════════════════════════

const int  EA       = 9;   // Left  motor PWM
const int  EB       = 11;  // Right motor PWM
const int  I1       = 8;   // Left  motor direction A
const int  I2       = 10;  // Left  motor direction B
const int  I3       = 13;  // Right motor direction A
const int  I4       = 12;  // Right motor direction B

const byte SIGNAL_A = 2;   // Left  encoder pulse  — must be an interrupt pin
const byte SIGNAL_B = 3;   // Right encoder pulse  — must be an interrupt pin
const byte SIGNAL_C = 4;   // Left  encoder direction
const byte SIGNAL_D = 5;   // Right encoder direction

// ═══════════════════════════════════════════════════════════════════════════════
//  ROBOT PHYSICAL CONSTANTS  ← tune these for your hardware
// ═══════════════════════════════════════════════════════════════════════════════

const int    TPR  = 300;    // Encoder ticks per wheel revolution
const double RHO  = 0.0625;  // Wheel radius [m]
const double ELL  = 0.2775;  // Track width (centre-to-centre of wheels) [m]

// ═══════════════════════════════════════════════════════════════════════════════
//  CONTROLLER PARAMETERS  ← tune these after physical constants are correct
// ═══════════════════════════════════════════════════════════════════════════════

// Control-loop period [ms].  Faster = more responsive, but needs a stable
// encoder signal.  50 ms (20 Hz) is a good starting point.
const int T = 50;

// PWM dead-zone: values below this produce no motion on most motor drivers.
// Raise this if the motors twitch without moving; lower if motion is jerky.
const int DEADZONE = 40;

// Feedforward gain:  set this to your wheel's free-running top speed [m/s]
// at PWM = 255 (i.e. 100 % duty cycle, no load).
// To measure: set vd = 0.2, wd = 0, record what speed the encoders report
// when steady.  Then scale: MAX_WHEEL_SPEED = measured_speed / (0.2 / 0.5).
// Getting this within ±20 % is good enough; the PI loop corrects the rest.
const double MAX_WHEEL_SPEED = 0.5;  // [m/s]  ← TUNE THIS FIRST

// PI feedback gains — these only need to trim the residual error left after
// feedforward, so values much smaller than a pure-PI system are appropriate.
const double k_P = 60.0;   // Proportional gain [PWM / (m/s)]
const double k_I = 25.0;   // Integral gain     [PWM / (m/s · s)]

// Maximum absolute integral accumulator.  Keeps windup bounded.
const double INT_CLAMP = 1.5;  // [m/s · s]

// ═══════════════════════════════════════════════════════════════════════════════
//  SHARED STATE
// ═══════════════════════════════════════════════════════════════════════════════

// Velocity setpoints — written by the serial command parser in the .ino
extern float vd;  // Desired linear  velocity [m/s]
extern float wd;  // Desired angular velocity [rad/s]

// Measured wheel speeds — written by PID_update(), read by odometry output
extern double speed_L;  // [m/s]
extern double speed_R;  // [m/s]

// Raw encoder counts — written by ISRs
extern volatile long encoder_ticksL;
extern volatile long encoder_ticksR;

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC API
// ═══════════════════════════════════════════════════════════════════════════════

void movement_setup();

// Call once per main-loop iteration.  Runs the PID at period T; applies
// the last motor command on every other call so the motors don't coast.
void PID_update();

// Clears PI integrators.  Call when setpoints change significantly or on STOP.
void reset_integrators();

// Derived vehicle velocities from the last wheel-speed measurement.
double vehicle_speed();  // linear  [m/s]  = (speed_L + speed_R) / 2
double vehicle_omega();  // angular [rad/s] = (speed_R - speed_L) / ELL

// Encoder ISRs — attached inside movement_setup()
void ISR_encoderL();
void ISR_encoderR();

#endif  // MOVEMENT_H
