#ifndef MOVEMENT_DEBUG_H
#define MOVEMENT_DEBUG_H

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

const byte SIGNAL_A = 2;   // Left  encoder pulse A (Interrupt)
const byte SIGNAL_B = 4;   // Right encoder pulse A (Interrupt)
const byte SIGNAL_C = 3;   // Left  encoder pulse B (Interrupt for 4X)
const byte SIGNAL_D = 5;   // Right encoder pulse B (Interrupt for 4X)

// ═══════════════════════════════════════════════════════════════════════════════
//  ROBOT PHYSICAL CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

// [DEBUG] Originally 300 TPR for 1X decoding. Now using 4X decoding, yielding 1200 ticks per revolution. 
// If your specification states "3000 TPR" for the motor output shaft in 4X, leave this at 3000.
// // const int TPR = 3000;
const int    TPR  = 12000;    // Ticks per wheel revolution (Assuming full 4X resolution)
const double RHO  = 0.0625;  // Wheel radius [m]
const double ELL  = 0.2775;  // Track width [m]

// ═══════════════════════════════════════════════════════════════════════════════
//  CONTROLLER PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════════

const int T = 50; // Control-loop period [ms]
const int DEADZONE = 40;
const double MAX_WHEEL_SPEED = 0.5;  // [m/s] 
const double k_P = 60.0;   
const double k_I = 25.0;   
const double INT_CLAMP = 1.5;

// ═══════════════════════════════════════════════════════════════════════════════
//  SHARED STATE
// ═══════════════════════════════════════════════════════════════════════════════

extern float vd; 
extern float wd; 

extern double speed_L;
extern double speed_R;

// [DEBUG] Instead of resetting ticks, we accumulate absolute distance natively
// // extern volatile long encoder_ticksL;
// // extern volatile long encoder_ticksR;
extern volatile long absolute_ticksL;
extern volatile long absolute_ticksR;

// ═══════════════════════════════════════════════════════════════════════════════
//  PUBLIC API
// ═══════════════════════════════════════════════════════════════════════════════

void movement_setup();
void PID_update();
void reset_integrators();

// These are purely for backward compatibility if you still use them in print
double vehicle_speed(); 
double vehicle_omega(); 

// [DEBUG] Added new interrupt handlers for 4X encoding
void ISR_encoderL_A();
void ISR_encoderL_B();
void ISR_encoderR_A();
void ISR_encoderR_B();

#endif  // MOVEMENT_DEBUG_H
