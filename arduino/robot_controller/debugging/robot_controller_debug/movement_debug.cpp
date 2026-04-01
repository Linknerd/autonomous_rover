#include "movement_debug.h"
#include <Arduino.h>

// ── Setpoints (written by serial parser) ──────────────────────────────────────
float vd = 0.0;
float wd = 0.0;

// [DEBUG] Old implementation cleared the ticks every loop.
// // volatile long encoder_ticksL = 0;
// // volatile long encoder_ticksR = 0;
volatile long absolute_ticksL = 0;
volatile long absolute_ticksR = 0;

// Internal variables for measuring delta ticks for the local PID loop
static long previous_ticksL = 0;
static long previous_ticksR = 0;

// ── Measured wheel speeds (exported for odometry) ─────────────────────────────
double speed_L = 0.0;
double speed_R = 0.0;

// ── PI integrator accumulators ────────────────────────────────────────────────
static double iL = 0.0;
static double iR = 0.0;

static short pwmL = 0;
static short pwmR = 0;

static long t_last = 0;

void movement_setup()
{
    pinMode(EA, OUTPUT); pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT); pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT); pinMode(I4, OUTPUT);

    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);
    pinMode(SIGNAL_C, INPUT);
    pinMode(SIGNAL_D, INPUT);

    // [DEBUG] Old code: 1X Decoding on RISING edges
    // // attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_encoderL, RISING);
    // // attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_encoderR, RISING);
    
    // [DEBUG] New code: 4X Quadrature Decoding on CHANGE for all A and B pins
    // This utilizes the ATmega4809's capability to interrupt on any digital pin natively.
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_encoderL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_C), ISR_encoderL_B, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_encoderR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_D), ISR_encoderR_B, CHANGE);

    t_last = millis();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ISRs - 4X Quadrature Encoding
// ═══════════════════════════════════════════════════════════════════════════════

// [DEBUG] Replace 1X encoder logic with exact 4X phase reading
/*
void ISR_encoderL() {
    encoder_ticksL += (digitalRead(SIGNAL_C) == HIGH) ? 1 : -1;
}
void ISR_encoderR() {
    encoder_ticksR += (digitalRead(SIGNAL_D) == HIGH) ? -1 : 1;
}
*/

// --- Left Encoder (4X) ---
void ISR_encoderL_A() {
    bool A = digitalRead(SIGNAL_A);
    bool B = digitalRead(SIGNAL_C);
    if (A == B) absolute_ticksL++; else absolute_ticksL--;
}
void ISR_encoderL_B() {
    bool A = digitalRead(SIGNAL_A);
    bool B = digitalRead(SIGNAL_C);
    if (A != B) absolute_ticksL++; else absolute_ticksL--;
}

// --- Right Encoder (4X) ---
// Note: Intentionally inverted counting direction to match previous logic natively
void ISR_encoderR_A() {
    bool A = digitalRead(SIGNAL_B);
    bool B = digitalRead(SIGNAL_D);
    if (A == B) absolute_ticksR--; else absolute_ticksR++;
}
void ISR_encoderR_B() {
    bool A = digitalRead(SIGNAL_B);
    bool B = digitalRead(SIGNAL_D);
    if (A != B) absolute_ticksR--; else absolute_ticksR++;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ODOMETRY CONTROLLER
// ═══════════════════════════════════════════════════════════════════════════════

double vehicle_speed() { return 0.5 * (speed_L + speed_R); }
double vehicle_omega() { return (speed_R - speed_L) / ELL; }

void reset_integrators() { iL = 0.0; iR = 0.0; }

static void motor_write(short left, short right)
{
    if (abs(left) < DEADZONE) {
        digitalWrite(I1, HIGH); digitalWrite(I2, HIGH); analogWrite(EA, 255);
    } else if (left > 0) {
        digitalWrite(I1, LOW);  digitalWrite(I2, HIGH); analogWrite(EA, (byte)left);
    } else {
        digitalWrite(I1, HIGH); digitalWrite(I2, LOW);  analogWrite(EA, (byte)(-left));
    }

    if (abs(right) < DEADZONE) {
        digitalWrite(I3, HIGH); digitalWrite(I4, HIGH); analogWrite(EB, 255);
    } else if (right > 0) {
        digitalWrite(I3, LOW);  digitalWrite(I4, HIGH); analogWrite(EB, (byte)right);
    } else {
        digitalWrite(I3, HIGH); digitalWrite(I4, LOW);  analogWrite(EB, (byte)(-right));
    }
}

static short feedforward(double desired)
{
    if (fabs(desired) < 1e-4) return 0;
    double mag = (fabs(desired) / MAX_WHEEL_SPEED) * (255.0 - DEADZONE) + DEADZONE;
    mag = constrain(mag, (double)DEADZONE, 255.0);
    return (desired > 0) ? (short)mag : -(short)mag;
}

void PID_update()
{
    long t_now = millis();

    if (t_now - t_last < (long)T) {
        motor_write(pwmL, pwmR);
        return;
    }

    double dt = (double)(t_now - t_last) / 1000.0;
    t_last = t_now;

    // [DEBUG] Old logic cleared the interrupts. We now sample and compute difference
    // // noInterrupts();
    // // long ticksL = encoder_ticksL;  encoder_ticksL = 0;
    // // long ticksR = encoder_ticksR;  encoder_ticksR = 0;
    // // interrupts();
    
    noInterrupts();
    long currentL = absolute_ticksL;
    long currentR = absolute_ticksR;
    interrupts();

    long ticksL = currentL - previous_ticksL;
    long ticksR = currentR - previous_ticksR;
    
    previous_ticksL = currentL;
    previous_ticksR = currentR;

    double omegaL = 2.0 * PI * (double)ticksL / ((double)TPR * dt);
    double omegaR = 2.0 * PI * (double)ticksR / ((double)TPR * dt);
    speed_L = omegaL * RHO;
    speed_R = omegaR * RHO;

    double vdL = vd - (wd * ELL) * 0.5;
    double vdR = vd + (wd * ELL) * 0.5;

    if (vdL * iL < -1e-6) iL = 0.0;
    if (vdR * iR < -1e-6) iR = 0.0;

    double eL = vdL - speed_L;
    double eR = vdR - speed_R;

    double uL = (double)feedforward(vdL) + k_P * eL + k_I * iL;
    double uR = (double)feedforward(vdR) + k_P * eR + k_I * iR;

    uL = constrain(uL, -255.0, 255.0);
    uR = constrain(uR, -255.0, 255.0);

    bool satL = (fabs(uL) >= 254.9);
    bool satR = (fabs(uR) >= 254.9);

    if (!satL || (eL * uL < 0)) iL += eL * dt;
    if (!satR || (eR * uR < 0)) iR += eR * dt;

    iL = constrain(iL, -INT_CLAMP, INT_CLAMP);
    iR = constrain(iR, -INT_CLAMP, INT_CLAMP);

    pwmL = (short)uL;
    pwmR = (short)uR;

    motor_write(pwmL, pwmR);
}
