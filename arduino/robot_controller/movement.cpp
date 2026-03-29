/*
#include "movement.h"
#include <Arduino.h>

float vd = 0.0;
float wd = 0.0;

volatile long encoder_ticksL = 0;
volatile long encoder_ticksR = 0;

// Shared state (updated by PID, read by sensors)
double omega_L = 0.0;
double omega_R = 0.0;
double speed_L = 0.0;
double speed_R = 0.0;

static double errorAL  = 0.0;
static double errorAR  = 0.0;

static short  inLeft   = 0;
static short  inRight  = 0;

static long   t_last   = 0;

void movement_setup()
{
    // Motor driver pins
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Encoder input pins
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);
    pinMode(SIGNAL_C, INPUT);
    pinMode(SIGNAL_D, INPUT);

    // Attach interrupts for encoder counting
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_encoderL, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_encoderR, RISING);

    t_last = millis();
}


void reset_integrators()
{
    errorAL = 0.0;
    errorAR = 0.0;
}


void ISR_encoderL()
{
    if (digitalRead(SIGNAL_C) == HIGH) {
        encoder_ticksL++;
    } else {
        encoder_ticksL--;
    }
}


void ISR_encoderR()
{
    if (digitalRead(SIGNAL_D) == HIGH) {
        encoder_ticksR++;
    } else {
        encoder_ticksR--;
    }
}


double compute_vehicle_speed(double speed_L, double speed_R)
{
    return 0.5 * (speed_L + speed_R);
}

double compute_vehicle_rate(double speed_L, double speed_R)
{
    return (1.0 / ELL) * (speed_R - speed_L);
}


double compute_leftd()
{
    return vd - (wd * ELL) / 2.0;
}


double compute_rightd()
{
    return vd + (wd * ELL) / 2.0;
}


short PI_controller(double e_now, double e_int, double kP, double kI)
{
    long u = (long)(kP * e_now + kI * e_int);

    if      (u >  255) u =  255;
    else if (u < -255) u = -255;

    return (short)u;
}


void motor_write(short left, short right)
{
    if (abs(left) < DEADZONE) {
        digitalWrite(I1, HIGH);
        digitalWrite(I2, HIGH);
        analogWrite(EA, 255);
    } else if (left > 0) {
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
        analogWrite(EA, left);
    } else {
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
        analogWrite(EA, -left);
    }

    if (abs(right) < DEADZONE) {
        digitalWrite(I3, HIGH);
        digitalWrite(I4, HIGH);
        analogWrite(EB, 255);
    } else if (right > 0) {
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
        analogWrite(EB, right);
    } else {
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
        analogWrite(EB, -right);
    }
}


void PID()
{
    long t_now = millis();

    if (t_now - t_last >= T)
    {
        double dt = (double)(t_now - t_last);   // [ms]

        // Snapshot and reset encoder counters atomically
        noInterrupts();
        long ticksL = encoder_ticksL;
        long ticksR = encoder_ticksR;
        encoder_ticksL = 0;
        encoder_ticksR = 0;
        interrupts();

        // Angular velocities [rad/s]
        omega_L = 2.0 * PI * ((double)ticksL / (double)TPR) * 1000.0 / dt;
        omega_R = 2.0 * PI * ((double)ticksR / (double)TPR) * 1000.0 / dt;

        // Linear wheel speeds [m/s]
        speed_L = omega_L * RHO;
        speed_R = omega_R * RHO;

        t_last = t_now;

        // Desired wheel speeds
        double leftVd  = compute_leftd();
        double rightVd = compute_rightd();

        // Speed errors
        double error_left  = leftVd  - speed_L;
        double error_right = rightVd - speed_R;

        // PI control outputs
        inLeft  = PI_controller(error_left,  errorAL, k_P, k_I);
        inRight = PI_controller(error_right, errorAR, k_P, k_I);

        // Accumulate integral (convert dt from ms to s)
        errorAL += error_left  * (dt / 1000.0);
        errorAR += error_right * (dt / 1000.0);

        // Clamp integral to prevent windup
        const double INT_MAX_VAL = 2.5;
        errorAL = constrain(errorAL, -INT_MAX_VAL, INT_MAX_VAL);
        errorAR = constrain(errorAR, -INT_MAX_VAL, INT_MAX_VAL);
    }

    motor_write(inLeft, inRight);
}
*/
#include "movement.h"
#include <Arduino.h>

// ── Setpoints (written by serial parser) ──────────────────────────────────────
float vd = 0.0;
float wd = 0.0;

// ── Encoder counters (written by ISRs, cleared each PID cycle) ───────────────
volatile long encoder_ticksL = 0;
volatile long encoder_ticksR = 0;

// ── Measured wheel speeds (exported for odometry) ─────────────────────────────
double speed_L = 0.0;
double speed_R = 0.0;

// ── PI integrator accumulators ────────────────────────────────────────────────
static double iL = 0.0;
static double iR = 0.0;

// ── Last computed motor commands (re-applied between PID ticks) ──────────────
static short pwmL = 0;
static short pwmR = 0;

static long t_last = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void movement_setup()
{
    // Motor driver
    pinMode(EA, OUTPUT); pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT); pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT); pinMode(I4, OUTPUT);

    // Encoder
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);
    pinMode(SIGNAL_C, INPUT);
    pinMode(SIGNAL_D, INPUT);

    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_encoderL, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_encoderR, RISING);

    t_last = millis();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ISRs
// ═══════════════════════════════════════════════════════════════════════════════

void ISR_encoderL()
{
    encoder_ticksL += (digitalRead(SIGNAL_C) == HIGH) ? 1 : -1;
}

void ISR_encoderR()
{
    encoder_ticksR += (digitalRead(SIGNAL_D) == HIGH) ? -1 : 1;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ODOMETRY HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

double vehicle_speed() { return 0.5 * (speed_L + speed_R); }
double vehicle_omega() { return (speed_R - speed_L) / ELL; }

void reset_integrators()
{
    iL = 0.0;
    iR = 0.0;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTOR DRIVER
// ═══════════════════════════════════════════════════════════════════════════════

static void motor_write(short left, short right)
{
    // ── Left ────────────────────────────────────────────────────────────────
    if (abs(left) < DEADZONE) {
        // Active brake (both direction pins HIGH)
        digitalWrite(I1, HIGH);
        digitalWrite(I2, HIGH);
        analogWrite(EA, 255);
    } else if (left > 0) {
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
        analogWrite(EA, (byte)left);
    } else {
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
        analogWrite(EA, (byte)(-left));
    }

    // ── Right ───────────────────────────────────────────────────────────────
    if (abs(right) < DEADZONE) {
        digitalWrite(I3, HIGH);
        digitalWrite(I4, HIGH);
        analogWrite(EB, 255);
    } else if (right > 0) {
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
        analogWrite(EB, (byte)right);
    } else {
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
        analogWrite(EB, (byte)(-right));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FEEDFORWARD
//
//  Maps a desired wheel speed [m/s] directly to a PWM value, bypassing the
//  integrator.  This means the PI only corrects a small residual error rather
//  than carrying the entire steady-state load — which is what made the old
//  controller sluggish and prone to windup.
// ═══════════════════════════════════════════════════════════════════════════════

static short feedforward(double desired)
{
    if (fabs(desired) < 1e-4) return 0;

    // Linear interpolation between deadzone and 255
    double mag = (fabs(desired) / MAX_WHEEL_SPEED) * (255.0 - DEADZONE) + DEADZONE;
    mag = constrain(mag, (double)DEADZONE, 255.0);

    return (desired > 0) ? (short)mag : -(short)mag;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PID UPDATE  —  call every main-loop iteration
// ═══════════════════════════════════════════════════════════════════════════════

void PID_update()
{
    long t_now = millis();

    // Between PID ticks, keep re-applying the last command so the motors
    // don't coast or stall during the gap.
    if (t_now - t_last < (long)T) {
        motor_write(pwmL, pwmR);
        return;
    }

    double dt = (double)(t_now - t_last) / 1000.0;  // [s]
    t_last = t_now;

    // ── 1. Snapshot and reset encoder counters (atomic) ───────────────────────
    noInterrupts();
    long ticksL = encoder_ticksL;  encoder_ticksL = 0;
    long ticksR = encoder_ticksR;  encoder_ticksR = 0;
    interrupts();

    // ── 2. Compute measured wheel speeds [m/s] ────────────────────────────────
    double omegaL = 2.0 * PI * (double)ticksL / ((double)TPR * dt);  // [rad/s]
    double omegaR = 2.0 * PI * (double)ticksR / ((double)TPR * dt);
    speed_L = omegaL * RHO;
    speed_R = omegaR * RHO;

    // ── 3. Differential-drive inverse kinematics ──────────────────────────────
    //  vdL and vdR are the desired linear speeds of each wheel.
    //  They correctly couple vd and wd — turning at wd slows one wheel and
    //  speeds up the other by exactly (wd * ELL / 2).
    double vdL = vd - (wd * ELL) * 0.5;
    double vdR = vd + (wd * ELL) * 0.5;

    // ── 4. Integrator direction-reversal reset ────────────────────────────────
    //  If the integrator has wound up in the opposite direction to the current
    //  setpoint, wipe it immediately.  This is what causes the turning-direction
    //  delay in the original code — the integrator had to unwind from +INT_CLAMP
    //  all the way through zero before the motor reversed.
    if (vdL * iL < -1e-6) iL = 0.0;
    if (vdR * iR < -1e-6) iR = 0.0;

    // ── 5. Compute errors ─────────────────────────────────────────────────────
    double eL = vdL - speed_L;
    double eR = vdR - speed_R;

    // ── 6. Feedforward + PI feedback ──────────────────────────────────────────
    double uL = (double)feedforward(vdL) + k_P * eL + k_I * iL;
    double uR = (double)feedforward(vdR) + k_P * eR + k_I * iR;

    // ── 7. Clamp outputs ──────────────────────────────────────────────────────
    uL = constrain(uL, -255.0, 255.0);
    uR = constrain(uR, -255.0, 255.0);

    // ── 8. Conditional integration (anti-windup) ──────────────────────────────
    //  Only accumulate when not saturated, or when the error is pulling the
    //  output back away from the rail (i.e. error opposes saturation).
    bool satL = (fabs(uL) >= 254.9);
    bool satR = (fabs(uR) >= 254.9);

    if (!satL || (eL * uL < 0)) iL += eL * dt;
    if (!satR || (eR * uR < 0)) iR += eR * dt;

    iL = constrain(iL, -INT_CLAMP, INT_CLAMP);
    iR = constrain(iR, -INT_CLAMP, INT_CLAMP);

    // ── 9. Store and apply ────────────────────────────────────────────────────
    pwmL = (short)uL;
    pwmR = (short)uR;

    motor_write(pwmL, pwmR);
}
