#include "movement.h"
#include <Arduino.h>


float vd = 0.5;
float wd = 4.0;

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
    delay(50);
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

        // Debug: show raw ticks per interval (should be non-zero when wheels move)
        Serial.print("D,");
        Serial.print(ticksL);
        Serial.print(",");
        Serial.println(ticksR);

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

        // Accumulate integral (convert T from ms to s)
        errorAL += error_left  * (dt / 1000.0);
        errorAR += error_right * (dt / 1000.0);

        // PI control outputs
        inLeft  = PI_controller(error_left,  errorAL, k_P, k_I);
        inRight = PI_controller(error_right, errorAR, k_P, k_I);

        motor_write(inLeft, inRight);
    }
  }
