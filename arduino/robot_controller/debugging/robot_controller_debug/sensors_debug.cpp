#include "sensors_debug.h"
#include "movement_debug.h"
#include <Arduino.h>

static long t_last_sensor = 0;

void sensors_setup()
{
}

void sensors_update()
{
    long t_now = millis();

    if (t_now - t_last_sensor >= (long)T)
    {
        // [DEBUG] The old implementation transmitted instant velocity.
        // // Serial.print("O,");
        // // Serial.print(vehicle_speed(), 4);
        // // Serial.print(",");
        // // Serial.println(vehicle_omega(), 4);
        
        // [DEBUG] The new implementation transmits:
        // O,<timestamp_ms>,<absolute_left_ticks>,<absolute_right_ticks>
        // This makes the python integration completely immune to USB jitter.
        
        noInterrupts();
        long currentL = absolute_ticksL;
        long currentR = absolute_ticksR;
        interrupts();

        Serial.print("O,");
        Serial.print(t_now);
        Serial.print(",");
        Serial.print(currentL);
        Serial.print(",");
        Serial.println(currentR);

        t_last_sensor = t_now;
    }
}
