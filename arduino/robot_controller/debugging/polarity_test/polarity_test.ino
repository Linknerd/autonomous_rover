const int EA = 9;   // Left  motor PWM
const int I1 = 8;   // Left  motor direction A
const int I2 = 10;  // Left  motor direction B

const int EB = 11;  // Right motor PWM
const int I3 = 13;  // Right motor direction A
const int I4 = 12;  // Right motor direction B

const byte SIGNAL_A = 2;   // Left encoder pulse
const byte SIGNAL_B = 4;   // Right encoder pulse
const byte SIGNAL_C = 3;   // Left encoder direction
const byte SIGNAL_D = 5;   // Right encoder direction

volatile long encoder_ticksL = 0;
volatile long encoder_ticksR = 0;

void ISR_encoderL() {
    encoder_ticksL += (digitalRead(SIGNAL_C) == HIGH) ? 1 : -1;
}

void ISR_encoderR() {
    encoder_ticksR += (digitalRead(SIGNAL_D) == HIGH) ? -1 : 1;
}

void setup() {
    Serial.begin(115200);

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

    // Drive both motors strictly FORWARD at a low PWM
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    analogWrite(EA, 100);

    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    analogWrite(EB, 100);
    
    Serial.println("=== POLARITY TEST INITIATED ===");
    Serial.println("Both wheels are commanded to spin FORWARD.");
    Serial.println("If wiring is correct, BOTH values should INCREASE positively.");
}

void loop() {
    delay(1000);
    noInterrupts();
    long ticksL = encoder_ticksL;
    long ticksR = encoder_ticksR;
    interrupts();
    
    Serial.print("Left Ticks: ");
    Serial.print(ticksL);
    Serial.print(" | Right Ticks: ");
    Serial.println(ticksR);
}
