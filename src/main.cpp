#include <Arduino.h>
#include "pid.h"
#include "config_wheels.h"

void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(BL_BR_IN1, OUTPUT);
    pinMode(BL_BR_IN2, OUTPUT);
    pinMode(BL_BR_ENA, OUTPUT);

    pinMode(FL_FR_IN1, OUTPUT);
    pinMode(FL_FR_IN2, OUTPUT);
    pinMode(FL_FR_ENA, OUTPUT);

    // Encoder pins
    pinMode(BL_ENC_A, INPUT_PULLUP);
    pinMode(BL_ENC_B, INPUT_PULLUP);

    pinMode(BR_ENC_A, INPUT_PULLUP);
    pinMode(BR_ENC_B, INPUT_PULLUP);

    pinMode(FL_ENC_A, INPUT_PULLUP);
    pinMode(FL_ENC_B, INPUT_PULLUP);

    pinMode(FR_ENC_A, INPUT_PULLUP);
    pinMode(FR_ENC_B, INPUT_PULLUP);

    // Stop motors at startup
    setMotor_BL_BR(0);
    setMotor_FL_FR(0);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(BL_ENC_A), encoderISR_BL, RISING);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_A), encoderISR_BR, RISING);
    attachInterrupt(digitalPinToInterrupt(FL_ENC_A), encoderISR_FL, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A), encoderISR_FR, RISING);

    // Initial targets
    targetRPM_BLBR = 0.0f;
    targetRPM_FLFR = 0.0f;

    // Create PID task
    xTaskCreatePinnedToCore(
        apply_pid,
        "PID_Task",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}

void loop() {
    // Example test
    targetRPM_BLBR = 50.0f;
    targetRPM_FLFR = 50.0f;

    delay(100);
}