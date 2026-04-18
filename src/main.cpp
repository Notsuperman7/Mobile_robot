#include <Arduino.h>
#include "pid.h"
#include "config_wheels.h"

PIController pi_BL((Kp_BL) , (Ki_BL) , (Kd_BL), PWM_MIN, PWM_MAX);
PIController pi_FL((Kp_FL) , (Ki_FL) , (Kd_FL), PWM_MIN, PWM_MAX);
PIController pi_BR((Kp_BR) , (Ki_BR) , (Kd_BR), PWM_MIN, PWM_MAX);
PIController pi_FR((Kp_FR) , (Ki_FR) , (Kd_FR), PWM_MIN, PWM_MAX);


QueueHandle_t BR_target_queue;
QueueHandle_t FR_target_queue;
QueueHandle_t BL_target_queue;
QueueHandle_t FL_target_queue;

typedef struct MotorConfig MotorConfig;

MotorConfig BL_Motor = {
    .EN_pin = BL_EN,
    .IN1_pin = BL_IN1,
    .IN2_pin = BL_IN2,
    .name = "BL",
    .pi = pi_BL,
    .encoderCount = 0,
    .M_queue = NULL
};

MotorConfig FL_Motor = {
    .EN_pin = FL_EN,
    .IN1_pin = FL_IN1,
    .IN2_pin = FL_IN2,
    .name = "FL",
    .pi = pi_FL,
    .encoderCount = 0,
    .M_queue = NULL
};

MotorConfig BR_Motor = {
    .EN_pin = BR_EN,
    .IN1_pin = BR_IN1,
    .IN2_pin = BR_IN2,
    .name = "BR",
    .pi = pi_BR,
    .encoderCount = 0,
    .M_queue = NULL
};

MotorConfig FR_Motor = {
    .EN_pin = FR_EN,
    .IN1_pin = FR_IN1,
    .IN2_pin = FR_IN2,
    .name = "FR",
    .pi = pi_FR,
    .encoderCount = 0,
    .M_queue = NULL
};
void startMotors(void* pvprm) {
    const float targetRPM = 90.0f;
    while (true) {
        xQueueSend(BR_target_queue, &targetRPM, 0);
        xQueueSend(FR_target_queue, &targetRPM, 0);
        xQueueSend(BL_target_queue, &targetRPM, 0);
        xQueueSend(FL_target_queue, &targetRPM, 0);
        delay(50);
    }
}

void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(BL_IN1, OUTPUT);
    pinMode(BL_IN2, OUTPUT);
    pinMode(BL_EN, OUTPUT);

    pinMode(FL_IN1, OUTPUT);
    pinMode(FL_IN2, OUTPUT);
    pinMode(FL_EN, OUTPUT);

    pinMode(BR_IN1, OUTPUT);
    pinMode(BR_IN2, OUTPUT);
    pinMode(BR_EN, OUTPUT);

    pinMode(FR_IN1, OUTPUT);
    pinMode(FR_IN2, OUTPUT);
    pinMode(FR_EN, OUTPUT);

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
    setMotor(BL_EN, BL_IN1, BL_IN2, 0);
    setMotor(FL_EN, FL_IN1, FL_IN2, 0);
    setMotor(BR_EN, BR_IN1, BR_IN2, 0);
    setMotor(FR_EN, FR_IN1, FR_IN2, 0);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(BL_ENC_A), encoderISR_BL, RISING);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_A), encoderISR_BR, RISING);
    attachInterrupt(digitalPinToInterrupt(FL_ENC_A), encoderISR_FL, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A), encoderISR_FR, RISING);

    // Initial targets
    BR_target_queue = xQueueCreate(10, sizeof(float));
    FR_target_queue = xQueueCreate(10, sizeof(float));
    BL_target_queue = xQueueCreate(10, sizeof(float));
    FL_target_queue = xQueueCreate(10, sizeof(float));

    BL_Motor.M_queue = BL_target_queue;
    FL_Motor.M_queue = FL_target_queue;
    BR_Motor.M_queue = BR_target_queue;
    FR_Motor.M_queue = FR_target_queue;

    // Create PID task
    xTaskCreate(
        apply_pid,
        "BL_Task",
        4096,
        (void *)&BL_Motor,
        1,
        NULL
    );
        xTaskCreate(
        apply_pid,
        "FL_Task",
        4096,
        (void *)&FL_Motor,
        1,
        NULL
    );
        xTaskCreate(
        apply_pid,
        "BR_Task",
        4096,
        (void *)&BR_Motor,
        1,
        NULL
    );
        xTaskCreate(
        apply_pid,
        "FR_Task",
        4096,
        (void *)&FR_Motor,
        1,
        NULL
    );
        xTaskCreate(
        startMotors,
        "Start_Motors",
        4096,
        NULL,
        2,
        NULL
    );
}

void loop() {
    delay(100);
}