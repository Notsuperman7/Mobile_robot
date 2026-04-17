#ifndef _PID_H_
#define _PID_H_

#include <Arduino.h>
#include "config_wheels.h"

class PIController {
public:
    PIController(float kp = 0.0f, float ki = 0.0f, float kd=0.0f, float outMin = -220.0f, float outMax = 220.0f); //  addjust the default values to your needs

    void reset();
    float update(float error, float prev_error, float dt);

private:
    float kp_;
    float ki_;
    float kd_;
    float outMin_;
    float outMax_;
    float integral_;
    float derivative_;
};

struct MotorConfig {
    int EN_pin;
    int IN1_pin;
    int IN2_pin;
    String name;
    PIController pi;
    volatile long encoderCount;
    QueueHandle_t M_queue;
    float currentRPM;
};





void IRAM_ATTR encoderISR_BL();
void IRAM_ATTR encoderISR_FL();
void IRAM_ATTR encoderISR_BR();
void IRAM_ATTR encoderISR_FR();

float countsToRPM(long deltaCount, float dt, int ppr);
void setMotor(int EN_pin,int IN1_pin,int IN2_pin,int pwm);

void apply_pid(void* parameter);
extern MotorConfig BL_Motor;
extern MotorConfig FL_Motor;
extern MotorConfig BR_Motor;
extern MotorConfig FR_Motor;

#endif