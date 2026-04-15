#ifndef _PID_H_
#define _PID_H_

#include <Arduino.h>
#include "config_wheels.h"

class PIController {
public:
    PIController(float kp = 0.0f, float ki = 0.0f, float outMin = -220.0f, float outMax = 220.0f); //  addjust the default values to your needs

    void reset();
    float update(float target, float measured, float dt);

private:
    float kp_;
    float ki_;
    float outMin_;
    float outMax_;
    float integral_;
};

void IRAM_ATTR encoderISR_BL();
void IRAM_ATTR encoderISR_FL();
void IRAM_ATTR encoderISR_BR();
void IRAM_ATTR encoderISR_FR();

float countsToRPM(long deltaCount, float dt, int ppr);
void setMotor_BL_BR(int pwm);
void setMotor_FL_FR(int pwm);

void apply_pid(void* parameter);



#endif