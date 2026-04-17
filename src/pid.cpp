#include "pid.h"
#include <Arduino.h>
#include "config_wheels.h"

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

unsigned long lastTime = 0;
long lastCount = 0;

constexpr int sampleTimeMs = 100;


PIController::PIController(float kp, float ki, float outMin, float outMax)
    : kp_(kp), ki_(ki), outMin_(outMin), outMax_(outMax), integral_(0.0f) {}

void PIController::reset() {
    integral_ = 0.0f;
}

float PIController::update(float target, float measured, float dt) {
    float error = target - measured;
    integral_ += error * dt;

    float output = kp_ * error + ki_ * integral_;

    if (output > outMax_) {
        output = outMax_;
        if (error > 0) integral_ -= error * dt;
    } 
    else if (output < outMin_) {
        output = outMin_;
        if (error < 0) integral_ -= error * dt;
    }

    return output;
}


void IRAM_ATTR encoderISR_BL() {
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(BL_ENC_B)) BL_Motor.encoderCount--;
    else                       BL_Motor.encoderCount++;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR_FL() {
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(FL_ENC_B)) FL_Motor.encoderCount--;
    else                       FL_Motor.encoderCount++;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR_BR() {
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(BR_ENC_B)) BR_Motor.encoderCount--;
    else                       BR_Motor.encoderCount++;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR_FR() {
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(FR_ENC_B)) FR_Motor.encoderCount--;
    else                       FR_Motor.encoderCount++;
    portEXIT_CRITICAL_ISR(&mux);
}

float countsToRPM(long deltaCount, float dt, int ppr) {
    return (deltaCount * 60.0f) / (ppr * dt);
}


void setMotor(int EN_pin,int IN1_pin,int IN2_pin,int pwm)
{
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);
    analogWrite(EN_pin, abs(pwm));

    if (pwm > 0) {
        digitalWrite(IN1_pin, LOW);
        digitalWrite(IN2_pin, HIGH);
    } else if (pwm < 0) {
        digitalWrite(IN1_pin, HIGH);
        digitalWrite(IN2_pin, LOW);
    } else {
        digitalWrite(IN1_pin, LOW);
        digitalWrite(IN2_pin, LOW);
    }
}

void apply_pid(void *passedConfig) {
    MotorConfig*  motorConfig = (MotorConfig*)passedConfig;
    PIController pi = motorConfig->pi;
    QueueHandle_t targetQueue = motorConfig->M_queue;
    float targetRPM;
    float measuredRPM;
    
    long prevCount = 0;

    unsigned long lastTime = millis();

    while (true) {
        xQueueReceive(targetQueue, &targetRPM, pdMS_TO_TICKS(100));
        
        unsigned long now = millis();
        if (now - lastTime < sampleTimeMs) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;

        long count;

        portENTER_CRITICAL(&mux);
        count = motorConfig->encoderCount;
        portEXIT_CRITICAL(&mux);

        long dCount = count - prevCount;
        prevCount = count;


        float measuredRPM = countsToRPM(dCount, dt, PPR);


        if (targetRPM == 0) {
            pi.reset();
        }

        int pwm = (int)pi.update(targetRPM, measuredRPM, dt);

        if (pwm != 0 && abs(pwm) < 40) {    // safe minimum PWM 
            pwm = (pwm > 0) ? 40 : -40;
        }

        setMotor(motorConfig->EN_pin, motorConfig->IN1_pin, motorConfig->IN2_pin, pwm);
        motorConfig->currentRPM = measuredRPM;
        vTaskDelay(pdMS_TO_TICKS(100));

    }
}