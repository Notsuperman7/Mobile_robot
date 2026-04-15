#include "pid.h"
#include <Arduino.h>
#include "config_wheels.h"


volatile long encoderCount_BL = 0;
volatile long encoderCount_FL = 0;
volatile long encoderCount_BR = 0;
volatile long encoderCount_FR = 0;

volatile uint32_t isrCallCount = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

float targetRPM_BLBR = 0.0f; // target RPM for BL and BR addjust it to your needs
float targetRPM_FLFR = 0.0f; // target RPM for FL and FR addjust it to your needs

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


PIController pi_BLBR((Kp_BL + Kp_BR) * 0.5f, (Ki_BL + Ki_BR) * 0.5f, PWM_MIN, PWM_MAX);
PIController pi_FLFR((Kp_FL + Kp_FR) * 0.5f, (Ki_FL + Ki_FR) * 0.5f, PWM_MIN, PWM_MAX);




void IRAM_ATTR encoderISR_BL() {
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(BL_ENC_B)) encoderCount_BL--;
    else                       encoderCount_BL++;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR_FL() {
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(FL_ENC_B)) encoderCount_FL--;
    else                       encoderCount_FL++;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR_BR() {
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(BR_ENC_B)) encoderCount_BR--;
    else                       encoderCount_BR++;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR_FR() {
    portENTER_CRITICAL_ISR(&mux);
    if (digitalRead(FR_ENC_B)) encoderCount_FR--;
    else                       encoderCount_FR++;
    portEXIT_CRITICAL_ISR(&mux);
}

float countsToRPM(long deltaCount, float dt, int ppr) {
    return (deltaCount * 60.0f) / (ppr * dt);
}


void setMotor_BL_BR(int pwm)
{
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);
    analogWrite(BL_BR_ENA, abs(pwm));

    if (pwm > 0) {
        digitalWrite(BL_BR_IN1, LOW);
        digitalWrite(BL_BR_IN2, HIGH);
    } else if (pwm < 0) {
        digitalWrite(BL_BR_IN1, HIGH);
        digitalWrite(BL_BR_IN2, LOW);
    } else {
        digitalWrite(BL_BR_IN1, LOW);
        digitalWrite(BL_BR_IN2, LOW);
    }
}

void setMotor_FL_FR(int pwm)
{
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);
    analogWrite(FL_FR_ENA, abs(pwm));

    if (pwm > 0) {
        digitalWrite(FL_FR_IN1, LOW);
        digitalWrite(FL_FR_IN2, HIGH);
    } else if (pwm < 0) {
        digitalWrite(FL_FR_IN1, HIGH);
        digitalWrite(FL_FR_IN2, LOW);
    } else {
        digitalWrite(FL_FR_IN1, LOW);
        digitalWrite(FL_FR_IN2, LOW);
    }
}

void apply_pid(void *pvParameters) {
    Serial.println("Starting velocity PI control");

    long prevCount_BL = 0;
    long prevCount_FL = 0;
    long prevCount_BR = 0;
    long prevCount_FR = 0;

    unsigned long lastTime = millis();

    while (true) {
        unsigned long now = millis();
        if (now - lastTime < sampleTimeMs) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;

        if (dt <= 0.0f) {
            continue;
        }

        long countBL, countFL, countBR, countFR;

        portENTER_CRITICAL(&mux);
        countBL = encoderCount_BL;
        countFL = encoderCount_FL;
        countBR = encoderCount_BR;
        countFR = encoderCount_FR;
        portEXIT_CRITICAL(&mux);

        long dBL = countBL - prevCount_BL;
        long dFL = countFL - prevCount_FL;
        long dBR = countBR - prevCount_BR;
        long dFR = countFR - prevCount_FR;

        prevCount_BL = countBL;
        prevCount_FL = countFL;
        prevCount_BR = countBR;
        prevCount_FR = countFR;

        float rpmBL = countsToRPM(dBL, dt, PPR);
        float rpmFL = countsToRPM(dFL, dt, PPR);
        float rpmBR = countsToRPM(dBR, dt, PPR);
        float rpmFR = countsToRPM(dFR, dt, PPR);

        float measuredRPM_BLBR = 0.5f * (rpmBL + rpmBR);
        float measuredRPM_FLFR = 0.5f * (rpmFL + rpmFR);

        if (targetRPM_BLBR == 0) {
            pi_BLBR.reset();
        }
        if (targetRPM_FLFR == 0) {
            pi_FLFR.reset();
        }

        int pwmBLBR = (int)pi_BLBR.update(targetRPM_BLBR, measuredRPM_BLBR, dt);
        int pwmFLFR = (int)pi_FLFR.update(targetRPM_FLFR, measuredRPM_FLFR, dt);

        if (pwmBLBR != 0 && abs(pwmBLBR) < 40) {    // safe minimum PWM 
            pwmBLBR = (pwmBLBR > 0) ? 40 : -40;
        }
        if (pwmFLFR != 0 && abs(pwmFLFR) < 40) {
            pwmFLFR = (pwmFLFR > 0) ? 40 : -40;
        }

        setMotor_BL_BR(pwmBLBR);
        setMotor_FL_FR(pwmFLFR);

        Serial.print("BL:");
        Serial.print(rpmBL);
        Serial.print(" BR:");
        Serial.print(rpmBR);
        Serial.print(" AVG_R:");
        Serial.print(measuredRPM_BLBR);
        Serial.print(" PWM_R:");
        Serial.print(pwmBLBR);

        Serial.print(" | FL:");
        Serial.print(rpmFL);
        Serial.print(" FR:");
        Serial.print(rpmFR);
        Serial.print(" AVG_F:");
        Serial.print(measuredRPM_FLFR);
        Serial.print(" PWM_F:");
        Serial.println(pwmFLFR);
    }
}