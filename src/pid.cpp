

#include "pid.h"
#include <Arduino.h>
#include "config_wheels.h"

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

unsigned long lastTime = 0;
long lastCount = 0;

constexpr int sampleTimeMs = 30;
int32_t targetRPM_i = 0;
float targetRPM = 0.0f;

PIController::PIController(float kp, float ki, float Kd, float outMin, float outMax)
    : kp_(kp), ki_(ki), kd_(Kd),outMin_(outMin), outMax_(outMax), integral_(0.0f) {}

void PIController::reset() {
    integral_ = 0.0f;
}

float PIController::update(float error, float prev_error, float dt) {
    
    integral_ += error * dt;
    derivative_ = (error - prev_error) / dt;


    float output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    output = constrain(output, outMin_, outMax_);

    if ((output > outMax_) || (output < outMin_)) {
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
    MotorConfig* motorConfig = (MotorConfig*)passedConfig;
    PIController pi = motorConfig->pi;
    QueueHandle_t targetQueue = motorConfig->M_queue;
    float targetRPM = 0.0f;
    
    int pwm;
    long prevCount = 0;
    float prevError = 0;

    unsigned long lastTime = millis();

    while (true) {
        // FIX 1: Non-blocking read (0 ms wait). If no new command, keep driving at current target!
        if (xQueueReceive(targetQueue, &targetRPM_i, 0) == pdTRUE) {
            targetRPM = (float)targetRPM_i;
        }
        
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        
        // Safety check to prevent divide-by-zero on the very first loop
        if (dt <= 0.0f) dt = 0.03f; 
        
        lastTime = now;

        long count;
        portENTER_CRITICAL(&mux);
        count = motorConfig->encoderCount;
        portEXIT_CRITICAL(&mux);

        long dCount = count - prevCount;
        prevCount = count;

        float measuredRPM = countsToRPM(dCount, dt, PPR);
        motorConfig->currentRPM = measuredRPM;

        if (targetRPM == 0) {
            pi.reset();
        }
        
        float error = targetRPM - measuredRPM;
        if(abs(error) < abs(targetRPM) * 0.05f) { // Deadband
            error = 0;
        }

        pwm = (int)pi.update(error, prevError, dt);

        if (pwm != 0 && abs(pwm) < 40) {    // safe minimum PWM 
            pwm = (pwm > 0) ? 40 : -40;
        }

        prevError = error;

        setMotor(motorConfig->EN_pin, motorConfig->IN1_pin, motorConfig->IN2_pin, pwm);
        
        // FIX 2: Rely entirely on FreeRTOS to maintain a perfect 30ms loop
        vTaskDelay(pdMS_TO_TICKS(sampleTimeMs));
    }
}

