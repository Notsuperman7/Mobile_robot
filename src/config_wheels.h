#ifndef _CONFIG_H_
#define _CONFIG_H_

#define BL_IN1 18   // addjust it to ur pins 
#define BL_IN2 19
#define BL_EN 5

#define FL_IN1 16
#define FL_IN2 17
#define FL_EN 4

#define BR_IN1 32
#define BR_IN2 33
#define BR_EN 23

#define FR_IN1 2
#define FR_IN2 0
#define FR_EN 15


#define BL_ENC_A 26
#define BL_ENC_B 25

#define BR_ENC_A 34
#define BR_ENC_B 35

#define FL_ENC_A 14
#define FL_ENC_B 12

#define FR_ENC_A 21
#define FR_ENC_B 22

constexpr int PPR = 515;


constexpr float Kp_BL =3.9619f;   
constexpr float Ki_BL = 0.0297f;

constexpr float Kp_FL =4.0198f;   
constexpr float Ki_FL = 0.0402f;

constexpr float Kp_BR =4.2489f;   
constexpr float Ki_BR = 0.0425f;

constexpr float Kp_FR =4.0046f;   
constexpr float Ki_FR = 0.0400f;

constexpr int PWM_MIN = -240;
constexpr int PWM_MAX = 240;



#endif