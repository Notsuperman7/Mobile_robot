#ifndef _CONFIG_H_
#define _CONFIG_H_

#define BL_IN1 2   // addjust it to ur pins 
#define BL_IN2 4
#define BL_EN 15

#define FL_IN1 2
#define FL_IN2 4
#define FL_EN 15

#define BR_IN1 2
#define BR_IN2 4
#define BR_EN 15

#define FR_IN1 2
#define FR_IN2 4
#define FR_EN 15


#define BL_ENC_A 25
#define BL_ENC_B 26

#define BR_ENC_A 32
#define BR_ENC_B 33

#define FL_ENC_A 12
#define FL_ENC_B 14

#define FR_ENC_A 13
#define FR_ENC_B 27

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