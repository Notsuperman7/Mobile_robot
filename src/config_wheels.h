#ifndef _CONFIG_H_
#define _CONFIG_H_

// BL = Back Left
#define BL_IN1 4
#define BL_IN2 5
#define BL_EN  6

#define BL_ENC_A 8
#define BL_ENC_B 9

// FL = Front Left
#define FL_IN1 7
#define FL_IN2 15
#define FL_EN  16

#define FL_ENC_A 10
#define FL_ENC_B 11

// BR = Back Right
#define BR_IN1 17
#define BR_IN2 18
#define BR_EN  21

#define BR_ENC_A 12
#define BR_ENC_B 13

// FR = Front Right
#define FR_IN1 38
#define FR_IN2 39
#define FR_EN  40

#define FR_ENC_A 14
#define FR_ENC_B 47

constexpr int PPR = 515;


constexpr float Kp_BL =3.9619f*1.4f;   
constexpr float Ki_BL = 0.0297f*15.0f;
constexpr float Kd_BL = 36.3636f*0.0f;

constexpr float Kp_FL =4.0198f*1.4f;   
constexpr float Ki_FL = 0.0268f*15.0f;
constexpr float Kd_FL = 33.3333f*0.0f;

constexpr float Kp_BR =4.2489f*1.4f;   
constexpr float Ki_BR = 0.0283f*15.0f;
constexpr float Kd_BR = 33.3333f*0.0f;

constexpr float Kp_FR =4.0046f*1.4f;   
constexpr float Ki_FR = 0.0267f*15.0f;
constexpr float Kd_FR = 33.3333f*0.0f;

constexpr int PWM_MIN = -240;
constexpr int PWM_MAX = 240;



#endif