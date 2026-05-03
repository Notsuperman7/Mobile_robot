#ifndef _CONFIG_H_
#define _CONFIG_H_

// BL = Back Left
#define BL_IN1 7
#define BL_IN2 15
#define BL_EN  6

#define BL_ENC_A 4
#define BL_ENC_B 5

// FL = Front Left
#define FL_IN1 40
#define FL_IN2 39
#define FL_EN  41

#define FL_ENC_A 1
#define FL_ENC_B 2

// BR = Back Right
#define BR_IN1 8
#define BR_IN2 9
#define BR_EN  18

#define BR_ENC_A 16
#define BR_ENC_B 17

// FR = Front Right
#define FR_IN1 20
#define FR_IN2 21
#define FR_EN  47

#define FR_ENC_A 38
#define FR_ENC_B 48

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