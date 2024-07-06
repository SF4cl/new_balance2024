#ifndef __VMC_CAL_H
#define __VMC_CAL_H

#include "main.h"
#include "insTask.h"
#include "arm_math.h"

#define pi 3.1415926f

#define LEG_LEFT_PID_KP  0.0f
#define LEG_LEFT_PID_KI  0.0f
#define LEG_LEFT_PID_KD  0.0f
#define LEG_LEFT_PID_MAX_OUT  100.0f 
#define LEG_LEFT_PID_MAX_IOUT 30.0f

#define LEG_RIGHT_PID_KP  0.0f
#define LEG_RIGHT_PID_KI  0.0f
#define LEG_RIGHT_PID_KD  0.0f
#define LEG_RIGHT_PID_MAX_OUT  100.0f 
#define LEG_RIGHT_PID_MAX_IOUT 30.0f

#define TP_PID_KP  0.0f
#define TP_PID_KI  0.0f
#define TP_PID_KD  0.0f
#define TP_PID_MAX_OUT  20.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 0.0f
#define TURN_PID_KI 0.0f
#define TURN_PID_KD 0.0f
#define TURN_PID_MAX_OUT 10.0f
#define TURN_PID_MAX_IOUT 0.0f

typedef struct{
    float l5;
    float l1;
    float l2;
    float l3;
    float l4;

    float xB,yB;
    float xD,yD;
    float d_xB, d_yB;
    float d_xD, d_yD;

    float xC,yC;
    float L0,phi0;
    float d_xc, d_yc;

    float lBD;

    float d_phi0;
    float last_phi0;

    float A0, B0, C0;
    float phi2, phi3;
    float d_phi2;
    float phi1, phi4;
    float d_phi1, d_phi4;

    float J11, J12, J21, J22;
    float T_set[2];

    float F0;
    float Tp;

    float theta;
    float d_theta;
    float d2_theta;
    float last_d_theta;   

    float d_L0;
    float d2_L0;
    float last_L0;
    float last_d_L0;

    float Fn;

    float alpha;
    float d_alpha;

    uint8_t vmc_flag;
    uint8_t first_flag;
}vmc_t;

extern void VMC_init(vmc_t *vmc);
extern void VMC_R_cal1(vmc_t *vmc, float bodyPitchAngle,float bodyPitchGyro);
extern void VMC_L_cal1(vmc_t *vmc, float bodyPitchAngle,float bodyPitchGyro);
extern void VMC_cal2(vmc_t *vmc);

#endif