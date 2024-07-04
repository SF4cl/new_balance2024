#ifndef __CHASSIS_R_TASK_H
#define __CHASSIS_R_TASK_H

#include "main.h"
#include "HT04_drv.h"
#include "GO1_drv.h"
#include "insTask.h"
#include "arm_math.h"
#include "pid.h"
#include "vmcCal.h"
#include "user_lib.h"
#include "stdio.h"
#include "stm32h7xx_it.h"



typedef struct{
    Joint_Motor_t joint_motor[4];
    Wheel_Motor_t wheel_motor[2];

    float v_set;
    float x_set;
    float turn_set;
    float leg_set;
    float last_leg_set;

    float v;
    float x;

    float pitchR;
    float pitchL;
    float pitchGyroR;
    float pitchGyroL;
    float roll;
    float total_yaw;
    float theta_err;

    float T_turn;
    float Tp_2legs;

    uint8_t start_flag;
    uint8_t recover_flag;

} chassis_t;

extern void chassisRtask(void);
extern void chassisR_init(chassis_t *chassis, vmc_t *vmc, PidTypeDef *legr);
extern void chassisR_update(chassis_t *chassis, vmc_t *vmc, INS_t *ins);
extern void chassisR_cal(chassis_t *chassis, vmc_t *vmc, INS_t *ins, PidTypeDef *legr, float *K);
extern float LQR_K_cal(float *coe, float len);
extern void limit(float *in, float min, float max);
extern void compensate_init(PidTypeDef *Tp, PidTypeDef *turn);

// extern void chassisL_init(chassis_t *chassis, vmc_t *vmc, PidTypeDef *legl);
// extern void chassisL_update(chassis_t *chassis, vmc_t *vmc, INS_t *ins);
// extern void chassisL_cal(chassis_t *chassis, vmc_t *vmc, INS_t *ins, PidTypeDef *legl, float *K);

#endif