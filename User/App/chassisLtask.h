#ifndef __CHASSIS_L_TASK_H
#define __CHASSIS_L_TASK_H

#include "main.h"
#include "HT04_drv.h"
#include "GO1_drv.h"
#include "insTask.h"
#include "arm_math.h"
#include "pid.h"
#include "vmcCal.h"
#include "chassisRtask.h"

extern void chassisLtask(void);
extern void chassisL_init(chassis_t *chassis, vmc_t *vmc, PidTypeDef *legl);
extern void chassisL_update(chassis_t *chassis, vmc_t *vmc, INS_t *ins);
extern void chassisL_cal(chassis_t *chassis, vmc_t *vmc, INS_t *ins, PidTypeDef *legl, float *K);

#endif