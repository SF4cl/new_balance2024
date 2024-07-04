#ifndef __HT04_DRV_H__
#define __HT04_DRV_H__

#include "main.h"
#include "fdcan.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define RFmotor  0x04
#define RBmotor  0x03
#define LBmotor  0x02
#define LFmotor  0x01

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

typedef struct{
    uint8_t id;
    float pos;
    float vel;
    float tor;
}Jmotor_fbpara_t;

typedef struct{
    uint8_t mode;
    Jmotor_fbpara_t para;
}Joint_Motor_t;

static void canTransmit(uint8_t id, uint8_t *buf, uint8_t len);
extern void canSend_comd(uint8_t id, float f_p, float f_v, float f_kp, float f_kd, float f_t);
extern void canSend_mode(uint8_t id, uint8_t mode);

#endif
