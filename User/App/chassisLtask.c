#include "chassisLtask.h"
#include "fdcan.h"
#include "cmsis_os.h"

vmc_t vmc_L;

PidTypeDef LegL_Pid;

extern DMA_HandleTypeDef hdma_usart1_tx;

extern first_order_filter_type_t phi1_r_filter;
extern first_order_filter_type_t d_phi1_r_filter;
extern first_order_filter_type_t phi4_r_filter;
extern first_order_filter_type_t d_phi4_r_filter;
extern first_order_filter_type_t phi0_r_filter;
extern first_order_filter_type_t d_phi0_r_filter;
extern first_order_filter_type_t theta_r_filter;
extern first_order_filter_type_t d_theta_r_filter;
extern first_order_filter_type_t L0_r_filter;
extern first_order_filter_type_t wheel_r_speed_filter;
extern first_order_filter_type_t pitchGryoR_filter;
extern first_order_filter_type_t wheel_speed_filter;

first_order_filter_type_t phi1_l_filter;
first_order_filter_type_t d_phi1_l_filter;
first_order_filter_type_t phi4_l_filter;
first_order_filter_type_t d_phi4_l_filter;
first_order_filter_type_t phi0_l_filter;
first_order_filter_type_t d_phi0_l_filter;
first_order_filter_type_t theta_l_filter;
first_order_filter_type_t d_theta_l_filter;
first_order_filter_type_t L0_l_filter;
first_order_filter_type_t wheel_l_speed_filter;
first_order_filter_type_t pitchGryoL_filter;
first_order_filter_type_t LFjoint_motor_filter;
first_order_filter_type_t LBjoint_motor_filter;

fp32 lpara1[1] = {0.01};
fp32 lpara2[1] = {0.008};
fp32 lpara3[1] = {0.003};
fp32 lpara4[1] = {0.007};
fp32 lpara5[1] = {0.008};
fp32 lpara6[1] = {0.05};
fp32 lpara7[1] = {0.03};

extern chassis_t chassis_move;
extern INS_t INS;
extern vmc_t vmc_R;

float LQR_K_L[12] = {
    -15.7725, -1.4432, -2.8507, -2.8933, 9.5062, 2.1972,
    18.0773, 2.5909, 9.1851, 8.3496, 20.4557, 2.3536};

extern float Poly_Coefficient[12][4];

float crtL = 35.0f;

float Ltest = 0.0f;

void chassisLtask(void)
{
  while (INS.ins_flag == 0)
  {
    osDelay(1);
  }

  chassisL_init(&chassis_move, &vmc_L, &LegL_Pid);

  while (1)
  {
    chassisL_update(&chassis_move, &vmc_L, &INS);

    chassisL_cal(&chassis_move, &vmc_L, &INS, &LegL_Pid, LQR_K_L);

    if (chassis_move.start_flag == 1)
    {
      //            canSend_comd(LFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      //            canSend_comd(LBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      canSend_comd(LFmotor, 0.0f, 0.0f, 0.0f, 0.0f, -vmc_L.T_set[1] / 6.0f);

      canSend_comd(LBmotor, 0.0f, 0.0f, 0.0f, 0.0f, -vmc_L.T_set[0] / 6.0f);

      wheel_Lctrl(chassis_move.wheel_motor[0].T_wheel / 6.33f);
      //						wheel_Lctrl(0.0f);
      //					wheel_Lctrl(0.05f);
      //					wheel_Lctrl(Ltest);
      osDelay(2);
    }
    else if (chassis_move.start_flag == 0)
    {
      canSend_comd(LFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      canSend_comd(LBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      wheel_Lctrl(0.0f);
      osDelay(2);
    }
  }
}

void chassisL_init(chassis_t *chassis, vmc_t *vmc, PidTypeDef *legl)
{
  const static float legl_pid[3] = {LEG_LEFT_PID_KP, LEG_LEFT_PID_KI, LEG_LEFT_PID_KD};

  set_mode(1);

  canSend_mode(LFmotor, CMD_MOTOR_MODE);
  osDelay(2);
  canSend_mode(LBmotor, CMD_MOTOR_MODE);
  osDelay(2);

  osDelay(500);

  canSend_comd(LFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  osDelay(2);
  canSend_comd(LBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  osDelay(2);

  osDelay(500);

  canSend_mode(LFmotor, CMD_ZERO_POSITION);
  osDelay(2);
  canSend_mode(LBmotor, CMD_ZERO_POSITION);
  osDelay(2);

  osDelay(500);

  canSend_mode(LFmotor, CMD_MOTOR_MODE);
  osDelay(2);
  canSend_mode(LBmotor, CMD_MOTOR_MODE);
  osDelay(2);

  osDelay(500);

  VMC_init(vmc);

  PID_init(legl, PID_POSITION, legl_pid, LEG_LEFT_PID_MAX_OUT, LEG_LEFT_PID_MAX_IOUT);
}

void chassisL_update(chassis_t *chassis, vmc_t *vmc, INS_t *ins)
{
  vmc->phi4 = -chassis->joint_motor[0].para.pos + pi / 2.0f - 1.9823f;
  vmc->phi1 = -chassis->joint_motor[1].para.pos + pi / 2.0f + 1.9823f;
  
  vmc->d_phi1 = -chassis->joint_motor[1].para.vel;
  vmc->d_phi4 = -chassis->joint_motor[0].para.vel;

  chassis->pitchL = -ins->Pitch;
  chassis->pitchGyroL = -ins->Gyro[1];
  // todo: filter
  // todo: 倒地检测
}

void chassisL_cal(chassis_t *chassis, vmc_t *vmc, INS_t *ins, PidTypeDef *legl, float *K)
{
  VMC_L_cal1(vmc, chassis->pitchL, chassis->pitchGyroL);

  for (int i = 0; i < 12; i++)
  {
    K[i] = LQR_K_cal(&Poly_Coefficient[i][0], vmc->L0);
  }

  chassis->wheel_motor[0].T_wheel = (K[0] * (vmc->theta - 0.0f) + K[1] * (vmc->d_theta - 0.0f) + K[2] * (chassis->x - chassis->x_set) + K[3] * (chassis->v - chassis->v_set) + K[4] * (chassis->pitchL - 0.0f) + K[5] * (chassis->pitchGyroL - 0.0f));
  vmc->Tp = (K[6] * (vmc->theta - 0.0f) + K[7] * (vmc->d_theta - 0.0f) + K[8] * (chassis->x - chassis->x_set) + K[9] * (chassis->v - chassis->v_set) + K[10] * (chassis->pitchL - 0.0f) + K[11] * (chassis->pitchGyroL - 0.0f));
  chassis->wheel_motor[0].T_wheel = chassis->wheel_motor[0].T_wheel - chassis->T_turn;

  //! 防劈叉
  // vmc->Tp = vmc->Tp - chassis->Tp_2legs;

  vmc->F0 = crtL + PID_Calc(legl, vmc->L0, chassis->leg_set);

  limit(&chassis->wheel_motor[0].T_wheel, -5.0f, 5.0f);
  limit(&vmc->F0, -150.0f, 150.0f);

  VMC_cal2(vmc);

  limit(&vmc->T_set[0], -15.0f, 15.0f);
  limit(&vmc->T_set[1], -15.0f, 15.0f);
}
