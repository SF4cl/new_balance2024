#include "chassisRtask.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "remoteTask.h"

extern DMA_HandleTypeDef hdma_usart1_tx;

extern INS_t INS;
extern vmc_t vmc_L;
extern RM_Remote_t *remote;
extern PidTypeDef LegL_Pid;

float LQR_K_R[12] = {
    -15.7725, -1.4432, -2.8507, -2.8933, 9.5062, 2.1972,
    18.0773, 2.5909, 9.1851, 8.3496, 20.4557, 2.3536};

float Poly_Coefficient[12][4] = {

296.846839,-84.263650,-61.728856,-1.394435,
1013.987867,-745.343868,181.671621,5.084916,
81.122744,-50.041981,-0.738978,-0.182728,
35.028242,-43.262381,17.967829,0.588639,
-3.396444,50.131040,-29.478609,-0.489269,
455.077941,-206.323554,-3.875520,15.603733,
27.691559,25.033137,-24.261944,-0.718436,
345.609466,-157.130658,-4.479293,13.708661,
536.191957,-253.532384,-2.462957,21.504384,
89.592424,-539.608683,311.011599,-3.596440,
47.806337,-27.302664,1.809522,2.480727,
-60.837954,-14.776359,26.546842,-2.130430
};

vmc_t vmc_R;
chassis_t chassis_move;

PidTypeDef LegR_Pid;
PidTypeDef Turn_Pid;
PidTypeDef Tp_Pid;

first_order_filter_type_t d_phi1_r_filter;
first_order_filter_type_t d_phi4_r_filter;
first_order_filter_type_t pitch_gyro_r_filter;

float d_phi1_r_filter_para[1] = {0.05};
float d_phi4_r_filter_para[1] = {0.05};
float pitch_gyro_r_filter_para[1] = {0.01};

float crtR = 35.0f;
float Rtest = 0.0f;

void chassisRtask(void)
{

  while (INS.ins_flag == 0)
  {
    osDelay(1);
  }

  chassisR_init(&chassis_move, &vmc_R, &LegR_Pid);

  compensate_init(&Tp_Pid, &Turn_Pid);

  while (1)
  {
    chassisR_update(&chassis_move, &vmc_R, &INS);

    chassisR_cal(&chassis_move, &vmc_R, &INS, &LegR_Pid, LQR_K_R);

    if (chassis_move.start_flag == 1)
    {
     canSend_comd(RFmotor, 0.0f, 0.0f, 0.0f, 0.0f, vmc_R.T_set[1] / 6.0f);

     canSend_comd(RBmotor, 0.0f, 0.0f, 0.0f, 0.0f, vmc_R.T_set[0] / 6.0f);

//     canSend_comd(RFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//     
//     canSend_comd(RBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

//      wheel_Rctrl(-chassis_move.wheel_motor[1].T_wheel / 6.33f);
      wheel_Rctrl(0.0f);
      					//wheel_Rtest(Rtest);

      osDelay(2);
    }
    else if (chassis_move.start_flag == 0)
    {
      canSend_comd(RFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      canSend_comd(RBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      wheel_Rctrl(0.0f);

      osDelay(2);
    }
  }
}

void chassisR_init(chassis_t *chassis, vmc_t *vmc, PidTypeDef *legr)
{
  const static float legr_pid[3] = {LEG_RIGHT_PID_KP, LEG_RIGHT_PID_KI, LEG_RIGHT_PID_KD};

  set_mode(1);

  canSend_mode(RFmotor, CMD_MOTOR_MODE);
  osDelay(2);
  canSend_mode(RBmotor, CMD_MOTOR_MODE);
  osDelay(2);
  osDelay(500);

  canSend_comd(RFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  osDelay(2);
  canSend_comd(RBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  osDelay(2);

  osDelay(500);

  canSend_mode(RFmotor, CMD_ZERO_POSITION);
  osDelay(2);
  canSend_mode(RBmotor, CMD_ZERO_POSITION);
  osDelay(2);

  osDelay(500);

  canSend_mode(RFmotor, CMD_MOTOR_MODE);
  osDelay(2);
  canSend_mode(RBmotor, CMD_MOTOR_MODE);
  osDelay(2);

  osDelay(500);

  VMC_init(vmc);

  PID_init(legr, PID_POSITION, legr_pid, LEG_RIGHT_PID_MAX_OUT, LEG_RIGHT_PID_MAX_IOUT);

  // 滤波器初始化
  first_order_filter_init(&d_phi1_r_filter, 0.002f, d_phi1_r_filter_para);
  first_order_filter_init(&d_phi4_r_filter, 0.002f, d_phi4_r_filter_para);
  first_order_filter_init(&pitch_gyro_r_filter, 0.002f, pitch_gyro_r_filter_para);
}

void chassisR_update(chassis_t *chassis, vmc_t *vmc, INS_t *ins)
{
  vmc->phi4 = chassis->joint_motor[3].para.pos - 0.3925f + 0.0233f;
  vmc->phi1 = chassis->joint_motor[2].para.pos + pi + 0.3925f + 0.0233f;

  vmc->d_phi4 = chassis->joint_motor[3].para.vel;
  vmc->d_phi1 = chassis->joint_motor[2].para.vel;

  chassis->pitchR = -ins->Pitch + 0.021f; //修正pitch值
  chassis->pitchGyroR = -ins->Gyro[1];

  chassis->total_yaw = ins->YawTotalAngle;
  chassis->roll = ins->Roll;

  chassis->theta_err = -vmc->theta - vmc_L.theta;

  first_order_filter_cali(&d_phi1_r_filter, vmc->d_phi1);
  first_order_filter_cali(&d_phi4_r_filter, vmc->d_phi4);
  first_order_filter_cali(&pitch_gyro_r_filter, chassis->pitchGyroR);

  // 把滤波后的值赋给vmc
  vmc->d_phi1 = d_phi1_r_filter.out;
  vmc->d_phi4 = d_phi4_r_filter.out;
  chassis->pitchGyroR = pitch_gyro_r_filter.out;
  // todo: 倒地检测
}

void chassisR_cal(chassis_t *chassis, vmc_t *vmc, INS_t *ins, PidTypeDef *legr, float *K)
{
  // VMC运动学正解
  VMC_R_cal1(vmc, chassis->pitchR, chassis->pitchGyroR);

  for (int i = 0; i < 12; i++)
  {
    K[i] = LQR_K_cal(&Poly_Coefficient[i][0], vmc->L0);
  }

  chassis->T_turn = Turn_Pid.Kp * (chassis->turn_set - chassis->total_yaw) + Turn_Pid.Kd * (0.0f - ins->Gyro[2]);

  chassis->Tp_2legs = PID_Calc(&Tp_Pid, chassis->theta_err, 0.0f);

//  chassis->x_set = 0.0f;//x_set 在不断累加，暂时先强制置零
  chassis->wheel_motor[1].T_wheel = -(K[0] * vmc->theta + K[1] * vmc->d_theta + K[2] * (chassis->x-chassis->x_set) + K[3] * chassis->v + K[4] * chassis->pitchR  + K[5] * chassis->pitchGyroR);
  vmc->Tp = -(K[6] * vmc->theta + K[7] * vmc->d_theta+ K[8] * (chassis->x - chassis->x_set) + K[9] * chassis->v + K[10] * chassis->pitchR + K[11] * chassis->pitchGyroR);
  // chassis->wheel_motor[1].T_wheel = chassis->wheel_motor[1].T_wheel - chassis->T_turn;
  //! 防劈叉
  // vmc->Tp = vmc->Tp - chassis->Tp_2legs;

  vmc->F0 = crtR + PID_Calc(legr, vmc->L0, chassis->leg_set);

  limit(&chassis->wheel_motor[1].T_wheel, -5.0f, 5.0f);
  limit(&vmc->F0, -300.0f, 300.0f);

  VMC_cal2(vmc);

  limit(&vmc->T_set[0], -5.0f, 5.0f);
  limit(&vmc->T_set[1], -5.0f, 5.0f);
}

void compensate_init(PidTypeDef *Tp, PidTypeDef *turn)
{
  const static float Tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
  const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};

  PID_init(Tp, PID_POSITION, Tp_pid, TP_PID_MAX_OUT, TP_PID_MAX_IOUT);
  PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
}

float LQR_K_cal(float *coe, float len)
{
  return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}

void limit(float *in, float min, float max)
{
  if (*in < min)
  {
    *in = min;
  }
  else if (*in > max)
  {
    *in = max;
  }
}
