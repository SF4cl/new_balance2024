#include "chassisRtask.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "remoteTask.h"

extern DMA_HandleTypeDef hdma_usart1_tx;

extern INS_t INS;
extern vmc_t vmc_L;
extern RM_Remote_t *remote;
extern PidTypeDef LegL_Pid;

vmc_t vmc_R;
chassis_t chassis_move;

PidTypeDef LegR_Pid;
PidTypeDef Turn_Pid;
PidTypeDef Tp_Pid;

float LQR_K_R[12] = {
    -15.7725, -1.4432, -2.8507, -2.8933, 9.5062, 2.1972,
    18.0773, 2.5909, 9.1851, 8.3496, 20.4557, 2.3536};

float Poly_Coefficient[12][4] = {

    263.788721, -71.164975, -59.109475, -1.306055,
    707.595152, -545.296757, 141.059093, 4.060010,
    74.124964, -46.185456, -0.400223, -0.179535,
    6.394145, -21.919128, 12.349682, 0.438991,
    -8.820482, 45.574070, -26.081728, -0.214925,
    287.228854, -133.506343, -2.704920, 11.155760,
    19.494578, 24.203106, -21.970126, -0.487006,
    221.879542, -103.957430, -3.092361, 10.058762,
    418.704944, -203.816299, -2.134386, 19.134460,
    113.728017, -404.364750, 227.436631, -3.781842,
    50.788800, -29.184018, 1.580652, 2.927516,
    -20.995350, -35.670321, 29.927415, -1.999610

};

first_order_filter_type_t d_phi1_r_filter;
first_order_filter_type_t d_phi4_r_filter;
first_order_filter_type_t pitch_angle_r_filter;
first_order_filter_type_t pitch_gyro_r_filter;

float d_phi1_r_filter_para[1] = {0.005};
float d_phi4_r_filter_para[1] = {0.005};
float pitch_angle_r_filter_para[1] = {0.005};
float pitch_gyro_r_filter_para[1] = {0.005};

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

      //						canSend_comd(RFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      //
      //            canSend_comd(RBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      wheel_Rctrl(-chassis_move.wheel_motor[1].T_wheel / 6.33f);
      //					wheel_Rctrl(0.0f);
      //					wheel_Rtest(Rtest);

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
  first_order_filter_init(d_phi1_r_filter, 0.002f, d_phi1_r_filter_para);
  first_order_filter_init(d_phi4_r_filter, 0.002f, d_phi4_r_filter_para);
  first_order_filter_init(pitch_angle_r_filter, 0.002f, pitch_angle_r_filter_para);
  first_order_filter_init(pitch_gyro_r_filter, 0.002f, pitch_gyro_r_filter_para);
}

void chassisR_update(chassis_t *chassis, vmc_t *vmc, INS_t *ins)
{
  vmc->phi4 = chassis->joint_motor[3].para.pos - 0.3925f + 0.0233f;
  vmc->phi1 = chassis->joint_motor[2].para.pos + pi + 0.3925f + 0.0233f;

  vmc->d_phi4 = chassis->joint_motor[3].para.vel;
  vmc->d_phi1 = chassis->joint_motor[2].para.vel;

  chassis->pitchR = -ins->Pitch;
  chassis->pitchGyroR = -ins->Gyro[1];

  chassis->total_yaw = ins->YawTotalAngle;
  chassis->roll = ins->Roll;

  chassis->theta_err = -vmc->theta - vmc_L.theta;

  first_order_filter_calc(&d_phi1_r_filter, vmc->d_phi1);
  first_order_filter_cali(&d_phi4_r_filter, vmc->d_phi4);
  first_order_filter_cali(&pitch_angle_r_filter, chassis->pitchR);
  first_order_filter_cali(&pitch_gyro_r_filter, chassis->pitchGyroR);

  // 把滤波后的值赋给vmc
  vmc->d_phi1 = d_phi1_r_filter.out;
  vmc->d_phi4 = d_phi4_r_filter.out;
  chassis->pitchR = pitch_angle_r_filter.out;
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

  chassis->wheel_motor[1].T_wheel = (K[0] * (vmc->theta - 0.0f) + K[1] * (vmc->d_theta - 0.0f) + K[2] * (chassis->x - chassis->x_set) + K[3] * (chassis->v - chassis->v_set) + K[4] * (chassis->pitchR - 0.0f) + K[5] * (chassis->pitchGyroR - 0.0f));
  vmc->Tp = (K[6] * (vmc->theta - 0.0f) + K[7] * (vmc->d_theta - 0.0f) + K[8] * (chassis->x - chassis->x_set) + K[9] * (chassis->v - chassis->v_set) + K[10] * (chassis->pitchR - 0.0f) + K[11] * (chassis->pitchGyroR - 0.0f));
  chassis->wheel_motor[1].T_wheel = chassis->wheel_motor[1].T_wheel - chassis->T_turn;
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
