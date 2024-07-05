#include "observeTask.h"
#include "kalman_filter.h"
#include "cmsis_os.h"

extern INS_t INS;
extern chassis_t chassis_move;
extern vmc_t vmc_L;
extern vmc_t vmc_R;

KalmanFilter_t vEstimateKF;

first_order_filter_type_t wheel_r_speed_filter;
first_order_filter_type_t wheel_l_speed_filter;

float wheel_r_speed_filter_para[1] = {0.01f};
float wheel_l_speed_filter_para[1] = {0.01f};

float vEstimateKF_F[4] = {1.0f, 0.002f,
                          0.0f, 1.0f};

float vEstimateKF_P[4] = {1.0f, 0.0f,
                          0.0f, 1.0f};

float vEstimateKF_Q[4] = {0.1f, 0.0f,
                          0.0f, 0.1f};

float vEstimateKF_R[4] = {100.0f, 0.0f,
                          0.0f, 100.0f};

const float vEstimateKF_H[4] = {1.0f, 0.0f,
                                0.0f, 1.0f};

float vEstimateKF_K;

void observeTask(void)
{
    while (INS.ins_flag == 0)
    {
        osDelay(1);
    }

    static float wr, wl = 0.0f;
    static float vrb, vlb = 0.0f;
    static float ave_v = 0.0f;

    first_order_filter_init(&wheel_r_speed_filter, 0.002, wheel_r_speed_filter_para);
    first_order_filter_init(&wheel_l_speed_filter, 0.002, wheel_l_speed_filter_para);
    while (1)
    {
        first_order_filter_cali(&wheel_r_speed_filter, chassis_move.wheel_motor[1].para.speed);
        first_order_filter_cali(&wheel_l_speed_filter, chassis_move.wheel_motor[0].para.speed);

        chassis_move.v = -(wheel_r_speed_filter.out / 6.33f - wheel_l_speed_filter.out / 6.33f) / 2.0f * 0.1f;
        chassis_move.x = chassis_move.x + chassis_move.v * 2.0f / 1000.0f;

        osDelay(2);
    }
}
