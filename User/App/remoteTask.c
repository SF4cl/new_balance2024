#include "remoteTask.h"
#include "cmsis_os.h"

RM_Remote_t *remote;

extern chassis_t chassis_move;

void remoteTask(void)
{
    RM_Remote_Register_t remote_register;
    remote_register.usart_handle = &huart5;
    remote = rmRemoteAdd(&remote_register);

    while (1)
    {
        remote_process(remote, &chassis_move, 0.01f);

        osDelay(10);
    }
}

void remote_process(RM_Remote_t *remote, chassis_t *chassis, float dt)
{
    if (remote->state_interfaces.rc.s[1] == 2)
    {
        chassis->start_flag = 0;
        chassis->recover_flag = 0;
    }
    else if (remote->state_interfaces.rc.s[1] == 3)
    {
        chassis->start_flag = 1;
        if ((chassis->recover_flag == 0 && (chassis->pitchR < (-pi / 12.0f) && chassis->pitchR > (-pi / 2.0f))) || (chassis->pitchR > (pi / 12.0f) && chassis->pitchR < (pi / 2.0f)))
        {
            chassis->recover_flag = 1;
        }
    }

    if (chassis->start_flag == 1)
    {
        chassis->v_set = -remote->state_interfaces.rc.ch[1] / 660.0f / 2.0f;
        chassis->x_set = chassis->x_set + chassis->v_set * dt;
        chassis->turn_set = chassis->turn_set + remote->state_interfaces.rc.ch[2] / 660.0f;

        chassis->leg_set = chassis->leg_set + remote->state_interfaces.rc.ch[3] / 1500.0f;
        limit(&chassis->leg_set, 0.10f, 0.26f);

        chassis->last_leg_set = chassis->leg_set;
    }

    else if (chassis->start_flag == 0)
    {
        chassis->v_set = 0.0f;
        chassis->x_set = chassis->x;
        chassis->turn_set = chassis->total_yaw;
        chassis->leg_set = 0.15f;
    }
}
