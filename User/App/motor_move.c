#include "motor_move.h"
#include "main.h"
#include "usart.h"
#include "freeRTOS.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "chassisRtask.h"
#include "chassisLtask.h"

 #define Nor_printf(...)  HAL_UART_Transmit(&huart1,\
 																				(uint8_t  *)u1_buf,\
 																				sprintf((char*)u1_buf,__VA_ARGS__),0xff)

 uint8_t u1_buf[80];

extern chassis_t chassis_move;
extern vmc_t vmc_R;
extern vmc_t vmc_L;
extern first_order_filter_type_t  wheel_l_speed_filter;
extern first_order_filter_type_t  wheel_r_speed_filter;
extern first_order_filter_type_t  LFjoint_motor_filter;
extern first_order_filter_type_t  LBjoint_motor_filter;

extern INS_t INS;																				
																				
double flag=0;
void motorMoveTask(void *argument)
{
//	  canSend_mode(CMD_MOTOR_MODE);
//    canSend_comd(0, 5, 0, 1, 0);
//		osDelay(1);
		//canSend_mode(CMD_MOTOR_MODE);
    while(1)
		{

    //  set_mode(1);
  //    wheel_Lctrl(cflag);
//      wheel_Rctrl(cflag);
//        set_mode(1);  
//				if(cflag==1)
//						canSend_mode(CMD_MOTOR_MODE);
//				if(cflag==10)
//						canSend_mode(CMD_RESET_MODE);
						//canSend_comd(0, 5, 0, 1, 0);

        //Joint_Control(cflag);
//			set_mode(1);
//			test_Motor_test(cflag);
//			

//				Nor_printf("%f,%f,%f,%f,%f,%f\r\n",chassis_move.wheel_motor[0].T_wheel/6.33f,chassis_move.wheel_motor[1].T_wheel/6.33f,vmc_L.T_set[0]/6.0f,vmc_L.T_set[1]/6.0f,vmc_R.T_set[0]/6.0f,vmc_R.T_set[1]/6.0f);
//			Nor_printf("%f, %f\r\n",chassis_move.wheel_motor[0].para.torque, chassis_move.wheel_motor[0].para.speed);
			if(flag == 0)
				Nor_printf("%f, %f\r\n",vmc_L.d_phi0, vmc_L.phi0);
			else if(flag == 1)
				Nor_printf("%f, %f\r\n",vmc_L.L0, vmc_L.d_L0);
			else if(flag == 2)	
				Nor_printf("%f, %f\r\n",vmc_L.phi2, vmc_L.phi3);
			else if(flag == 3)	
				Nor_printf("%f, %f\r\n",vmc_L.d_phi2, vmc_L.phi2);
			else if(flag == 4)		
				Nor_printf("%f, %f\r\n",vmc_L.phi1, vmc_L.d_phi1);
			else if(flag == 5)		
				Nor_printf("%f, %f\r\n",vmc_L.phi4, vmc_L.d_phi4);
			else if(flag == 6)		
				Nor_printf("%f, %f\r\n",vmc_L.theta, vmc_L.d_theta);
			else if(flag == 106)		
				Nor_printf("%f, %f\r\n",vmc_R.theta, vmc_R.d_theta);
			else if(flag == 7)		
				Nor_printf("%f, %f\r\n",chassis_move.pitchL, chassis_move.pitchGyroL);
			else if(flag == 8)		
				Nor_printf("%f, %f, %f\r\n", INS.Gyro[0], INS.Gyro[1], INS.Gyro[2]);
//			else if(flag == 8)		
//				Nor_printf("%f, %f, %f\r\n", INS. INS.Pitch, INS.Roll);
			
        osDelay(1);
    }
}