#include "bsp_can.h"
#include "fdcan.h"
#include "HT04_drv.h"
#include "string.h"
#include "chassisRtask.h"

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0000;
  sFilterConfig.FilterID2 = 0x0000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

extern chassis_t chassis_move;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint16_t tmp_value;

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
            Error_Handler();
        }
        if(hfdcan == &hfdcan1)
				{
						if(RxHeader.Identifier==0x000)
						{
								switch(RxData[0])
								{
										case LFmotor:
												tmp_value = (RxData[3]<<4)|(RxData[4]>>4);
												chassis_move.joint_motor[0].para.vel = uint_to_float(tmp_value, V_MIN, V_MAX, 12);

												tmp_value = (RxData[1]<<4)|(RxData[2]>>4);
												chassis_move.joint_motor[0].para.pos = uint_to_float(tmp_value, P_MIN, P_MAX, 12);	

                        tmp_value = (RxData[4]<<4|RxData[5]>>8);
                        chassis_move.joint_motor[0].para.tor = uint_to_float(tmp_value, T_MIN, T_MAX, 12);								
												break;
										case LBmotor:
												tmp_value = (RxData[3]<<4)|(RxData[4]>>4);
												chassis_move.joint_motor[1].para.vel = uint_to_float(tmp_value, V_MIN, V_MAX, 12);

												tmp_value = (RxData[1]<<4)|(RxData[2]>>4);
												chassis_move.joint_motor[1].para.pos = uint_to_float(tmp_value, P_MIN, P_MAX, 12);	

                        tmp_value = (RxData[4]<<4|RxData[5]>>8);
                        chassis_move.joint_motor[1].para.tor = uint_to_float(tmp_value, T_MIN, T_MAX, 12);		
												break;
                    case RBmotor:
												tmp_value = (RxData[3]<<4)|(RxData[4]>>4);
												chassis_move.joint_motor[2].para.vel = uint_to_float(tmp_value, V_MIN, V_MAX, 12);

												tmp_value = (RxData[1]<<4)|(RxData[2]>>4);
												chassis_move.joint_motor[2].para.pos = uint_to_float(tmp_value, P_MIN, P_MAX, 12);	

                        tmp_value = (RxData[4]<<4|RxData[5]>>8);
                        chassis_move.joint_motor[2].para.tor = uint_to_float(tmp_value, T_MIN, T_MAX, 12);								
												break;
                    case RFmotor:
												tmp_value = (RxData[3]<<4)|(RxData[4]>>4);
												chassis_move.joint_motor[3].para.vel = uint_to_float(tmp_value, V_MIN, V_MAX, 12);

												tmp_value = (RxData[1]<<4)|(RxData[2]>>4);
												chassis_move.joint_motor[3].para.pos = uint_to_float(tmp_value, P_MIN, P_MAX, 12);	

                        tmp_value = (RxData[4]<<4|RxData[5]>>8);
                        chassis_move.joint_motor[3].para.tor = uint_to_float(tmp_value, T_MIN, T_MAX, 12);								
												break;
										default:
												break;
								}
						}
        }
    }
}
