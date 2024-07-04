#include "HT04_drv.h"

#include "fdcan.h"
#include <math.h>

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

FDCAN_TxHeaderTypeDef can_tx_message;
uint8_t MotorSendBuffer[8];

/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval                         
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void canSend_comd(uint8_t id, float f_p, float f_v, float f_kp, float f_kd, float f_t){
    uint16_t p, v, kp, kd, t;

    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);

    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);

    MotorSendBuffer[0] = p>>8;
    MotorSendBuffer[1] = p&0xFF;
    MotorSendBuffer[2] = v>>4;
    MotorSendBuffer[3] = ((v&0xF)<<4)|(kp>>8);
    MotorSendBuffer[4] = kp&0xFF;
    MotorSendBuffer[5] = kd>>4;
    MotorSendBuffer[6] = ((kd&0xF)<<4)|(t>>8);
    MotorSendBuffer[7] = t&0xff;

    canTransmit(id, MotorSendBuffer, sizeof(MotorSendBuffer));
}
uint8_t send_buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
void canSend_mode(uint8_t id, uint8_t mode){
    

    switch(mode){
        case CMD_MOTOR_MODE:
            send_buf[7] = 0xFC;
            break;
        case CMD_RESET_MODE:
            send_buf[7] = 0xFD;
            break;
        case CMD_ZERO_POSITION:
            send_buf[7] = 0xFE;
            break;
        default:
            return;
    }
   
    canTransmit(id, send_buf, sizeof(send_buf));
}

static void canTransmit(uint8_t id, uint8_t *buf, uint8_t len){
    can_tx_message.Identifier = id;
    can_tx_message.IdType = FDCAN_STANDARD_ID;
    can_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    can_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    can_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    can_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    can_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    can_tx_message.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_message, buf);
}

