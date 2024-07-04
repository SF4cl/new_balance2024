#ifndef GO1_DRV_H
#define GO1_DRV_H

#include "motor_msg_GO-M8010-6.h"
#include "crc_ccitt.h"
#include "usart.h"
#include "string.h"
#include "stdbool.h"
		
extern MOTOR_send wheel_Lsend;
extern MOTOR_send wheel_Rsend;
extern MOTOR_recv wheel_Lrecv;
extern MOTOR_recv wheel_Rrecv;

extern void set_mode(int mode);
extern void wheel_Rctrl(double T_right);
extern void wheel_Lctrl(double T_left);
extern void modify_data(MOTOR_send *Motor_s);
extern bool extract_data(uint8_t id, MOTOR_recv *Motor_r);
extern void wheel_Rtest(double T_right);
extern void wheel_Ltest(double T_left);

typedef struct
{
	uint8_t RX_flag:1;//IDLE receive flag

	uint16_t RX_Size;//receive length

	uint8_t RX_pData[16];//DMA receive buffer

	uint8_t TX_pData[17];
}USART_SENDRECEIVETYPE;

typedef struct{
	RIS_Mode_t mode;
	float T_wheel;
	MOTOR_recv para;
}Wheel_Motor_t;

extern USART_SENDRECEIVETYPE Usart2Type;
extern USART_SENDRECEIVETYPE Usart3Type;
extern uint8_t rx_buffer[50];

#endif