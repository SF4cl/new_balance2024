#include "GO1_drv.h"

USART_SENDRECEIVETYPE Usart2Type __attribute__((section(".ARM.__at_0x24000000")));
USART_SENDRECEIVETYPE Usart3Type __attribute__((section(".ARM.__at_0x24000000")));
uint8_t rx_buffer[50] __attribute__((section(".ARM.__at_0x24000000")));

/*
MOTOR_send Motor_test;
MOTOR_recv Motor_recv;

void set_mode(int mode){
    Motor_test.mode = mode;
    Motor_test.id = 3;
}

void test_Motor_test(double T){
    Motor_test.torque = 0;
    Motor_test.speed = T;
    Motor_test.pos = 0;
    Motor_test.k_pos = 0;
    Motor_test.k_spd = 0.02;
    modify_data(&Motor_test);
    memcpy(Usart2Type.TX_pData, (uint8_t *)&(Motor_test.motor_send_data), 17);
    SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart2Type.TX_pData), 17);
    HAL_UART_Transmit_DMA(&huart2, Usart2Type.TX_pData, 17);
}
*/

MOTOR_send wheel_Lsend;
MOTOR_send wheel_Rsend;
MOTOR_recv wheel_Lrecv;
MOTOR_recv wheel_Rrecv;

void set_mode(int mode){
    wheel_Lsend.mode = mode;
    wheel_Lsend.id = 3;
    wheel_Rsend.mode = mode;
    wheel_Rsend.id = 4;
}

void wheel_Lctrl(double T_left){
    wheel_Lsend.torque = T_left;
    wheel_Lsend.speed = 0;
    wheel_Lsend.pos = 0;
    wheel_Lsend.k_pos = 0;
    wheel_Lsend.k_spd = 0;
    modify_data(&wheel_Lsend);
	  SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart2Type.TX_pData), 17);
    memcpy(Usart2Type.TX_pData, (uint8_t *)&(wheel_Lsend.motor_send_data), 17);
    //SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart2Type.TX_pData), 17);
    HAL_UART_Transmit_DMA(&huart2, Usart2Type.TX_pData, 17);
	//HAL_UART_Transmit(&huart2, Usart2Type.TX_pData, 17, 0xff);
}

void wheel_Rctrl(double T_right){
    wheel_Rsend.torque = T_right;
    wheel_Rsend.speed = 0;
    wheel_Rsend.pos = 0;
    wheel_Rsend.k_pos = 0;
    wheel_Rsend.k_spd = 0;
    modify_data(&wheel_Rsend);
	  SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart3Type.TX_pData), 17);
    memcpy(Usart3Type.TX_pData, (uint8_t *)&(wheel_Rsend.motor_send_data), 17);
    //SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart3Type.TX_pData), 17);
    HAL_UART_Transmit_DMA(&huart3, Usart3Type.TX_pData, 17);
	//HAL_UART_Transmit(&huart3, Usart3Type.TX_pData, 17, 0xff);
}

void wheel_Rtest(double T_right){
    wheel_Rsend.torque = 0;
    wheel_Rsend.speed = T_right;
    wheel_Rsend.pos = 0;
    wheel_Rsend.k_pos = 0;
    wheel_Rsend.k_spd = 0.02;
    modify_data(&wheel_Rsend);
	  SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart3Type.TX_pData), 17);
    memcpy(Usart3Type.TX_pData, (uint8_t *)&(wheel_Rsend.motor_send_data), 17);
    //SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart3Type.TX_pData), 17);
    HAL_UART_Transmit_DMA(&huart3, Usart3Type.TX_pData, 17);
	//HAL_UART_Transmit(&huart3, Usart3Type.TX_pData, 17, 0xff);
}

void wheel_Ltest(double T_left){
    wheel_Lsend.torque = 0;
    wheel_Lsend.speed = T_left;
    wheel_Lsend.pos = 0;
    wheel_Lsend.k_pos = 0;
    wheel_Lsend.k_spd = 0.02;
    modify_data(&wheel_Lsend);
	  SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart2Type.TX_pData), 17);
    memcpy(Usart2Type.TX_pData, (uint8_t *)&(wheel_Lsend.motor_send_data), 17);
    //SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart2Type.TX_pData), 17);
    HAL_UART_Transmit_DMA(&huart2, Usart2Type.TX_pData, 17);
	//HAL_UART_Transmit(&huart2, Usart2Type.TX_pData, 17, 0xff);
}

void modify_data(MOTOR_send *Motor_s){
    Motor_s->motor_send_data.head[0] = 0xFE;
    Motor_s->motor_send_data.head[1] = 0xEE;
    Motor_s->motor_send_data.mode.id = Motor_s->id;
    Motor_s->motor_send_data.mode.status = Motor_s->mode;
    Motor_s->motor_send_data.mode.none = 0x1;
    Motor_s->motor_send_data.comd.tor_des = Motor_s->torque*256;
    Motor_s->motor_send_data.comd.spd_des = Motor_s->speed/6.28*256;
    Motor_s->motor_send_data.comd.pos_des = Motor_s ->pos/6.2831f*32768.0f;
    Motor_s->motor_send_data.comd.k_pos = Motor_s->k_pos*1280;
    Motor_s->motor_send_data.comd.k_spd = Motor_s->k_spd*1280;
	
    Motor_s->motor_send_data.CRC16 = crc_ccitt(0x0, (uint8_t *)&Motor_s->motor_send_data, 15);
}

bool extract_data(uint8_t id, MOTOR_recv *Motor_r){
    //if(Motor_r->motor_recv_data.head[0] == 0xFD && Motor_r->motor_recv_data.head[1] == 0xEE){
        if(Motor_r->motor_recv_data.mode.id == id){
            Motor_r->id = Motor_r->motor_recv_data.mode.id;
            Motor_r->mode = Motor_r->motor_recv_data.mode.status;
            Motor_r->torque = Motor_r->motor_recv_data.fbk.torque/256.0f;
            Motor_r->speed = Motor_r->motor_recv_data.fbk.speed*2.0f*3.1415926f/256.0f;
            Motor_r->pos = Motor_r->motor_recv_data.fbk.pos*2.0f*3.1415926f/32768.0f;
            Motor_r->temp = Motor_r->motor_recv_data.fbk.temp;
            Motor_r->MError = Motor_r->motor_recv_data.fbk.MError;
            Motor_r->force = Motor_r->motor_recv_data.fbk.force;
            Motor_r->correct = true;           
        }
        return Motor_r->correct;
    //}
}   
