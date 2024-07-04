#ifndef __MOTOR_MOVE_H
#define __MOTOR_MOVE_H

#include <stdint.h>
#include <stddef.h>
#include "GO1_drv.h"
#include "motor_msg_GO-M8010-6.h"
#include "crc_ccitt.h"
#include "stm32h7xx_it.h"
#include "struct_typedef.h"
#include "main.h"
#include "usart.h"
#include "freeRTOS.h"
#include "HT04_drv.h"

void motorMoveTask(void *argument);

#endif
