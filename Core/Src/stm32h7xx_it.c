/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GO1_drv.h"
#include "chassisRtask.h"
#include "chassisLtask.h"
#include "bsp_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern chassis_t chassis_move;

volatile uint8_t rx_len = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
uint8_t tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //Ëé∑ÂèñIDLEÁä∂ÊÄÅ
  if((tmp_flag != RESET))//Âà§Êñ≠Êé•Êî∂ÊòØÂê¶ÁªìÊùü
    { 
      // recv_end_flag = 1; //Êé•Êî∂ÁªìÊùü
       __HAL_UART_CLEAR_IDLEFLAG(&huart1);//Ê∏ÖÊ•öÊ†áÂøó‰Ωç
       //HAL_UART_DMAStop(&huart1); 
			 __HAL_DMA_DISABLE(&hdma_usart1_rx);
       uint8_t temp=__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);                

       rx_len =100-temp; 

	     //HAL_UART_Transmit_DMA(&huart1, rx_buffer,rx_len);//ÂèëÈÄÅÊï∞ÊçÆ
			 //for(uint8_t i=0;i<100;i++)	rx_buffer[i]=0;			
//			RC_Data_Handle();//ÈÅ•ÊéßÂô®Êï∞ÊçÆËß£ÁÆó
//			
//			GM6020ID6_Speed_Ref = rc.ch4 * 0.3f;
			 //DMA1_Stream0->NDTR = 100;
			 __HAL_DMA_ENABLE(&hdma_usart1_rx);			
       HAL_UART_Receive_DMA(&huart1,rx_buffer,100);//ÂºÄÂêØDMA
		}	
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  uint8_t tmp_flag =__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); //ªÒ»°IDLE◊¥Ã¨
  if((tmp_flag != RESET))//≈–∂œΩ” ’ «∑ÒΩ· ¯
    { 
       __HAL_UART_CLEAR_IDLEFLAG(&huart2);//«Â≥˝IDLE±Í÷æ
       HAL_UART_DMAStop(&huart2); 
       uint8_t temp=__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);                

       Usart2Type.RX_Size =16-temp;

       if(Usart2Type.RX_Size == 16){
          SCB_InvalidateDCache_by_Addr(Usart2Type.RX_pData, 16);
          memcpy(&chassis_move.wheel_motor[0].para, Usart2Type.RX_pData, 16);
				  extract_data(wheel_Lsend.id, &chassis_move.wheel_motor[0].para);
          Usart2Type.RX_flag = 1;
        }

        HAL_UART_Receive_DMA(&huart2, Usart2Type.RX_pData, 16);
    } 
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  uint8_t tmp_flag =__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); //ªÒ»°IDLE◊¥Ã¨
  if((tmp_flag != RESET))//≈–∂œΩ” ’ «∑ÒΩ· ¯
    { 
       __HAL_UART_CLEAR_IDLEFLAG(&huart3);//«Â≥˝IDLE±Í÷æ
       HAL_UART_DMAStop(&huart3); 
       uint8_t temp=__HAL_DMA_GET_COUNTER(&hdma_usart3_rx);                

       Usart3Type.RX_Size =16-temp;

       if(Usart3Type.RX_Size == 16){
          SCB_InvalidateDCache_by_Addr(Usart3Type.RX_pData, 16);
          memcpy(&chassis_move.wheel_motor[1].para, Usart3Type.RX_pData, 16);
          extract_data(wheel_Rsend.id, &chassis_move.wheel_motor[1].para);
          Usart3Type.RX_flag = 1;


        HAL_UART_Receive_DMA(&huart3, Usart3Type.RX_pData, 16);
    }
	}			 
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ACC_INT_Pin);
  HAL_GPIO_EXTI_IRQHandler(GYRO_INT_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
//if (huart5.Instance->SR & UART_FLAG_RXNE)//Ω” ’µΩ ˝æ›
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart5);
//    }
//    else if (UART5->ISR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart5);

//        if ((hdma_uart5_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            //disable DMA
//            // ß–ßDMA
//            __HAL_DMA_DISABLE(&hdma_uart5_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //ªÒ»°Ω” ’ ˝æ›≥§∂»,≥§∂» = …Ë∂®≥§∂» -  £”‡≥§∂»
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_uart5_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //÷ÿ–¬…Ë∂® ˝æ›≥§∂»
//            hdma_uart5_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//            //set memory buffer 1
//            //…Ë∂®ª∫≥Â«¯1
//            hdma_uart5_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            // πƒ‹DMA
//            __HAL_DMA_ENABLE(&hdma_uart5_rx);

//            if (this_time_rx_len == RC_FRAME_LENGTH)
//            {
//                //¥¶¿Ì“£øÿ∆˜ ˝æ›
//                sbus_to_rc(0);
//                //º«¬º ˝æ›Ω” ’ ±º‰
//                //RemoteOfflineCounterUpdate();
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            // ß–ßDMA
//            __HAL_DMA_DISABLE(&hdma_usart3_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //ªÒ»°Ω” ’ ˝æ›≥§∂»,≥§∂» = …Ë∂®≥§∂» -  £”‡≥§∂»
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_uart5_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //÷ÿ–¬…Ë∂® ˝æ›≥§∂»
//            hdma_uart5_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//            //set memory buffer 0
//            //…Ë∂®ª∫≥Â«¯0
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            // πƒ‹DMA
//            __HAL_DMA_ENABLE(&hdma_usart3_rx);

//            if (this_time_rx_len == RC_FRAME_LENGTH)
//            {
//                //¥¶¿Ì“£øÿ∆˜ ˝æ›
//                sbus_to_rc(1);
//                //º«¬º ˝æ›Ω” ’ ±º‰
//                //RemoteOfflineCounterUpdate();
//            }
//        }
//    }
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
