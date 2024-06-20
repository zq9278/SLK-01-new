/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g0xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "led.h"
#include "bq25895.h"
#include "apply.h"
#include "screen.h"
#include "tmp114.h"
#include "pid.h"
#include "heat.h"
#include "tmc5130.h"
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
uint8_t SW_CNT_Flag;
u16 Tim6Cnt, Tim7Cnt;
u8 Flag_3200ms, Flag_800ms, Flag_600ms, Flag_400ms, Flag_200ms, Flag_100ms;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_tim16_ch1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern s32 ForceRawSet;    // 屏幕设定的�?
extern s32 ForceRawSetAct; // 实时设定�?
extern s32 ForceRawActual; // 应变片实际�?
extern u8 WorkMode;

extern u8 PowerState;

extern u8 MotorState;
extern u8 MotorCompareState;

extern u8 LEDNub;
extern u16 LEDPWMVal;
extern u8 LEDDir;
extern u8 LEDState;
extern u32 RGB_DATA;
extern u32 MotorSpeed;

extern u8 USART1_RX_STA; // 接收状态标�?
extern u8 USART1_RX_BUF[];
u8 frame_started = 0; // �?否�?�测到帧头
u8 rx_index      = 0;
u8 last_byte     = 0;

extern u8 EyeTmpRaw[2];
extern float EyeTmp;

extern u8 HeatPWMVal;

extern PID_typedef HeatPID;
extern TIM_HandleTypeDef htim16;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
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
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVC_IRQn 0 */

    /* USER CODE END SVC_IRQn 0 */
    /* USER CODE BEGIN SVC_IRQn 1 */

    /* USER CODE END SVC_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line 2 and line 3 interrupts.
 */
void EXTI2_3_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI2_3_IRQn 0 */

    /* USER CODE END EXTI2_3_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(SW_CNT_Pin);
    /* USER CODE BEGIN EXTI2_3_IRQn 1 */

    /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel 1 interrupt.
 */
void DMA1_Channel1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

    /* USER CODE END DMA1_Channel1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_i2c1_rx);
    /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

    /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
 */
void DMA1_Channel2_3_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

    /* USER CODE END DMA1_Channel2_3_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_i2c1_tx);
    HAL_DMA_IRQHandler(&hdma_i2c2_rx);
    /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

    /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel 4, channel 5, channel 6, channel 7 and DMAMUX1 interrupts.
 */
void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */

    /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_i2c2_tx);
    HAL_DMA_IRQHandler(&hdma_adc1);
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
    HAL_DMA_IRQHandler(&hdma_tim16_ch1);
    /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */

    /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */
}

/**
 * @brief This function handles ADC1 interrupt.
 */
void ADC1_IRQHandler(void)
{
    /* USER CODE BEGIN ADC1_IRQn 0 */

    /* USER CODE END ADC1_IRQn 0 */
    HAL_ADC_IRQHandler(&hadc1);
    /* USER CODE BEGIN ADC1_IRQn 1 */

    /* USER CODE END ADC1_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt.
 */
void TIM6_IRQHandler(void)
{
    /* USER CODE BEGIN TIM6_IRQn 0 */

    /* USER CODE END TIM6_IRQn 0 */
    HAL_TIM_IRQHandler(&htim6);
    /* USER CODE BEGIN TIM6_IRQn 1 */

    /* USER CODE END TIM6_IRQn 1 */
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
void TIM7_IRQHandler(void)
{
    /* USER CODE BEGIN TIM7_IRQn 0 */

    /* USER CODE END TIM7_IRQn 0 */
    HAL_TIM_IRQHandler(&htim7);
    /* USER CODE BEGIN TIM7_IRQn 1 */

    /* USER CODE END TIM7_IRQn 1 */
}

/**
 * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
 */
void I2C1_IRQHandler(void)
{
    /* USER CODE BEGIN I2C1_IRQn 0 */

    /* USER CODE END I2C1_IRQn 0 */
    if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
        HAL_I2C_ER_IRQHandler(&hi2c1);
    } else {
        HAL_I2C_EV_IRQHandler(&hi2c1);
    }
    /* USER CODE BEGIN I2C1_IRQn 1 */

    /* USER CODE END I2C1_IRQn 1 */
}

/**
 * @brief This function handles I2C2 global interrupt.
 */
void I2C2_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_IRQn 0 */

    /* USER CODE END I2C2_IRQn 0 */
    if (hi2c2.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
        HAL_I2C_ER_IRQHandler(&hi2c2);
    } else {
        HAL_I2C_EV_IRQHandler(&hi2c2);
    }
    /* USER CODE BEGIN I2C2_IRQn 1 */

    /* USER CODE END I2C2_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    // u8 Res;  // 定义一�?8位的变量Res用于存储接收到的数据

    // // 检查是否收到数�?（接收非空标志位�?
    // if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE)!=RESET))
    // {
    //     // 从UART接收一�?字节的数�?到Res，并设置超时时间�?1000ms
    //     HAL_UART_Receive(&huart1,&Res,1,1000);

    //     // 检查接收状态，判断�?否接收未完成
    //     if((USART1_RX_STA&0x8000)==0)
    //     {
    //         // 如果接收到了0x0d，即回车�?
    //         if(USART1_RX_STA&0x4000)
    //         {
    //             if(USART1_RX_STA&0x2000)
    //             {
    //                 // 简化的代码，具体的�?注释掉的代码块在这里处理特定的情�?

    //                     // 将接收到的数�?存储到USART1_RX_BUF�?，并更新接收状�?
    //                     USART1_RX_BUF[USART1_RX_STA&0X1FFF]=Res ;
    //                     USART1_RX_STA++;
    //                     // 如果接收到特定长度的数据，则设置接收完成标志
    //                     if((USART1_RX_STA & 0x1fff)==0x07)
    //                     {
    //                         USART1_RX_STA|=0x8000;
    //                     }

    //             }
    //             // 特定的情况�?�理
    //             if(Res==0xa5 && USART1_RX_STA==0x4000)
    //             {
    //                 USART1_RX_STA|=0x2000;
    //             }
    //         }
    //         // 如果接收到特定的起�?�字�?0x5a，并且当前状态为0
    //         if(Res==0x5a && USART1_RX_STA==0)
    //         {
    //             USART1_RX_STA=0x4000;
    //         }
    //     }
    // }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) // 非空�?�?
    {
        uint8_t received_data = (uint8_t)(huart1.Instance->RDR & 0xFF); // 取出缓冲区的字�??
        if (last_byte == 0x5A && received_data == 0xA5) {
            frame_started    = 1; // �???测到帧头时�?�置�?--标志�???
            USART1_RX_BUF[0] = 0x5A;
            rx_index         = 1; // 重置索引
        }
        last_byte = received_data; // 更新上一�?字�??
        if (frame_started) {
            USART1_RX_BUF[rx_index++] = received_data;
        }
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) // 空闲�?�?
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        // HAL_UART_Transmit(&huart1, (uint8_t *)&rx_buffer, (size_t)rx_index, 0xFFFF); // 验证打印数据
        USART1_RX_STA = 1;
        // rx_index = 0;                                            //
        frame_started = 0; // 标�?�帧的结�?
    }
    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    // 检查并清除溢出错�??标志�?
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
    // 检查并清除帧错�?标志�?
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE) != RESET)) {
        __HAL_UART_CLEAR_FEFLAG(&huart1);
    }

    //					if((USART1_RX_STA & 0x3fff)==0x05)
    //					{
    //						if(Res==0xaa)
    //						{
    //							USART1_RX_STA|=0x8000;
    //						}
    //						else
    //						{
    //							USART1_RX_STA=0;
    //						}
    //					}
    //					else
    /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (KEY1 == 0) SW_CNT_Flag = 1;
}

/**/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) // 50ms定时�?
    {
        Tim6Cnt++;
        switch (MotorState) {
                // �?一阶�?�，电机�?动到挤压力到1/2为�??
                //  case 0:
                //  	ForceRawSetAct=ForceRawSet>>1;// /2
                //  	if(MotorCompareState==0 && Tim6Cnt>10)
                //  	{
                //  		MotorSpeed=0x6000;
                //  		MotorState=1;
                //  		Tim6Cnt=0;
                //  	}
                //  	break;
                // �?二阶段，等待1s

            case 0:
                if (Tim6Cnt >= 0) {
                    // TMC5130_Write(0xa7, MotorSpeed);
                    // TMC5130_Write(0xa0, 1);
                    ForceRawSetAct = ForceRawSet;
                    MotorState     = 1;
                    Tim6Cnt        = 0;
                }
                break;
            // �?三阶段，上升到�?��?�挤压力
            case 1:
                if ((MotorCompareState==0)&&(Tim6Cnt > 40)) // MotorCompareState==0 &&
                {
                    MotorState = 2;
                    ForceRawSetAct = 0;
                    // TMC5130_Write(0xa7, MotorSpeed);
                    // TMC5130_Write(0xa0, 2);
                    
                    Tim6Cnt = 0;
                }
                break;
            // �?四阶段，预�?�挤压力等待2s
            case 2:
                if (Tim6Cnt >= 20) {
                    // ForceRawSetAct=ForceRawSet>>1;
                    MotorState = 0;
                    Tim6Cnt    = 0;
                }
                break;
                // 	case 3:
                // if(Tim6Cnt>=40)
                // {
                // 	ForceRawSetAct=ForceRawSet>>1;
                // 	MotorState=4;
                // 	Tim6Cnt=0;
                // }
                // break;
            // //�?五阶段，下降到挤压力一�?
            // case 4:
            // 	if(MotorCompareState==0 && Tim6Cnt>5)
            // 	{
            // 		MotorState=5;
            // 		Tim6Cnt=0;
            // 	}
            // 	break;
            // //�?�?阶�?�，挤压力的一半等�?1s
            // case 5:
            // 	if(Tim6Cnt>=20)
            // 	{
            // 		MotorState=6;
            // 		Tim6Cnt=0;
            // 	}
            // 	break;
            // //�?七阶段，挤压力从一半下降到0
            // case 6:
            // 	ForceRawSetAct=(ForceRawSet>>1)-(ForceRawSet>>1)/20*Tim6Cnt;
            // 	if(Tim6Cnt>=20)
            // 	{
            // 		ForceRawSetAct=0;
            // 		MotorState=7;
            // 		Tim6Cnt=0;
            // 	}
            // 	break;
            // //�?�?阶�?�，挤压力在0等待1s
            // case 7:
            // 	if(Tim6Cnt>=20)
            // 	{
            // 		MotorState=8;
            // 		Tim6Cnt=0;
            // 	}
            // 	break;
            // //�?九阶段，挤压力从0上升到一�?
            // case 8:
            // 	ForceRawSetAct=(ForceRawSet>>1)/20*Tim6Cnt;
            // 	if(Tim6Cnt>=20)
            // 	{
            // 		ForceRawSetAct=ForceRawSet>>1;
            // 		MotorState=1;
            // 		Tim6Cnt=0;
            // 	}
            default:
                break;
        }
    }
    if (htim == &htim7) {
        Tim7Cnt++;
        if ((Tim7Cnt & 0x3f) == 0x3f) {
            Flag_3200ms = 1;
        }
        if ((Tim7Cnt & 0x0f) == 0x0f) {
            LED1Toggle;
            Flag_800ms = 1;
            // Flag_100ms=1;

        }
        if ((Tim7Cnt & 0x0b) == 0x0b) // 600ms 标志�?
        {
            Flag_600ms = 1;
        }
        if ((Tim7Cnt & 0x07) == 0x07) {
            Flag_400ms = 1;
        }
        if ((Tim7Cnt & 0x03) == 0x03) {
            Flag_200ms = 1;
        }
        if ((Tim7Cnt & 0x01) == 0x01) {
            Flag_100ms = 1;
           // if ((WorkMode & 0x05) == 0x05) {
            if  ((WorkMode == 5) || (WorkMode == 1)|| (WorkMode == 7) || (WorkMode == 3)) {
                TMP114_Read(0x00, EyeTmpRaw);
            }
            
           // if ((WorkMode & 0x05) == 0x05) {
            if ((WorkMode == 5) || (WorkMode == 1)|| (WorkMode == 7) || (WorkMode == 3)){
                EyeTmp     = TmpRaw2Ture(EyeTmpRaw);
                HeatPWMVal = PID_realize(&HeatPID, EyeTmp);
                HeatPWMSet(HeatPWMVal);
            }
        }
        if (PowerState == 1) {
            if (LEDDir == 0) {
                RGB_DATA += 5;
                if (RGB_DATA >= 255) {
                    LEDDir = 1;
                }
            } else {
                RGB_DATA -= 5;
                if (RGB_DATA == 0) {
                    LEDDir = 0;
                }
            }
            //			LEDPWMSet(LEDNub,LEDPWMVal);
        }
    }
}

/* USER CODE END 1 */
