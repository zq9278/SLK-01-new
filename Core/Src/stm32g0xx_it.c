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
u8 Flag_3200ms, Flag_800ms, Flag_600ms, Flag_400ms, Flag_200ms, Flag_100ms, Flag_1s, Flag_2s,Flag_3s;
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
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern s32 ForceRawSet;    // 灞忓箷璁惧畾鐨勫??
extern s32 ForceRawSetAct; // 瀹炴椂璁惧畾鍊?
extern s32 ForceRawActual; // 搴斿彉鐗囧疄闄呭??
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

extern u8 USART1_RX_STA; // 鎺ユ敹鐘舵?佹爣璁?
extern u8 USART1_RX_BUF[];
u8 frame_started = 0; // 鏄?鍚︽??娴嬪埌甯уご
u8 rx_index      = 0;
u8 last_byte     = 0;

extern u8 EyeTmpRaw[2];
extern float EyeTmp;

extern u8 HeatPWMVal;

extern PID_TypeDef HeatPID;
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
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
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

    HAL_UART_IRQHandler(&huart1);
 
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (KEY1 == 0) SW_CNT_Flag = 1;
}

/**/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) // 500ms定时器
    {
        Tim6Cnt++;
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); 
        // 检查计数器以实现不同的定时事件
        if (Tim6Cnt % 1 == 0) {
            // 每1秒执行一次
            // 1秒事件代码
            //Flag_500ms = 1;
        }
        if (Tim6Cnt % 3 == 0) {
            // 每1秒执行一次
            // 1秒事件代码
            Flag_1s = 1;
             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); 
        }
        if (Tim6Cnt % 4 == 0) {
            // 每2秒执行一次
            // 2秒事件代码
            Flag_2s = 1;
            
        }
        if (Tim6Cnt % 6 == 0) {
            // 每3秒执行一次
            // 3秒事件代码
            // 为了防止计数器溢出，每6次复位一次
             Flag_3s = 1;
            Tim6Cnt = 0;
        }
    }

    if (htim == &htim7) {
        Tim7Cnt++;
        if ((Tim7Cnt & 0x3f) == 0x3f) {
            Flag_3200ms = 1;
        }
        if ((Tim7Cnt & 0x0f) == 0x0f) {
            //LED1Toggle;//LED1
            Flag_800ms = 1;
            // Flag_100ms=1;
        }
        if ((Tim7Cnt & 0x0b) == 0x0b) // 600ms 鏍囧織浣?
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
            if ((WorkMode == 5) || (WorkMode == 1) || (WorkMode == 7) || (WorkMode == 3)) {
                TMP114_Read(0x00, EyeTmpRaw);
            }

            // if ((WorkMode & 0x05) == 0x05) {
            if ((WorkMode == 5) || (WorkMode == 1) || (WorkMode == 7) || (WorkMode == 3)) {
                EyeTmp = TmpRaw2Ture(EyeTmpRaw);
                // HeatPWMVal = PID_realize(&HeatPID, EyeTmp);
                HeatPWMVal = PID_Compute(&HeatPID, EyeTmp, HeatPID.setpoint);
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
