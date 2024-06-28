/* USER CODE BEGIN Header */
/**
ddsa
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "tmc5130.h"
#include "hx711.h"
#include "bq25895.h"
#include "apply.h"
#include "led.h"
#include "bq27441.h"
#include "screen.h"
#include "tmp114.h"
#include "heat.h"
#include "pid.h"
#include "24cxx.h"
#include "ws2812b.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
u8 PWM_SCALE[4], GCONF[4], XACTUAL[4];
u16 ADC_VALUE[3];
u8 tempbq[21];
u8 tempbq1[1], tempbq2[1];
u8 temptmp[2];
u16 tempadc[9];
s32 tempdata, tempdata1, temp2;
s32 ForceRawSetAct_temp;
s32 ForceRawSetAct_temp2;
float Force;
u8 Flag_100ms_Toggle;

extern u32 RGB_DATA;

extern s32 ForceRawOffset;
extern s32 ForceRawActual;
extern s32 ForceRawSet;
extern float EyeTmp;
extern u8 PWR_STATE;
extern u8 WorkMode;
extern u8 PowerState, LastPowerState;
extern PID_TypeDef HeatPID;
extern PID_TypeDef MotorPID;
extern PID_TypeDef MotorPID_speed;
uart_data uart_RX_data;                                                              // DMA空闲接收数组
uart_data received_data;                                                             // 环形数组取出，数组地址
ring_buffer_t uart_rx_ring_buffer = {.head = 0, .tail = 0, .size = UART_QUEUE_SIZE}; // 定义环形数组
int a;
// extern u8 USART1_RX_STA;
/*

1:
2:
3:
3:
*/
extern u8 Flag_3200ms, Flag_800ms, Flag_600ms, Flag_400ms, Flag_200ms, Flag_100ms;
extern u8 BQ25895Reg[21];

extern BQ27441_typedef BQ27441;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_TIM14_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_TIM7_Init();
    /* USER CODE BEGIN 2 */
    delay_init(64);
    //	LEDCtrl(LED3,LEDOn);
    AT24CXX_Init();
    BQ27441_Init();
    BQ25895_Init();
    // HeatPIDInit();
    PID_Init(&HeatPID, 30, 0, 0, 100, 0, 255, 0, 42.5);
    PID_Init(&MotorPID_speed, 0.035, 0, 0, 1000, -1000, (float)(50000), (float)(-50000), (float)ForceRawSet); // 0.03,0.05//0.02, 0.01, 0.02,
    // PID_Init(&MotorPID, 1.8, 0, 0, 1000, -1000, (float)(150000), (float)(0), (float)ForceRawSetAct);
    HeatPower(ON);
    //	HeatPWMSet(50);
    PWM_WS2812B_Init();
    PWM_WS2812B_Show(5);
    ScreenUpdateSOC(BQ27441.SOC, PowerState);
    PowerState = 3;
    PWM_WS2812B_Write_24Bits(5, 0x0000ff);
    PWR_STATE = 1;
    TMC5130_Init();
    MotorChecking();
    HX711_Init();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_VALUE, 3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_RX_data.buffer, sizeof(uart_RX_data.buffer)); // To open the DMA serial port free to receive, you need to configure the priority of the data processing function, or use the ring array directly
                                                                                             /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        if (ring_buffer_get(&uart_rx_ring_buffer, &received_data) == 0) {
            // 处理接收到的数据
            if ((received_data.buffer[0] == 0x5A) && (received_data.buffer[1] == 0xA5)) {
                UART1_CMDHandler((PCTRL_MSG)received_data.buffer);
            }
            if ((received_data.buffer[0] == 0x7A) && (received_data.buffer[1] == 0xA7)) {//updata pid
                PCTRL_MSG_PID pid_data = (CTRL_MSG_PID *)received_data.buffer;//force Conversion receiving frame
                PID_Init(&MotorPID_speed, pid_data->p, pid_data->i, pid_data->d, 1000, -1000, (float)(50000), (float)(-50000), (float)ForceRawSet); // 0.03,0.05//0.02, 0.01, 0.02,
            }
        }
        if (PowerState == 10) {
            if (Flag_800ms) {
                BQ25895_MultiRead(BQ25895Reg);
                PowerStateUpdate();
                LEDUpdate();
                BQ27441_MultiRead(&BQ27441);
                BATCheckDIS();
                Flag_800ms = 0;
            }
            LEDUpdate();
            PWR_STATE = 0;
            HAL_ADC_Stop_DMA(&hadc1);
            PWM_WS2812B_Write_24Bits(5, 0x000000);
            SystemPowerDown();
        }
        // else if (PowerState == 1) // iic confirms that type_c is  not charging
        // {
        // a++; // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,RESET);//灞忓箷澶嶅師
        LEDUpdate();
        // HAL_Delay(20);
        // ScreenUpdateSOC(BQ27441.SOC, PowerState); // 鐢甸噺涓婁紶
        SW_CMDHandler();
        TaskProcessing();
        if (USART1_RX_STA == 1) {
            UART1_CMDHandler((PCTRL_MSG)USART1_RX_BUF);
            USART1_RX_STA = 0;
        }
        if (Flag_3200ms) {
            ScreenUpdateSOC(BQ27441.SOC, PowerState); 
            Flag_3200ms = 0;
        }
        if (Flag_800ms) {
            BQ25895_MultiRead(BQ25895Reg); // 
            PowerStateUpdate();            // 
            LEDUpdate();                   // 
            BQ27441_MultiRead(&BQ27441);   // 
            BATCheckDIS();
            Flag_800ms = 0;
        }
        if (Flag_600ms) {

            if (((WorkMode == 5) || (WorkMode == 1) || (WorkMode == 3) || (WorkMode == 7)) && (Flag_100ms_Toggle == 0)) { //&& Flag_100ms_Toggle == 0
                ScreenUpdateTemperature(EyeTmp);
            }
            if ((((WorkMode & 0x06) == 0x06) || ((WorkMode & 0x07) == 0x07)) && (Flag_100ms_Toggle == 1)) { // && Flag_100ms_Toggle == 1
                ForceRawSetAct_temp = (ForceRawActual - ForceRawOffset < 0) ? 0 : (ForceRawActual - ForceRawOffset);
                Limit(ForceRawSetAct_temp, 0, ForceRawSetAct_temp);
                ScreenUpdateForce(ForceRawSetAct_temp);

                // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);LED0
            }
            Flag_100ms_Toggle = !Flag_100ms_Toggle;
            Flag_600ms        = 0;
        }
        if (PWR_SW == 0) {
        }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //}
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN       = 8;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
