#include "apply.h"
#include "tmc5130.h"
#include "hx711.h"
#include "led.h"
#include "heat.h"
#include "bq27441.h"
#include "bq25895.h"
#include "screen.h"
#include "pid.h"
s32 ForceRawOffset; //???????
s32 ForceRawActual; //????????

u8 PWR_STATE = 0;

u8 WorkMode = 0;
s32 ForceActual[10];
/*

*/
u8 PowerState, LastPowerState;
extern PID_TypeDef HeatPID;
extern UART_HandleTypeDef huart1;
/*


*/

float ForceSet  = 5.0;
s32 ForceRawSet = 422672;
u8 MotorCompareState;
u8 PressureModeStart = 1;
float control_output;
float control_output_speed;
PID_TypeDef MotorPID;
PID_TypeDef MotorPID_speed;

extern uint8_t SW_CNT_Flag;
extern TIM_HandleTypeDef htim6;
extern u16 Tim6Cnt;
extern u8 Flag_1s, Flag_2s, Flag_3s;
extern u8 BQ25895Reg[21];
 u8 MotorState;

extern u16 ScreenCmdAdd, ScreenCmdData;

extern u8 HeatPWMVal;

extern BQ27441_typedef BQ27441;

static CTRL_MSG1 pData;

void SW_CMDHandler(void) // 处理按键按下的功能
{
    if (SW_CNT_Flag) {
        SW_CNT_Flag = 0;
        switch (WorkMode) {
            /**/
            case 0: // 没有工作
                // WorkMode |=0x06;
                // TMC_ENN(0);//??????
                // HAL_TIM_Base_Start_IT(&htim6);

                break;

            /*开始加热*/
            case 0x01:
                WorkMode |= 0x04;
                HeatPID.setpoint = 42.5;
                ScreenTimerStart(WorkMode);
                HeatPower(ON);
                break;

            /*开始脉动*/
            case 0x02:
                WorkMode |= 0x04;
                ScreenTimerStart(WorkMode);
                TMC_ENN(0); //??????
                PressureModeStart = 1;
                HAL_TIM_Base_Start_IT(&htim6);
                break;

            /*开始自动*/
            case 0x03:
                WorkMode |= 0x04;
                HeatPID.setpoint = 42.5;
                ScreenTimerStart(WorkMode);
                HeatPower(ON);
                TMC_ENN(0);
                PressureModeStart = 1;
                HAL_TIM_Base_Start_IT(&htim6);
                break;

            case 0x04:
                break;

            /*关闭加热*/
            case 0x05:
                ScreenWorkModeQuit(WorkMode);
                WorkMode = 0;
                HeatPower(OFF);
                HeatPWMVal = 0;
                break;

            /*停止脉动*/
            case 0x06:
                ScreenWorkModeQuit(WorkMode);
                WorkMode = 0;
                HAL_TIM_Base_Stop_IT(&htim6);
                Tim6Cnt    = 0;
                MotorState = 0;
                MotorChecking();
                break;

            /*停止自动*/
            case 0x07:
                ScreenWorkModeQuit(WorkMode);
                WorkMode = 0;
                HeatPower(OFF);
                HeatPWMVal = 0;
                HAL_TIM_Base_Stop_IT(&htim6);
                Tim6Cnt    = 0;
                MotorState = 0;
                MotorChecking();
                break;
            default:
                break;
        }
    }
}

void TaskProcessing(void)
{
    switch (WorkMode) {
        case 0:
            break;
        case 1:
            break;
        case 6:
            pData.cmd_head_high = 0x5A;
            pData.cmd_head_low  = 0xA5;
            pData.cmd_type_high = 0x70;
            pData.cmd_type_low  = 0x01;
            s32 temp            = ForceRawActual - ForceRawOffset;
            temp                = (u16)((temp / HX711_SCALE_FACTOR_100) * 0.0098 * 88.4);
            pData.data          = (float)temp;
            pData.data2         = (float)control_output_speed;
            pData.end_high      = 0xff;
            pData.end_low       = 0xee;
            MOTOR_Process();
            HAL_UART_Transmit(&huart1, (uint8_t *)&pData, sizeof(CTRL_MSG1), 1);
        case 7:
            MOTOR_Process();
        default:
            break;
    }
}

void PowerStateUpdate(void)
{
    u8 CHRG_STAT;
    CHRG_STAT = (BQ25895Reg[0x0b] & 0x18) >> 3;
    if (CHRG_STAT == 1 || CHRG_STAT == 2) // Pre-charge Fast Charging
    {
        PowerState = 1;
    } else if (CHRG_STAT == 3) // Charge Termination Done
    {
        PowerState = 2;
    } else if (CHRG_STAT == 0) // Not Charging
    {
        PowerState = 3;
    }
}

void UART1_CMDHandler(PCTRL_MSG msg)
{
    uint16_t cmd_head = ((msg->cmd_head_high) << 8) | (msg->cmd_head_low);
    uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
    // uint16_t data = ((msg->data_high) << 8) | (msg->data_low);
    float data = (float)msg->data;

    // ScreenCmdAdd=(DATA[2]<<8)|DATA[3];
    // ScreenCmdData=(DATA[5]<<8)|DATA[6];
    // switch(ScreenCmdAdd)
    switch (cmd_type) {
        /*收到屏幕加热开始*/
        case 0x1041:
            WorkMode         = 0x01;
            HeatPID.setpoint = 37.5;
            HeatPower(ON);
            break;
        /*收到屏幕加热停止*/
        case 0x1030:
            WorkMode = 0;
            HeatPower(OFF);
            HeatPWMVal = 0;
            break;
        /*收到屏幕脉动开始*/
        case 0x1005:
            //SW_CNT_Flag = 1;//不需要按下物理按键No need to press the physical buttons
            ForceSet    = data / 88.4;
            // ForceRawSet=ForceSet*HX711_SCALE_FACTOR;
            ForceRawSet = (ForceSet / 0.0098) * HX711_SCALE_FACTOR_100;
             WorkMode    = 0x02;
            break;
        /*收到屏幕脉动停止*/
        case 0x1034:
            WorkMode = 0;
            HAL_TIM_Base_Stop_IT(&htim6);
            Tim6Cnt    = 0;
            MotorState = 0;
            MotorChecking();
            break;
        /*收到屏幕自动开始*/
        case 0x1037:
            HeatPID.setpoint = 37.5;
            HeatPower(ON);
            ForceSet = data / 88.4; //?趨???
            // ForceRawSet=ForceSet*HX711_SCALE_FACTOR;
            ForceRawSet = (ForceSet / 0.0098) * HX711_SCALE_FACTOR_100;
            WorkMode    = 0x03;
            break;

        /*收到屏幕自动停止*/
        case 0x1038:
            WorkMode = 0;
            HeatPower(OFF);
            HeatPWMVal = 0;
            HAL_TIM_Base_Stop_IT(&htim6);
            Tim6Cnt    = 0;
            MotorState = 0;
            MotorChecking();
            break;

        default:
            break;
    }
}

void BATCheckDIS(void)
{
    if (BQ27441.Voltage <= 3000) {
        BQ25895_Write(0x09, 0x64); //???BATFET
    }
}
void SystemPowerDown(void)
{
    WorkMode = 0;
    HeatPower(OFF);
    HeatPWMVal = 0;
    HAL_TIM_Base_Stop_IT(&htim6);
    Tim6Cnt    = 0;
    MotorState = 0;
}
void MOTOR_Process(void)
{
    ForceRawActual = HX711_Read();
    for (int i = 0; i < 10; i++) {
        ForceActual[i] = HX711_Read();
    }
    ForceRawActual = processFilter_force((uint32_t *)ForceActual, 10);

    switch (PressureModeStart) {
        case 1: // 最开始的前进阶段
            control_output_speed = 30000;
            SetMotorSpeed((int)control_output_speed);
            if ((ForceRawActual - ForceRawOffset) >= (ForceRawSet * (2.0 / 5.0))) {
                PressureModeStart    = 3;
                control_output_speed = 0;
                SetMotorSpeed((int)control_output_speed);
                HAL_Delay(10);
                Flag_3s = 0;
            }
            pData.a = 1;
            break;
        case 2:

            SetMotorSpeed((int)control_output_speed);
            pData.a = 2;
            break;
        case 3:
            control_output_speed = PID_Compute(&MotorPID_speed, (float)(ForceRawActual - ForceRawOffset), (float)ForceRawSet);
            SetMotorSpeed((int)control_output_speed);
            if (Flag_3s == 1) {
                PressureModeStart = 4;
                Flag_1s           = 0;
                Tim6Cnt           = 0;
            }
            pData.a = 3;
            break;
        case 4:
            control_output_speed = -10000;
            SetMotorSpeed((int)control_output_speed);
            if (Flag_1s == 1) {
                PressureModeStart = 1;
                Flag_3s           = 0;
                Tim6Cnt           = 0;
            }
            pData.a = 4;
            break;
    }
}