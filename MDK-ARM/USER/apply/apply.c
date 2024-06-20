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
0:???
0000 0001:?????????
0000 0010:?????????
0000 0011:??????
0000 0100:
0000 0101:??????????
0000 0110:??????????
0000 0111:???????
4:???????
*/
u8 PowerState, LastPowerState;
extern PID_typedef HeatPID;
/*

1:??????
2:???????
3:��??????????
*/

float ForceSet  = 5.0;
s32 ForceRawSet = 422672; //????څ???
s32 ForceRawSetAct;       //???څ?

u8 MotorState;
u8 MotorCompareState;

extern uint8_t SW_CNT_Flag;
extern TIM_HandleTypeDef htim6;
extern u16 Tim6Cnt;

extern u8 BQ25895Reg[21];

extern u16 ScreenCmdAdd, ScreenCmdData;

extern u8 HeatPWMVal;

extern BQ27441_typedef BQ27441;

void SW_CMDHandler(void)
{
    if (SW_CNT_Flag) {
        SW_CNT_Flag = 0;
        switch (WorkMode) {
            /**/
            case 0:
                // WorkMode |=0x06;
                // TMC_ENN(0);//??????
                // HAL_TIM_Base_Start_IT(&htim6);
				
                break;

            /*?????????->??????????*/
            case 0x01:
                WorkMode |= 0x04;
				HeatPID.target_val=42.5;
                 ScreenTimerStart(WorkMode);
                // HeatPower(ON);
                break;

            /*?????????->??????????*/
            case 0x02:
                WorkMode |= 0x04;
                ScreenTimerStart(WorkMode);
                TMC_ENN(0); //??????
                HAL_TIM_Base_Start_IT(&htim6);
                break;

            /*??????->???????*/
            case 0x03:
                WorkMode |= 0x04;
				HeatPID.target_val=42.5;
                 ScreenTimerStart(WorkMode);
                // HeatPower(ON);
                TMC_ENN(0); //??????
                HAL_TIM_Base_Start_IT(&htim6);
                break;

            case 0x04:
                break;

            /*????????????*/
            case 0x05:
                ScreenWorkModeQuit(WorkMode);
                WorkMode = 0;
                HeatPower(OFF);
                HeatPWMVal = 0;
                break;

            /*????????????*/
            case 0x06:
                ScreenWorkModeQuit(WorkMode);
                WorkMode = 0;
                HAL_TIM_Base_Stop_IT(&htim6);
                Tim6Cnt    = 0;
                MotorState = 0;
                MotorChecking();
                break;

            /*?????????*/
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
            ForceRawActual = HX711_Read();
            //			MotorCompare(ForceRawOffset+422672,ForceRawActual);//5N
            MotorCompareState = MotorCompare(ForceRawSetAct, ForceRawActual - ForceRawOffset);
            for (int i = 0; i < 10; i++) {
                ForceActual[i] = HX711_Read();
            }
            ForceRawActual = processFilter_force((uint32_t *)ForceActual, 10);
            break;
        case 7:
            ForceRawActual = HX711_Read();
            //			MotorCompare(ForceRawOffset+422672,ForceRawActual);//5N
            MotorCompareState = MotorCompare(ForceRawSetAct, ForceRawActual - ForceRawOffset);
            for (int i = 0; i < 10; i++) {
                ForceActual[i] = HX711_Read();
            }
            ForceRawActual = processFilter_force((uint32_t *)ForceActual, 10);
            break;
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
    uint16_t cmd_head = ((msg->cmd_head_high) << 8) | (msg->cmd_head_low); // ������8λֵ�ϲ���һ��16λֵ
    uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low); // ������8λֵ�ϲ���һ��16λֵ
    // uint16_t data = ((msg->data_high) << 8) | (msg->data_low);			   // ָ������
    float data = msg->data;

    // ScreenCmdAdd=(DATA[2]<<8)|DATA[3];
    // ScreenCmdData=(DATA[5]<<8)|DATA[6];
    // switch(ScreenCmdAdd)
    switch (cmd_type) {
        /*串口发送数据开始预热*/
        case 0x1041:
            WorkMode = 0x01;
			HeatPID.target_val=37.5;
            
            HeatPower(ON);
            break;

        /*�ȷ����*/
        case 0x1030:
            WorkMode = 0;
            HeatPower(OFF);
            HeatPWMVal = 0;
            break;

        /*������ʼ*/
        case 0x1005:
            // ForceSet=ScreenCmdData/50;//?څ???
            ForceSet = data / 88.4; //?څ???
            // ForceRawSet=ForceSet*HX711_SCALE_FACTOR;
            ForceRawSet = (ForceSet / 0.0098) * HX711_SCALE_FACTOR_100;
            WorkMode    = 0x02;
            break;

        /*��������*/
        case 0x1034:
            WorkMode = 0;
            HAL_TIM_Base_Stop_IT(&htim6);
            Tim6Cnt    = 0;
            MotorState = 0;
            MotorChecking();
            break;

        /*开启预热*/
        case 0x1037:
    HeatPID.target_val=37.5;
            
            HeatPower(ON);
			
            ForceSet = data / 88.4; //?څ???
            // ForceRawSet=ForceSet*HX711_SCALE_FACTOR;
            ForceRawSet = (ForceSet / 0.0098) * HX711_SCALE_FACTOR_100;
            WorkMode    = 0x03;
            break;

        /*�Զ�����*/
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
