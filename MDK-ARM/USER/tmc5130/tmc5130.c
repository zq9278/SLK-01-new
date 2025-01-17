#include "tmc5130.h"
#include "delay.h"
#include "usart.h"
#include "pid.h"
#include "hx711.h"
#include "screen.h"
/*motor*/
u32 MotorSpeed = 0x4000;

/*motor*/

extern SPI_HandleTypeDef hspi1;
u8 tempa[4];
extern s32 ForceRawOffset; //???????
extern s32 ForceRawActual; //????????
extern s32 ForceActual[10];
extern s32 ForceRawSetAct;

extern float control_output;
extern float control_output_speed;
extern PID_TypeDef MotorPID;
extern PID_TypeDef MotorPID_speed;
void TMC5130_Init(void)
{
    //	TMC_ENN(0);//锟斤拷锟绞癸拷锟?
    delay_ms(200);

    TMC5130_Write(0x81, 0x00000001); // reset
    TMC5130_Write(0xec, 0x000300c3); // CHOPCONF: vsense=1,TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
    TMC5130_Write(0x90, 0x00001000); // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
    TMC5130_Write(0x91, 0x0000000a); // TPOWERDOWN=10: Delay before power down in stand still
    TMC5130_Write(0x80, 0x00000004); // EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
    TMC5130_Write(0x93, 0x000001f4); // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
    TMC5130_Write(0xf0, 0x000701c8); // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1

    TMC5130_Write(0xa4, 0x00011000); // A1 = 1 000 First acceleration
    TMC5130_Write(0xa5, 0x00015000); // V1 = 50 000 Acceleration threshold velocity V1
    TMC5130_Write(0xa6, 0x00018fff); // AMAX = 500 Acceleration above V1
    TMC5130_Write(0xa7, MotorSpeed); // VMAX = 200 000
    TMC5130_Write(0xa8, 0x00001fff); // DMAX = 700 Deceleration above V1
    TMC5130_Write(0xaa, 0x00008000); // D1 = 1400 Deceleration below V1
    TMC5130_Write(0xab, 0x0000000a); // VSTOP = 10 Stop velocity (Near to zero)
    // TMC5130_Write(0xac, 0x00000000);
    // TMC5130_Write(0xb4, 0x0000075f);
    TMC5130_Write(0xa0, 0x00000000); // 位锟斤拷模式
}

void TMC5130_Read(u8 ReadAddr, u8 *pBuffer)
{
    u8 Data[5];
    Data[0] = ReadAddr;
    TMC_CSN(0);
    HAL_SPI_Transmit(&hspi1, Data, 5, 100);
    TMC_CSN(1);
    __nop();
    __nop();
    TMC_CSN(0);
    HAL_SPI_Transmit(&hspi1, Data, 1, 100);
    HAL_SPI_Receive(&hspi1, pBuffer, 4, 100);
    TMC_CSN(1);
}

void TMC5130_Write(u8 WriteAddr, u32 WriteData)
{
    u8 Data[5];
    Data[0] = WriteAddr;
    Data[1] = (u8)((WriteData) >> 24);
    Data[2] = (u8)((WriteData) >> 16);
    Data[3] = (u8)((WriteData) >> 8);
    Data[4] = (u8)WriteData;
    TMC_CSN(0);
    HAL_SPI_Transmit(&hspi1, Data, 5, 100);
    TMC_CSN(1);
}

void MotorSetHome(void)
{
    TMC5130_Write(0xa1, 0);
    TMC5130_Write(0xad, 0);
}

void MotorCtrl(s32 Step)
{
    u8 Data[5];
    Data[0] = 0xad;
    Data[1] = (u8)((Step) >> 24);
    Data[2] = (u8)((Step) >> 16);
    Data[3] = (u8)((Step) >> 8);
    Data[4] = (u8)Step;
    TMC_CSN(0);
    HAL_SPI_Transmit(&hspi1, Data, 5, 100);
    TMC_CSN(1);
}

void VelocityModeMove(u8 direction)
{
    u8 Data[5];
    Data[0] = 0xa0;
    Data[1] = 0x00;
    Data[2] = 0x00;
    Data[3] = 0x00;
    Data[4] = direction;
    TMC_CSN(0);
    HAL_SPI_Transmit(&hspi1, Data, 5, 100);
    TMC_CSN(1);
}

void StepMinMax(s32 *Step, s32 MinValue, s32 MaxValue)
{
    if (*Step < MinValue) {
        *Step = MinValue;
    }
    if (*Step > MaxValue) {
        *Step = MaxValue;
    }
}

u8 MotorChecking()
{
    u32 ReadCount = 0;
    u8 ReadData[4];

    TMC_ENN(0); // 锟斤拷锟绞癸拷锟?
    TMC5130_Write(0xa7, 0x10000);
    VelocityModeMove(Negative);
    do {
        TMC5130_Read(0x04, ReadData);
        if ((ReadData[3] & 0x02) == 0x00) {
            TMC5130_Read(0x04, ReadData);
        }
        delay_us(400);
        ReadCount++;
        if (ReadCount == 65535) {
            TMC5130_Write(0xa7, 0);
            return 0;
        }
    } while ((ReadData[3] & 0x02) == 0x02);

    MotorSetHome();
    TMC5130_Write(0xa0, 0x00000000); // 位锟斤拷模式
    TMC_ENN(1);

    return 1;
    //	else
    //	{
    //		VelocityModeMove(DriverID,Positive);
    //		do
    //		{
    //			TMC5130_Read(DriverID,0x04,MotorStatus);
    //			if((MotorStatus[3]&0x01)==0x00)
    //			{
    //				TMC5130_Read(DriverID,0x04,MotorStatus);
    //			}
    //			delay_us(400);
    //			ReadCount++;
    //			if(ReadCount==65535)
    //			{
    //				TMC5130_Write(DriverID,0xa7,0);
    //				return 0;
    //			}
    //		}while((MotorStatus[3]&0x01)==0x01);
    //	}
}

u8 MotorCompare(s32 SetData, s32 CompareData)
{
    s32 SubData;
    SubData = CompareData - SetData;
    if (SubData > 0) // ForceSen  瓒呬簡
    {
        TMC5130_Write(0xa7, 0x6000);
        TMC5130_Write(0xa0, 2);
        return 2;
    } else if (SubData < 0) // 灏戜簡
    {
        TMC5130_Write(0xa7, 0x4000);
        TMC5130_Write(0xa0, 1);
        return 1;
    } else {
        TMC5130_Write(0xa7, 0x6000);
        TMC5130_Write(0xa0, 2);
        return 0;
    }
}

// TMC_ENN(0); // 使能电机
// TMC5130_Write(0xad, 115000); // 绝对位置
/*pid控制位置*/
void SetMotorposition(int position)
{
    TMC5130_Write(0xa7, 1000);
    TMC5130_Write(0xa0, 0);
    TMC5130_Write(0xad, (uint32_t)position);
}
/*pid控制速度*/
void SetMotorSpeed(int speed)
{
    if (speed > 0) {
        TMC5130_Write(0xa0, 1);
        TMC5130_Write(0xa7, (uint32_t)speed);
    } else {
        TMC5130_Write(0xa0, 2);
        TMC5130_Write(0xa7, (uint32_t)(-speed));
    }
}

void PressureControl(void)
{
    // float setpoint = 100.0; // 初始目标压力值
    // ForceRawActual = HX711_Read();
    // for (int i = 0; i < 10; i++) {
    //     ForceActual[i] = HX711_Read();
    // }
    // ForceRawActual = processFilter_force((uint32_t *)ForceActual, 10);

    // // control_output = PID_Compute(&MotorPID, (float)(ForceRawActual - ForceRawOffset), (float)ForceRawSetAct); // 传递当前设定值
    // // SetMotorposition((int)control_output);
    // control_output_speed = PID_Compute(&MotorPID_speed, (float)(ForceRawActual - ForceRawOffset), (float)ForceRawSetAct); // 传递当前设定值

    // if ((ForceRawActual - ForceRawOffset) < (ForceRawSetAct * 3 / 5)) {
    //     control_output_speed = 30000;
    // }
 

    // SetMotorSpeed((int)control_output_speed);
     //static CTRL_MSG1 pData;
    // pData.cmd_head_high = 0x5A;
    // pData.cmd_head_low  = 0xA5;
    // pData.cmd_type_high = 0x70;
    // pData.cmd_type_low  = 0x01;
    // s32 temp            = ForceRawActual - ForceRawOffset;
    // temp                = (u16)((temp / HX711_SCALE_FACTOR_100) * 0.0098 * 88.4);

    // pData.data     = (float)temp;
    // pData.data2    = (float)control_output_speed;
    // pData.end_high = 0xff;
    // pData.end_low  = 0xee;
    // HAL_UART_Transmit(&huart1, (uint8_t *)&pData, sizeof(CTRL_MSG1), 1);
}