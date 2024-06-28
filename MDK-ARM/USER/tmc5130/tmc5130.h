/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-15 13:31:10
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-06-20 11:04:03
 * @FilePath: \EIDEd:\Project\SLK01\Software\SLK-01-new\MDK-ARM\USER\tmc5130\tmc5130.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __TMC5130_H
#define __TMC5130_H
#include "sys.h"

#define Positive 1
#define Negative 2

void TMC5130_Init(void);
void TMC5130_Read(u8 ReadAddr, u8 *pBuffer);
void TMC5130_Write(u8 WriteAddr, u32 WriteData);
void MotorCtrl(s32 Step);
void StepMinMax(s32 *Step, s32 MinValue, s32 MaxValue);
void MotorSetHome(void);
u8 MotorChecking(void);
void VelocityModeMove(u8 direction);
void MotorMove(u8 DriverID, s16 ADCValue, u16 absADCValue, u16 ADCThreshold, u16 MotorSpeed);
u8 MotorCompare(s32 SetData, s32 CompareData);

void SetMotorposition(int speed);
void SetMotorSpeed(int speed);

void PressureControl(void);
#endif
