/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-15 13:31:10
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-06-20 15:14:23
 * @FilePath: \EIDEd:\Project\SLK01\Software\SLK-01-new\MDK-ARM\USER\hx711\hx711.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "hx711.h"
#include "main.h"

extern s32 ForceRawOffset;

/* HX711 初�?�化 */
void HX711_Init(void)
{
	ForceRawOffset=HX711_Read();
	
}

/* 读取HX711数据 */
int32_t HX711_Read(void)
{
    int32_t data = 0;
    uint8_t i;

    /* 等待数据引脚为低 ,即A/D�?换器准�?�好*/
    while (HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_SET)//当数�?位是高电平时，一直进入循�?，直到数�?为变成低电平
    {;}

    /* 读取24位数�? */
    for (i = 0; i < 24; i++)
    {
        /* 时钟引脚�?�? */
        HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
       data <<= 1;

        /* 延迟 */
//        delay_us(1);
				__nop();
				__nop();
				__nop();
        /* 时钟引脚�?�? */
        HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);

        /* 检查数�?引脚的�? */
        if (HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_SET)
        {
            data++;
        }

        /* 延迟 */
//        delay_us(1);
				__nop();
				__nop();
				__nop();
    }

    /* 时钟引脚�?�? */
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
//    delay_us(1);
		__nop();
		__nop();
		__nop();
    /* 时钟引脚�?�? */
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
    //delay_us(1);    /* 符号位扩�? */
    if (data & 0x00800000)
    {
        data |= 0xFF000000;
    }
			//data=data^ 0x800000;
    return data;
}

int32_t HX711_AVG(u8 times)
{
	s32 tempdata[times];
	s64 tempSum=0;
	s32 AVGData;
	for(u16 i=0;i<times;i++)
	{
		tempdata[i]=HX711_Read();
		tempSum+=tempdata[i];
	}
	AVGData=tempSum/times;
	return AVGData;
}

/* 获取力（牛顿�? */
float HX711_GetForce(void)
{
    int32_t rawData = HX711_Read();
  //  float force = (float)rawData;
	  //float force = (float)rawData / HX711_SCALE_FACTOR;
	//float force = ((0-(float)rawData) / HX711_SCALE_FACTOR)/10+2.538;
	float force = (float)rawData / HX711_SCALE_FACTOR;
    return force;
}

s32 Force2Raw(u8 Force)
{
	return (s32)(Force*HX711_SCALE_FACTOR);
}
