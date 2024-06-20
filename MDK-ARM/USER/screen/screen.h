#ifndef _SCREEN_H
#define _SCREEN_H
#include "sys.h"
#include "stdio.h"	

#define USART_REC_LEN  			20  	//�����������ֽ��� 200

#define False	0
#define Ture	1
	  	
extern u8  USART1_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART1_RX_STA;       //����״̬���	

#define RXBUFFERSIZE   1 //�����С
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL��USART����Buffer


typedef struct __attribute__((packed))
{
    uint8_t cmd_head_high; // ֡ͷ
    uint8_t cmd_head_low; // ֡ͷ
    uint8_t cmd_type_high; // ��������(UPDATE_CONTROL)
    uint8_t cmd_type_low; // ��������(UPDATE_CONTROL)
    float data;

} CTRL_MSG, *PCTRL_MSG;


void SendDataToScreen(u8* Data,u8 len);
void ScreenUpdateTemperature(float value);
void ScreenUpdateForce(u32 value);
void ScreenUpdateSOC(u16 value,u8 state);
void ScreenWorkModeQuit(u8 workmodenumber);
void ScreenTimerStart(u8 workmodenumber);
uint32_t processFilter_force(uint32_t *buffer ,int filter);
#endif
