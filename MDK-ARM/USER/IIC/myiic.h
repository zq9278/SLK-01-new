#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"
	
//IO��������
#define SDA_IN()  {GPIOA->MODER&=~(GPIO_MODER_MODE0<<(8*2));GPIOA->MODER|=0<<8*2;}	//PA8����ģʽ
#define SDA_OUT() {GPIOA->MODER&=~(GPIO_MODER_MODE0<<(8*2));GPIOA->MODER|=1<<8*2;} //PA8���ģʽ
//IO����
#define IIC_SCL(n)  (n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)) //SCL
#define IIC_SDA(n)  (n?HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET)) //SDA
#define WP(n)  (n?HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET)) //SDA
#define READ_SDA    HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)  //����SDA

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
#endif

