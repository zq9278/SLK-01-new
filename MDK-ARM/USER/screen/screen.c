#include "screen.h"
#include "math.h"
#include "stdlib.h"
#include "hx711.h"

// �������´���,֧��printf����,������Ҫѡ��use MicroLIB
// #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#if 1
// #pragma import(__use_no_semihosting)
// ��׼����Ҫ��֧�ֺ���
struct _FILE {
    int handle;
};

FILE __stdout;
// ����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}
// �ض���fputc����
int fputc(int ch, FILE *f)
{
    while ((USART4->ISR & 0X40) == 0); // ѭ������,ֱ���������
    USART4->TDR = (u8)ch;
    return ch;
}
#endif
u8 USART1_RX_BUF[USART_REC_LEN]; // ���ջ���,���USART_REC_LEN���ֽ�.
// #if EN_USART1_RX   //���ʹ���˽���
// ����1�жϷ������
// ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���

// ����״̬
// bit15��	������ɱ�־
// bit14��	���յ�0x0d
// bit13~0��	���յ�����Ч�ֽ���Ŀ
u8 USART1_RX_STA = 0; // ����״̬���
u8 SendBuff[15];

u8 aRxBuffer[RXBUFFERSIZE]; // HAL��ʹ�õĴ��ڽ��ջ���

u16 ScreenCmdAdd, ScreenCmdData;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern u8 WorkMode;

uint32_t processFilter_force(uint32_t *buffer ,int filter)
{
	// �������������
	for (int i = 0; i < filter - 1; i++)
	{
		for (int j = 0; j < filter - i - 1; j++)
		{
			if (buffer[j] > buffer[j + 1])
			{
				// ���� buffer[j] �� buffer[j + 1]
				uint32_t temp = buffer[j];
				buffer[j] = buffer[j + 1];
				buffer[j + 1] = temp;
			}
		}
	}

	// ��� FILTER_SIZE �������������м��ֵ
	// �����ż���������м�����ֵ��ƽ��ֵ
	if (filter % 2 != 0)
	{
		//Limit(buffer[FILTER_SIZE / 2],min,max) 
		return buffer[filter / 2];
	}
	else
	{
		return (buffer[(filter - 1) / 2] + buffer[filter / 2]) / 2;
	}
}

void SendDataToScreen(u8 *Data, u8 len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)Data, len, 1000);
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET); // �ȴ����ͽ���
}
float Forcevalue;

void ScreenUpdateForce(u32 value)
{

 
    //Forcevalue = (u16)(value / HX711_SCALE_FACTOR_10 * 6);
    Forcevalue = (u16)((value /HX711_SCALE_FACTOR_100)*0.0098*88.4);

    static CTRL_MSG pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low  = 0xA5;
    pData.cmd_type_high = 0x20;

    pData.data = Forcevalue;
    if (WorkMode == 0x06) {
        pData.cmd_type_low = 0x05;
    } else if (WorkMode == 0x07) {
        pData.cmd_type_low = 0x47;
    }
    //while (hdma_usart1_tx.State != HAL_DMA_STATE_READY);

    HAL_UART_Transmit(&huart1, (u8 *)&pData, sizeof(pData),100);
}

void ScreenUpdateTemperature(float value)
{
    // u16 Tmpvalue;

    // Tmpvalue=value+0.5f;
   // while (hdma_usart1_tx.State != HAL_DMA_STATE_READY);
    static CTRL_MSG pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low  = 0xA5;
    pData.cmd_type_high = 0x20;
    if ((WorkMode == 0x05)|| (WorkMode ==1))  {
        if(WorkMode ==1){
            if(value>=37.9)value=37.9;
        }
       
        pData.cmd_type_low = 0x41;
    } else if ((WorkMode == 0x07)||(WorkMode == 3)){
           if(WorkMode ==3){
            if(value>=37.9)value=37.9;
        }
        pData.cmd_type_low = 0x37;
    }
     if(value>=42.9)value=42.9;
    pData.data = value;

    HAL_UART_Transmit(&huart1, (u8 *)&pData, sizeof(pData),100);
}

void ScreenUpdateSOC(u16 value, u8 state)
{
    // u8 SOCvalue;
    // SOCvalue=value/20;
    // if(SOCvalue==5)SOCvalue=4;
    static uint8_t last_state    = 0xFF; // ʹ�ò����ܵĳ�ʼֵȷ����һ�μ�⵽�仯
    static uint8_t current_state = 0xFF; // ʹ�ò����ܵĳ�ʼֵȷ����һ�μ�⵽�仯
    static uint8_t data_pending  = 0;    // ���ݴ����ͱ�־
    if (state == 1 || state == 2) {//����б仯
        current_state = 0;//����״ֵ̬
    } else {
        current_state = 1;
    }
    // ֻ��״̬�ı�ʱ���ô����ͱ�־��׼������
    if (current_state != last_state) {
        last_state   = current_state; // ����״̬��¼
        data_pending = 1;             // �������ݴ����ͱ�־
    }

    //  if ((data_pending == 1) && (hdma_usart1_tx.State != HAL_DMA_STATE_READY)) {
    if ((data_pending == 1) && (state == 1 || state == 2) && (hdma_usart1_tx.State == HAL_DMA_STATE_READY)) {
        static CTRL_MSG pData1;
        pData1.cmd_head_high = 0x5A;
        pData1.cmd_head_low  = 0xA5;
        pData1.cmd_type_high = 0x20;
        pData1.cmd_type_low  = 0x53;
        HAL_UART_Transmit(&huart1, (u8 *)&pData1, sizeof(pData1),100);//����ɫ
        data_pending = 0;
    }
    if ((data_pending == 1) && (state == 0 || state == 3) && (hdma_usart1_tx.State == HAL_DMA_STATE_READY)) {
        static CTRL_MSG pData1;
        pData1.cmd_head_high = 0x5A;
        pData1.cmd_head_low  = 0xA5;
        pData1.cmd_type_high = 0x20;
        pData1.cmd_type_low  = 0x54;
        HAL_UART_Transmit(&huart1, (u8 *)&pData1, sizeof(pData1),100);//�ر���ɫ
        data_pending = 0;
    }

    //while (hdma_usart1_tx.State != HAL_DMA_STATE_READY);
    static CTRL_MSG pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low  = 0xA5;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low  = 0x50;
    pData.data          = value;

    HAL_UART_Transmit(&huart1, (u8 *)&pData, sizeof(pData),100);//ʵʱ���µ���
}

void ScreenWorkModeQuit(u8 workmodenumber)
{

    //while (hdma_usart1_tx.State != HAL_DMA_STATE_READY);

    // 等待DMA传输通道空闲
    // while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY) {
    //     // 可以在这里添加超时机制，以避免死循环
    // }
    
    // // 等待UART外设空闲
    // while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
    //     // 可以在这里添加超时机制，以避免死循环
    // }

    static CTRL_MSG pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low  = 0xA5;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low  = 0x51;

    //HAL_UART_Transmit_DMA(&huart1, (u8 *)&pData, sizeof(pData));//���¿��ؼ����˳�����ģʽ
    HAL_UART_Transmit(&huart1, (u8 *)&pData, sizeof(pData),100);//���¿��ؼ����˳�����ģʽ
}

void ScreenTimerStart(u8 workmodenumber) 
{
    //while (hdma_usart1_tx.State != HAL_DMA_STATE_READY);
    static CTRL_MSG pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low  = 0xA5;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low  = 0x52;

    HAL_UART_Transmit(&huart1, (u8 *)&pData, sizeof(pData),100);// �������£�ģʽ����
}