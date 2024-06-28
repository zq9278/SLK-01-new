/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-24 19:24:03
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-06-27 16:59:58
 * @FilePath: \EIDEd:\Project\SLK01\Software\SLK-01-new\MDK-ARM\USER\screen\screen.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef _SCREEN_H
#define _SCREEN_H
#include "sys.h"
#include "stdio.h"

#define USART_REC_LEN 20 // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟街斤拷锟斤拷 200

#define False         0
#define Ture          1

extern u8 USART1_RX_BUF[USART_REC_LEN]; // 锟斤拷锟秸伙拷锟斤拷,锟斤拷锟経SART_REC_LEN锟斤拷锟街斤拷.末锟街斤拷为锟斤拷锟叫凤拷
extern u8 USART1_RX_STA;                // 锟斤拷锟斤拷状态锟斤拷锟?

#define RXBUFFERSIZE 1             // 锟斤拷锟斤拷锟叫?
extern u8 aRxBuffer[RXBUFFERSIZE]; // HAL锟斤拷USART锟斤拷锟斤拷Buffer

#define RX_BUFFER_SIZE  204
#define UART_QUEUE_SIZE 10
#define UART_MSG_SIZE   100

typedef struct
{
    uint8_t buffer[UART_MSG_SIZE]; // ?????‘????°????????????????
    uint16_t length;               // ??°???é??????
} uart_data;

typedef struct
{
    uart_data data[UART_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t size;
} ring_buffer_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd_head_high; // 帧头
    uint8_t cmd_head_low;  // 帧头
    uint8_t cmd_type_high; // 锟斤拷锟斤拷锟斤拷锟斤拷(UPDATE_CONTROL)
    uint8_t cmd_type_low;  // 锟斤拷锟斤拷锟斤拷锟斤拷(UPDATE_CONTROL)
    float data;
} CTRL_MSG, *PCTRL_MSG;
typedef struct __attribute__((packed)) {
    uint8_t cmd_head_high; // 帧头
    uint8_t cmd_head_low;  // 帧头
    uint8_t cmd_type_high; // 锟斤拷锟斤拷锟斤拷锟斤拷(UPDATE_CONTROL)
    uint8_t cmd_type_low;  // 锟斤拷锟斤拷锟斤拷锟斤拷(UPDATE_CONTROL)
    float data;
    float data2;
    uint8_t a;        // 锟斤拷锟斤拷锟斤拷锟斤拷(UPDATE_CONTROL)
    uint8_t end_high; // 帧头
    uint8_t end_low;  // 帧头
} CTRL_MSG1;
typedef struct __attribute__((packed)) {
    uint8_t cmd_head_high; // 帧头
    uint8_t cmd_head_low;  // 帧头
    float p;
    float i;
    float d;
    uint8_t end_high; // 帧头
    uint8_t end_low;  // 帧头
} CTRL_MSG_PID, *PCTRL_MSG_PID;
void SendDataToScreen(u8 *Data, u8 len);
void ScreenUpdateTemperature(float value);
void ScreenUpdateForce(u32 value);
void ScreenUpdateSOC(u16 value, u8 state);
void ScreenWorkModeQuit(u8 workmodenumber);
void ScreenTimerStart(u8 workmodenumber);
uint32_t processFilter_force(uint32_t *buffer, int filter);

int ring_buffer_put(ring_buffer_t *buffer, uart_data *data);
int ring_buffer_get(ring_buffer_t *buffer, uart_data *data);
#endif
