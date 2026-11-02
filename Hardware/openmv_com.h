#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "stdio.h"
#include "stm32f10x.h"
#include "Serial.h"
#include "LEDSEG.h"

#define USART_REC_LEN     10 // 定义最大接收字节数 10
#define USART_REC_LEN_NOT 10

extern u16 USART_RX_STA; // 接收状态标记
extern u16 x_error, width, hight, distance;

// void uart_init(u32 bound);
void Openmv_Receive_Data_NOT(int16_t data);
void Openmv_Receive_Data(int16_t data);

#endif
