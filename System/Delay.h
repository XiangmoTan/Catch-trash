#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f10x.h"

extern volatile uint32_t g_ms_tick;

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);
void TimeBase_Init(void);
#endif
