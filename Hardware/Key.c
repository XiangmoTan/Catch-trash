#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* 非阻塞按键：检测下降沿（按下），20ms 去抖。
 * 旧版用 while 等待松手，一旦 PA15 被拉低（接线错/按键卡住/JTAG 占用），
 * 整个主循环会卡死，数码管就永远不刷新了。 */
uint8_t Key_GetNum(void)
{
	static uint8_t last_level = 1;
	static uint32_t last_change = 0;
	uint8_t KeyNum = 0;
	uint8_t now_level = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);
	uint32_t now = g_ms_tick;

	if (now_level != last_level)
	{
		if ((now - last_change) >= 20) /* 电平稳定 20ms 才认 */
		{
			last_level = now_level;
			last_change = now;
			if (now_level == 0) /* 按下沿 */
			{
				KeyNum = 1;
			}
		}
	}
	else
	{
		last_change = now;
	}
	return KeyNum;
}
