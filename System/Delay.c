#include "Delay.h"
#include "stm32f10x.h"

volatile uint32_t g_ms_tick = 0;

/**
 * @brief  微秒级延时
 * @param  xus 延时时长，范围：0~233015
 * @retval 无
 */
void Delay_us(uint32_t xus)
{
    SysTick->LOAD = 72 * xus;   // 设置定时器重装值
    SysTick->VAL = 0x00;        // 清空当前计数值
    SysTick->CTRL = 0x00000005; // 设置时钟源为HCLK，启动定时器
    while (!(SysTick->CTRL & 0x00010000))
        ;                       // 等待计数到0
    SysTick->CTRL = 0x00000004; // 关闭定时器
}

/**
 * @brief  毫秒级延时
 * @param  xms 延时时长，范围：0~4294967295
 * @retval 无
 */
void Delay_ms(uint32_t xms)
{
    while (xms--)
    {
        Delay_us(1000);
    }
}

/**
 * @brief  秒级延时
 * @param  xs 延时时长，范围：0~4294967295
 * @retval 无
 */
void Delay_s(uint32_t xs)
{
    while (xs--)
    {
        Delay_ms(1000);
    }
}

/* 1ms 系统节拍，使用 TIM2。
 * 注意：不能用 TIM3 —— Servo_Init() 会把 TIM3 重新配置成 50Hz 的舵机 PWM，
 * 一旦共用，g_ms_tick 就会变成 20ms 才加 1，闪烁和超时时间全部错乱。 */
void TimeBase_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef t;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    t.TIM_Period = 1000 - 1; /* 72MHz / 72 / 1000 = 1kHz */
    t.TIM_Prescaler = 72 - 1;
    t.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &t);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef n;
    n.NVIC_IRQChannel = TIM2_IRQn;
    n.NVIC_IRQChannelCmd = ENABLE;
    n.NVIC_IRQChannelPreemptionPriority = 2;
    n.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&n);

    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        g_ms_tick++;
    }
}
