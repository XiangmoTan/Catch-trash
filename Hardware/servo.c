#include "servo.h"
#include "Delay.h"
#include "stm32f10x.h"
#include <stdlib.h>
/************************************************
                  舵机模块
************************************************/
void Servo_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1; // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1; // PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // CCR
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

void Servo_SetAngle(float Angle, float Angle_left)
{
    TIM_SetCompare3(TIM3, Angle / 180 * 2000 + 500);
    TIM_SetCompare4(TIM3, (180 - Angle_left) / 180 * 2000 + 500);
}

// 360°舵机“伪角度”控制（开环）
// 假设：全速转动下，转180度大约需要 2秒（2000ms），具体需实测校准
void Servo360_SetAngle_Fake(float target_angle)
{
    uint16_t pulse;
    if (target_angle < 0)
        target_angle = 0;
    if (target_angle > 180)
        target_angle = 180;

    // 90度对应停止(1.5ms)，小于90反转，大于90正转
    if (target_angle == 90)
    {
        pulse = 1500; // 停止
    }
    else if (target_angle < 90)
    {
        // 映射为 500~1500 的脉宽（反转）
        pulse = (uint16_t)(500 + (target_angle / 90.0) * 1000);
    }
    else
    {
        // 映射为 1500~2500 的脉宽（正转）
        pulse = (uint16_t)(1500 + ((target_angle - 90) / 90.0) * 1000);
    }

    // 设置转速方向
    TIM_SetCompare1(TIM3, pulse);

    // 用延时来估算转动角度（极度不准，受电压/负载影响）
    // 举例：转到位大约需要的时间
    uint32_t delay_time = (uint32_t)((abs((int)(target_angle - 90)) / 180.0) * 2000);
    Delay_ms(delay_time);

    // 到达“估算”位置后停止
    TIM_SetCompare1(TIM3, 1500);
}
