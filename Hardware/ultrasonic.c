/**
 * @file  ultrasonic.c
 * @brief HC-SR04 超声波测距（用于前进方向避障）
 *
 * 原理（一句话）：
 *      STM32 给 TRIG 一个 ≥10us 的高电平，模块就发 8 个 40kHz 的超声脉冲；
 *      模块把 ECHO 拉高的时间 = 声波“去 + 回”的时间。
 *      距离 = 声速 × 时间 / 2 = 343m/s × t / 2
 *      因为 1 tick = 1us，所以  mm = us × 343 / 2000。
 *
 * 计时为什么不用 Delay_us：
 *      Delay_us 用的是 SysTick，而且只能测“固定时长”；这里要测“一个外部
 *      脉冲有多宽”，所以把 TIM1 配成 1MHz 自由计数（1 tick = 1us），
 *      读两次 CNT 相减即可。TIM1 在这个工程里没有别的用途。
 */
#include "ultrasonic.h"
#include "Delay.h"

/* ===================== 接线与参数（改线只改这里） ===================== */
#define US_PORT        GPIOB
#define US_RCC         RCC_APB2Periph_GPIOB
#define US_TRIG_PIN    GPIO_Pin_10
#define US_ECHO_PIN    GPIO_Pin_11

#define US_TRIG_US      12      /* 触发脉冲宽度，HC-SR04 要求 ≥10us */
#define US_TIMEOUT_RISE 2000    /* 等 ECHO 上升沿的最长时间，单位 us */
#define US_TIMEOUT_HIGH 25000   /* ECHO 高电平最长时间，25ms ≈ 4.3m */
#define US_MIN_MM       20      /* 小于 2cm 当噪声（HC-SR04 近距盲区） */
#define US_MAX_MM       4000    /* 大于 4m 当无效 */
#define US_PERIOD_MS    60      /* 测距周期：60ms 一次，够快且不抢控制周期 */
#define US_MEDIAN_N     3       /* 中值滤波窗口：3 次取中间值，滤掉单次毛刺 */
#define US_MISS_RESET   3       /* 连续 3 次没回波 -> 判定“没有障碍信息” */

/* ===================== 内部状态 ===================== */
static uint16_t s_hist[US_MEDIAN_N]; /* 最近 3 次有效距离 */
static uint8_t  s_hist_i = 0;        /* 环形写入位置 */
static uint16_t s_mm = 0;            /* 滤波后的距离，0 = 无效 */
static uint32_t s_last_ms = 0;       /* 上次测量时刻 */
static uint8_t  s_miss = 0;          /* 连续无回波次数 */
static uint8_t  s_inited = 0;        /* 防止没人调用 Init 时直接读寄存器 */

/**
 * @brief  初始化 TRIG/ECHO 引脚和 TIM1 微秒计时器
 */
void Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(US_RCC | RCC_APB2Periph_TIM1, ENABLE);

    /* TRIG：推挽输出，空闲低电平 */
    GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(US_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(US_PORT, US_TRIG_PIN);

    /* ECHO：下拉输入。模块没接时读到 0，不会误判成“一直有回波” */
    GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(US_PORT, &GPIO_InitStructure);

    /* TIM1：72MHz / 72 = 1MHz，自由运行（0~65535us 循环），不开中断 */
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM1, ENABLE);

    s_hist[0] = 0;
    s_hist[1] = 0;
    s_hist[2] = 0;
    s_mm = 0;
    s_miss = 0;
    s_inited = 1;
}

/**
 * @brief  立即测一次距离（阻塞：最长约 2ms 等上升沿 + 25ms 等回波）
 * @retval 距离，单位 mm；0 = 超时 / 超量程 / 没接模块
 */
uint16_t Ultrasonic_ReadMm(void)
{
    uint16_t t0, width;
    uint32_t mm;

    if (!s_inited)
    {
        Ultrasonic_Init();
    }

    /* 1) 触发：拉高 ≥10us 再拉低 */
    GPIO_SetBits(US_PORT, US_TRIG_PIN);
    Delay_us(US_TRIG_US);
    GPIO_ResetBits(US_PORT, US_TRIG_PIN);

    /* 2) 等 ECHO 变高（模块发完 8 个脉冲才会拉高，通常 <1ms） */
    t0 = (uint16_t)TIM1->CNT;
    while (GPIO_ReadInputDataBit(US_PORT, US_ECHO_PIN) == Bit_RESET)
    {
        if ((uint16_t)(TIM1->CNT - t0) > US_TIMEOUT_RISE)
        {
            return 0; /* 没有回波：障碍太远、角度太偏或模块没接好 */
        }
    }

    /* 3) 量 ECHO 高电平宽度。16 位无符号相减自动处理回绕，
     *    只要脉宽 < 65ms（约 11m）就不会错 */
    t0 = (uint16_t)TIM1->CNT;
    while (GPIO_ReadInputDataBit(US_PORT, US_ECHO_PIN) == Bit_SET)
    {
        if ((uint16_t)(TIM1->CNT - t0) > US_TIMEOUT_HIGH)
        {
            return 0;
        }
    }
    width = (uint16_t)(TIM1->CNT - t0);

    /* 4) us -> mm：343m/s = 0.343mm/us，往返除以 2 */
    mm = ((uint32_t)width * 343UL) / 2000UL;
    if (mm < US_MIN_MM || mm > US_MAX_MM)
    {
        return 0;
    }
    return (uint16_t)mm;
}

/**
 * @brief  取有效样本的中值（0 表示还没有有效样本）
 */
static uint16_t median_valid(void)
{
    uint16_t v[US_MEDIAN_N];
    uint8_t n = 0, i, j;
    uint16_t t;

    for (i = 0; i < US_MEDIAN_N; i++)
    {
        if (s_hist[i] > 0)
        {
            v[n++] = s_hist[i];
        }
    }
    if (n == 0)
    {
        return 0;
    }

    /* 样本最多 3 个，插入排序足够 */
    for (i = 1; i < n; i++)
    {
        t = v[i];
        for (j = i; j > 0 && v[j - 1] > t; j--)
        {
            v[j] = v[j - 1];
        }
        v[j] = t;
    }
    return v[n / 2];
}

/**
 * @brief  周期任务：每 60ms 测一次，做中值滤波
 *         放在 VisionCtrl_Task() 的 20ms 循环里调用即可
 */
void Ultrasonic_Task(void)
{
    uint32_t now;
    uint16_t d;

    if (!s_inited)
    {
        Ultrasonic_Init();
    }

    now = g_ms_tick;
    if ((uint32_t)(now - s_last_ms) < US_PERIOD_MS)
    {
        return;
    }
    s_last_ms = now;

    d = Ultrasonic_ReadMm();

    if (d > 0)
    {
        s_miss = 0;
        s_hist[s_hist_i] = d;
        s_hist_i = (uint8_t)((s_hist_i + 1) % US_MEDIAN_N);
        s_mm = median_valid();
    }
    else
    {
        /* 连续多次没回波（模块掉了/前方太空旷）：清空历史，
         * 否则会一直拿着“最后看到的障碍”做避障，车永远走不动 */
        if (++s_miss >= US_MISS_RESET)
        {
            s_miss = US_MISS_RESET;
            s_hist[0] = 0;
            s_hist[1] = 0;
            s_hist[2] = 0;
            s_mm = 0;
        }
    }
}

/**
 * @brief  最近一次滤波后的距离，单位 mm；0 = 无效（没有障碍信息）
 */
uint16_t Ultrasonic_GetMm(void)
{
    return s_mm;
}
