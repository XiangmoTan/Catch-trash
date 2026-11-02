/**
 * @file  vision_ctrl.c
 * @brief 视觉伺服执行器（Catch-trash 工程用）
 *
 *  OpenMV 已经把 PID 算好了，这里只做三件事：
 *      1) 从 USART1 中断里解析 0x2C 0x13 速度帧
 *      2) 把 v/w 拆成左右轮速度：left = v - w，right = v + w
 *      3) 带死区补偿地写到 TIM4 四路 PWM（robot_speed）
 *
 *  安全：400ms 收不到新帧就停车。
 */
#include "vision_ctrl.h"
#include "Delay.h"
#include "LEDSEG.h"
#include "robot.h"

#define FRAME_A 0x2C
#define FRAME_B 0x13
#define FRAME_END 0x5B

#define CTRL_PERIOD_MS 20 /* 50Hz */
#define LOST_TICKS 20     /* 20 * 20ms = 400ms 没新帧就停 */
#define PWM_DEADBAND 20   /* 小于这个占空比电机不转 */
#define PWM_MAX 100

/* ---------------- 解析状态 ---------------- */
static volatile int8_t s_v = 0;          // 解析成功后的线速度，供主任务读取（-100~100）
static volatile int8_t s_w = 0;          // 解析成功后的角速度，供主任务读取（-100~100）
static volatile uint8_t s_state = 0;     // 解析成功后的状态码（0~5，对应 OpenMV 状态机）
static volatile uint16_t s_seq = 0;      // 成功帧计数器，每收到一帧有效数据 +1，用于判断数据是否刷新
static volatile uint8_t s_new_frame = 0; // 收到新有效帧置1
static volatile uint32_t s_last_ms = 0;  // 最近一次收帧的 tick（用 SysTick 或主循环计数）

static uint8_t p_state = 0;
static uint8_t p_idx = 0; // 当前正在接收第几个数据字节（0~3）
static uint8_t p_buf[4];  // 暂存收到的 4 个数据字节：v、w、state、chk

// 闪烁参数
#define BLINK_HALF_MS 1000  /* 亮 1s / 灭 1s */
#define DISPLAY_LOST_MS 400 /* 400ms 无新帧 → 停止闪烁、常亮 */

extern volatile uint32_t g_ms_tick;

/* ---------------- 内部函数 ---------------- */
static int16_t clamp100(int16_t x)
{
    if (x > PWM_MAX)
    {
        return PWM_MAX;
    }
    if (x < -PWM_MAX)
    {
        return -PWM_MAX;
    }
    return x;
}

static int16_t deadband(int16_t x)
{
    if (x > 0 && x < PWM_DEADBAND)
    {
        return PWM_DEADBAND;
    }
    if (x < 0 && x > -PWM_DEADBAND)
    {
        return -PWM_DEADBAND;
    }
    return x;
}

/* 左右轮速度（-100..100）-> TIM4 CH1~CH4 */
static void set_wheels(int16_t left, int16_t right)
{
    uint8_t lf = 0, lr = 0, rf = 0, rr = 0;

    left = deadband(clamp100(left));
    right = deadband(clamp100(right));

    if (left > 0)
    {
        lf = (uint8_t)left;
    }
    else if (left < 0)
    {
        lr = (uint8_t)(-left);
    }

    if (right > 0)
    {
        rf = (uint8_t)right;
    }
    else if (right < 0)
    {
        rr = (uint8_t)(-right);
    }
    // Digital_Display(rr % 10);
    robot_speed(lf, lr, rf, rr);
}

// 供外部读当前要显示的数字（比如状态机 state，或自定义数字）
uint8_t VisionCtrl_GetDisplayNum(void)
{
    return (uint8_t)s_state; // 你想闪哪个数就改这里
}

uint8_t VisionCtrl_HasNewFrame(void)
{
    if (s_new_frame)
    {
        s_new_frame = 0;
        return 1;
    }
    return 0;
}

/* ---------------- 对外接口 ---------------- */
void Vision_Receive_Data(uint8_t data)
{
    uint8_t chk;

    switch (p_state) // 解析状态机当前所处阶段（0 等帧头A、1 等帧头B、2 收数据体）
    {
    case 0:
        if (data == FRAME_A)
        {
            p_state = 1;
        }
        break;

    case 1:
        if (data == FRAME_B)
        {
            p_state = 2;
            p_idx = 0;
        }
        else if (data != FRAME_A)
        {
            p_state = 0;
        }
        break;

    case 2:
        p_buf[p_idx] = data;
        p_idx++;
        if (p_idx >= 4)
        {
            p_state = 0;
            chk = (uint8_t)(FRAME_A ^ FRAME_B ^ p_buf[0] ^ p_buf[1] ^ p_buf[2]);
            if (p_buf[3] == chk) // 检查校验位
            {
                s_v = (int8_t)p_buf[0];
                s_w = (int8_t)p_buf[1];
                s_state = p_buf[2];
                s_seq++;
                s_new_frame = 1; // 标记有新帧
            }
        }
        break;

    default:
        p_state = 0;
        break;
    }
}

void VisionCtrl_Reset(void)
{
    p_state = 0;
    p_idx = 0;
    s_v = 0;
    s_w = 0;
    s_state = 0;
    s_seq = 0;
    robot_speed(0, 0, 0, 0);
}

void VisionCtrl_Task(void)
{
    static uint16_t seen = 0;
    static uint16_t lost = 0;
    int16_t v, w;

    if (s_seq != seen)
    {
        seen = s_seq;
        lost = 0;
    }
    else if (lost < 1000)
    {
        lost++;
    }

    // Serial_Printf("seq=%d lost=%d v=%d w=%d\r\n", s_seq, lost, s_v, s_w);

    /* 掉线保护：400ms 没有新帧就停 */
    if (lost > LOST_TICKS)
    {
        robot_speed(0, 0, 0, 0);
        Delay_ms(CTRL_PERIOD_MS);
        return;
    }

    v = (int16_t)s_v;
    w = (int16_t)s_w;

    /* 差速：w>0 左转 -> 左轮慢、右轮快 */
    set_wheels(v - w, v + w);

    Delay_ms(CTRL_PERIOD_MS);
}

void VisionCtrl_DisplayBlink(uint8_t mode)
{
    static uint8_t phase = 0; /* 0=亮 1=灭 */
    static uint32_t last_toggle = 0;
    static uint32_t last_frame = 0;
    static uint8_t has_frame = 0; /* 是否收到过至少一帧 */

    /* 只要收到新帧，就刷新“最后一次收帧时间” */
    if (VisionCtrl_HasNewFrame())
    {
        has_frame = 1;
        last_frame = g_ms_tick;
    }

    uint32_t now = g_ms_tick;

    /* 通信正常 = 收到过帧 且 最近 DISPLAY_LOST_MS 内还有新帧 */
    uint8_t comm_ok = has_frame && ((now - last_frame) < DISPLAY_LOST_MS);

    if (mode == 2 && comm_ok)
    {
        /* 模式 2 且通信正常：2 闪烁 */
        if ((now - last_toggle) >= BLINK_HALF_MS)
        {
            last_toggle = now;
            phase = !phase;
        }
        Digital_Display(phase == 0 ? 2 : 10);
    }
    else
    {
        /* 其它所有情况：常亮显示 mode（含 temp=2 但通信异常 → 常亮 2） */
        phase = 0;
        last_toggle = now;
        Digital_Display(mode);
    }
}
