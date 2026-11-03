/**
 * @file  ultrasonic.h
 * @brief HC-SR04 超声波测距（用于前进方向避障）
 *
 * 接线（默认值，改线只需改 ultrasonic.c 顶部的三个宏）：
 *      HC-SR04          STM32F103C8
 *      VCC    ────────  5V
 *      GND    ────────  GND
 *      TRIG   ────────  PB10   （STM32 输出，12us 高电平触发）
 *      ECHO   ────────  PB11   （模块输出，高电平宽度 = 声波往返时间）
 *
 * 关于 5V 电平：
 *      PB10 / PB11 在 STM32F103 上属于 FT（5V 兼容）引脚，ECHO 可以直接接。
 *      如果板子标注不允许 5V，或想更保险，在 ECHO 和 PB11 之间串
 *      1kΩ + 2kΩ 分压（ECHO—1k—PB11—2k—GND），把 5V 降到 3.3V。
 *
 * 用法：
 *      1) main() 里调用一次 Ultrasonic_Init()（或在 Temp=2 首次调用前初始化）；
 *      2) 在 20ms 控制循环里调用 Ultrasonic_Task()，内部自动每 60ms 测一次；
 *      3) 用 Ultrasonic_GetMm() 读“滤波后的距离”，单位 mm；0 表示没有回波/无效。
 */
#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"

void Ultrasonic_Init(void);
void Ultrasonic_Task(void);       /* 周期调用，内部自带 60ms 节流 */
uint16_t Ultrasonic_GetMm(void);  /* 最近一次滤波结果，0 = 无效 */
uint16_t Ultrasonic_ReadMm(void); /* 立即测一次（阻塞 ≤ 30ms），标定/排查用 */

#endif /* __ULTRASONIC_H */
