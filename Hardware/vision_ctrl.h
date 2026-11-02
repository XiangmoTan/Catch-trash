/**
 * @file  vision_ctrl.h
 * @brief 视觉伺服执行器：接收 OpenMV 的 v/w 速度帧，直接驱动四路 PWM
 *
 *  帧格式（由 OpenMV 的 Car_For_Ball_3_PID.py 发送）：
 *      0x2C 0x13 v w state chk 0x5B
 *          v     : int8，-100..100，前进为正
 *          w     : int8，-100..100，左转为正
 *          state : 0 IDLE / 1 SEARCH / 2 APPROACH / 3 PUSH / 4 DONE / 5 BACK
 *          chk   : 0x2C ^ 0x13 ^ v ^ w ^ state
 *
 *  用法：
 *      1) 在 USART1 中断里调用 Vision_Receive_Data(Res);
 *      2) 在模式 4 的主循环里反复调用 VisionCtrl_Task();（内部自带 20ms 节拍）
 */
#ifndef __VISION_CTRL_H
#define __VISION_CTRL_H

#include "stm32f10x.h"
#include "Serial.h"

void Vision_Receive_Data(uint8_t data);
void VisionCtrl_DisplayBlink(uint8_t mode);
void VisionCtrl_Task(void);
void VisionCtrl_Reset(void);
uint8_t VisionCtrl_HasNewFrame(void);   /* 新增：返回 1 表示刚收到一帧有效数据 */
uint8_t VisionCtrl_GetDisplayNum(void); /* 新增：返回要显示的数字 */

#endif /* __VISION_CTRL_H */
