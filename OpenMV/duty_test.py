# -*- coding: utf-8 -*-
# ============================================================
# 电机最小占空比标定脚本（在 OpenMV 上运行，不是 STM32）
#
# 分两段：
#   A. 架空测试：轮子离地，先确认四个方向都能转、左右方向没接反
#   B. 地面测试：前进 / 后退 / 原地转 / 弧线转，找最小占空比
#
# 用法：
#   1) STM32 上电，按一下按键进入 temp=2（数码管闪 = 通信正常）
#   2) 先【把车架空】再运行本脚本，确认四个方向的轮子都转
#   3) 终端出现“请把车放回地面”后，把车放好，等 5 秒自动继续
#   4) 记录下面四个值，填回程序：
#        - 前进第一次动      -> Car_For_Ball.py 的 V_CREEP
#        - 后退第一次动      -> V_BACK
#        - 原地转第一次动    -> W_SEARCH / W_MIN_TURN / W_MAX
#        - 后退+弧线第一次动 -> TURN_ARC_V（原地转不动时用这个）
#
# 关键判断：
#   * 架空时轮子转、落地不转  -> 阻力/扭矩不够，不是接线问题，
#     继续往上加占空比，或用弧线转。
#   * 架空时某个方向就不转   -> 接线/驱动板通道问题，先查硬件。
#   * 原地转 100% 都不动     -> 这台车基本无法原地转，必须用弧线
#     （TURN_ARC_V > 0），不要死磕。
#
# 注意：电池电压下降后最小占空比会变大，请在“比赛电量”下测。
# ============================================================

import time
from pyb import UART, LED

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

LED(1).on()
LED(2).on()

# 架空测试：轮子离地时设为 True；只想直接测地面就设 False
AIR_TEST = True


def send_vel(v, w, st=9):
    """和 Car_For_Ball.py 完全相同的帧格式"""
    vi = int(v)
    wi = int(w)
    if vi > 100:
        vi = 100
    elif vi < -100:
        vi = -100
    if wi > 100:
        wi = 100
    elif wi < -100:
        wi = -100

    vb = vi & 0xFF
    wb = wi & 0xFF
    chk = (0x2C ^ 0x13 ^ vb ^ wb ^ (st & 0xFF)) & 0xFF
    uart.write(bytearray([0x2C, 0x13, vb, wb, st & 0xFF, chk, 0x5B]))


def hold(v, w, ms, gap_ms=400):
    """持续发送同一速度 ms 毫秒，然后停车 gap_ms 毫秒"""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        send_vel(v, w)
        time.sleep_ms(50)
    send_vel(0, 0)
    time.sleep_ms(gap_ms)


# ---------- 0. 等通信稳定 ----------
print("duty test: 3 秒后开始，确保 STM32 已进入 temp=2")
for i in range(3, 0, -1):
    print(i)
    time.sleep_ms(1000)


# ---------- A. 架空检查 ----------
# 目的：把“接线/方向”和“扭矩不够”分开。
# 如果这里轮子都不转，后面地面怎么加占空比都没用，先修硬件。
if AIR_TEST:
    print("")
    print("=========== A. 架空检查：请把车拿起来，轮子离地 ===========")
    time.sleep_ms(3000)

    print("A1. 前进 v=70（两轮都应向前）")
    hold(70, 0, 1200)
    print("A2. 后退 v=-70（两轮都应向后）")
    hold(-70, 0, 1200)
    print("A3. 原地左转 w=70（左轮向后、右轮向前）")
    hold(0, 70, 1200)
    print("A4. 原地右转 w=-70（左轮向前、右轮向后）")
    hold(0, -70, 1200)

    print("")
    print("★ 如果 A3/A4 轮子不转或方向不对，先查电机线/驱动板，不要往下测")
    print("★ 请把车放回地面，5 秒后开始地面测试")
    time.sleep_ms(5000)


# ---------- B. 地面测试 ----------

# ---------- B1. 前进 ----------
print("")
print("=========== B1. 前进 ===========")
for v in range(20, 101, 5):
    print("forward v=%d" % v)
    hold(v, 0, 1200)

# ---------- B2. 后退 ----------
print("")
print("=========== B2. 后退 ===========")
for v in range(20, 101, 10):
    print("backward v=-%d" % v)
    hold(-v, 0, 1200)

# ---------- B3. 原地左转（重点：这次一直扫到 100） ----------
print("")
print("=========== B3. 原地左转 (w>0) ===========")
for w in range(40, 101, 10):
    print("spin left w=%d" % w)
    hold(0, w, 1500)

# ---------- B4. 原地右转 ----------
print("")
print("=========== B4. 原地右转 (w<0) ===========")
for w in range(40, 101, 10):
    print("spin right w=-%d" % w)
    hold(0, -w, 1500)

# ---------- B5. 弧线左转（原地转不动时用这个） ----------
# v 固定为 55（你的实测前进最小是 55，可按需改），
# 只改 w，看多大的 w 能让车明显转弯。
print("")
print("=========== B5. 弧线左转 (v=55, w>0) ===========")
for w in range(20, 61, 10):
    print("arc left  v=55 w=%d" % w)
    hold(55, w, 1500)

# ---------- B6. 弧线右转 ----------
print("")
print("=========== B6. 弧线右转 (v=55, w<0) ===========")
for w in range(20, 61, 10):
    print("arc right v=55 w=-%d" % w)
    hold(55, -w, 1500)


# ---------- 5. 停车 ----------
send_vel(0, 0)
print("")
print("测试结束，已停车")
print("请记录：前进最小 / 后退最小 / 原地转最小 / 弧线转最小")
