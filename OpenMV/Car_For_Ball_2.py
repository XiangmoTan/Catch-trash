# import pyb, ustruct
# import image
import sensor, time  # 导入摄像头，机器视觉，时间模块
from pyb import UART, LED    # 从 pyb 中导入 UART（串口）模块，LED 模块
# import json
from machine import Pin

# 摄像头设置
sensor.reset()  # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565)  # use RGB565.
sensor.set_framesize(sensor.QQVGA)  # use QQVGA for speed.
sensor.skip_frames(20)  # Let new settings take affect.
sensor.set_auto_whitebal(False)  # turn this off.
clock = time.clock()  # Tracks FPS.

red_threshold = (46, 95, -67, -25, -21, 35)  # 颜色阈值设置（此处为红色）

# 定义超声波引脚
TRIGGER_PIN = Pin('P8', Pin.OUT_PP)  # 触发信号引脚
ECHO_PIN = Pin('P7', Pin.IN)  # 回波信号引脚

# 串口初始化
uart = UART(3, 115200)  # 定义串口3变量
uart.init(115200, bits=8, parity=None, stop=1)  # 串口初始化设置

# 打开 LED 指示灯
LED(1).on()
LED(2).on()
LED(3).on()

# 定义超声波距离计算函数
# def distance():
#     TRIGGER_PIN.low()
#     time.sleep_us(2)
#     TRIGGER_PIN.high()
#     time.sleep_us(10)
#     TRIGGER_PIN.low()

#     timeout = 5000  # 超时时间（微秒）
#     while not ECHO_PIN.value() and timeout > 0:  # 等待高电平开始
#         timeout -= 1
#     if timeout == 0:
#         print("未检测到回波信号开始")
#         return -1

#     pulse_start = time.ticks_us()

#     timeout = 5000
#     while ECHO_PIN.value() and timeout > 0:  # 等待高电平结束
#         timeout -= 1
#     if timeout == 0:
#         print("未检测到回波信号结束")
#         return -1

#     pulse_end = time.ticks_us()
#     pulse_duration = pulse_end - pulse_start

#     # 根据公式计算距离（单位：厘米）
#     distance_cm = (pulse_duration * 0.0343) / 2
#     return int(distance_cm)

from time import ticks_us, ticks_diff, sleep_us

def distance():
    TRIGGER_PIN.low()
    sleep_us(2)
    TRIGGER_PIN.high()
    sleep_us(10)
    TRIGGER_PIN.low()

    # 等待 ECHO 变高，超时 50ms
    start_us = ticks_us()
    while not ECHO_PIN.value():
        if ticks_diff(ticks_us(), start_us) > 50000:   # 50ms
            return -1

    pulse_start = ticks_us()
    while ECHO_PIN.value():
        if ticks_diff(ticks_us(), pulse_start) > 100000:  # 100ms
            return -1

    pulse_end = ticks_us()
    duration_us = ticks_diff(pulse_end, pulse_start)
    distance_cm = (duration_us * 0.0343) / 2
    return int(distance_cm)

# 多目标识别：矿泉水瓶 / 7号电池 / 一次性纸杯
# 适用硬件：OpenMV Cam H7 Plus
# 场景建议：摄像头固定，背景简洁，物体不互相遮挡

# ---------- 颜色阈值（LAB格式）----------
# 矿泉水瓶：红色标签或红色瓶盖
red_threshold   = [(0, 90, 30, 127, 20, 127)]

# 7号电池：黑色外壳 + 金色金属帽，两种阈值都试
black_threshold = [(0, 40, -20, 20, -20, 20)]
gold_threshold  = [(50, 100, 10, 60, 40, 127)]

# 纸杯：白色（备用方案，主要靠圆形检测）
white_threshold = [(80, 100, -10, 10, -10, 10)]


# ---------- 识别矿泉水瓶 ----------
def detect_bottle(img):
    blobs = img.find_blobs(red_threshold,
                           pixels_threshold=100,
                           area_threshold=100,
                           merge=True)
    best = None
    best_score = 0

    for b in blobs:
        area = b.area()
        elong = b.elongation()
        if area < 300:
            continue
        score = area * elong
        if score > best_score:
            best_score = score
            best = b

    if best is not None and best.elongation() > 0.3:
        # 返回 int(x), int(y), int(w), int(h)
        return (int(best.x()), int(best.y()), int(best.w()), int(best.h()))
    return None

# ---------- 识别7号电池 ----------
def detect_battery(img):
    thresholds = [black_threshold, gold_threshold]

    for th in thresholds:
        blobs = img.find_blobs(th,
                               pixels_threshold=50,
                               area_threshold=30,
                               merge=True)
        for b in blobs:
            area = b.area()
            elong = b.elongation()
            if 100 < area < 1500 and elong > 0.5:
                return (int(b.x()), int(b.y()), int(b.w()), int(b.h()))
    return None

# ---------- 识别一次性纸杯 ----------
def detect_cup(img):
    # 方法1：杯口圆形检测
    circles = img.find_circles(threshold=2000, r_min=15, r_max=60)

    if circles:
        c = max(circles, key=lambda c: c.r())
        x, y, r = c.x(), c.y(), c.r()

        # 验证亮度（白纸杯杯口亮度较高）
        stats = img.get_statistics(roi=(x - r, y - r, 2 * r, 2 * r))
        if stats.l_mean() > 60:
            # 返回外接矩形 (x-r, y-r, 2r, 2r)，全部取整
            return (int(x - r), int(y - r), int(2 * r), int(2 * r))

    # 方法2：备用，白色圆斑
    blobs = img.find_blobs(white_threshold,
                           pixels_threshold=200,
                           area_threshold=200,
                           merge=True)

    for b in blobs:
        if b.area() > 500 and b.roundness() > 0.6:
            return (int(b.x()), int(b.y()), int(b.w()), int(b.h()))
    return None


# 寻找最大色块函数
def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[2] * blob[3] > max_size:  # 面积 = 宽度 * 高度
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob

# 主循环
distance_buffer = []  # 滑动平均窗口

while True:
    dist = distance()  # 调用超声波测距函数
    if dist == -1:
        # print("测距失败，使用上一次平均值")
        dist = sum(distance_buffer) // len(distance_buffer) if len(distance_buffer) > 0 else -1

    # 更新滑动窗口
    if dist != -1:
        distance_buffer.append(dist)
        if len(distance_buffer) > 5:
            distance_buffer.pop(0)

    clock.tick()  # 更新时钟
    img = sensor.snapshot()  # 捕获图像帧

    bottle = detect_bottle(img)
    battery = detect_battery(img)
    cup = detect_cup(img)

    # 直接在图上画出每个目标的框
    if bottle:
        x, y, w, h = bottle
        img.draw_rectangle(x, y, w, h, color=(255, 0, 0))
        img.draw_string(x, max(0, y - 5), "Bottle", color=(255, 255, 255))
        # 数据打包发送
        checksum = x ^ w ^ h ^ dist
        data = bytearray([0x2C, 0x12, x, w, h, dist, checksum, 0x5B])
        uart.write(data)

        print("Bottle -> x:{} y:{} w:{} h:{}".format(x, y, w, h))

    if battery:
        x, y, w, h = battery
        img.draw_rectangle(x, y, w, h, color=(0, 255, 0))
        img.draw_string(x, max(0, y - 5), "Battery", color=(255, 255, 255))
        # 数据打包发送
        checksum = x ^ w ^ h ^ dist
        data = bytearray([0x2C, 0x12, x, w, h, dist, checksum, 0x5B])
        uart.write(data)
        print("Battery -> x:{} y:{} w:{} h:{}".format(x, y, w, h))

    if cup:
        x, y, w, h = cup
        img.draw_rectangle(x, y, w, h, color=(0, 0, 255))
        img.draw_string(x, max(0, y - 5), "Cup", color=(255, 255, 255))
        # 数据打包发送
        checksum = x ^ w ^ h ^ dist
        data = bytearray([0x2C, 0x12, x, w, h, dist, checksum, 0x5B])
        uart.write(data)
        print("Cup -> x:{} y:{} w:{} h:{}".format(x, y, w, h))

    if not bottle and not battery and not cup:
        checksum = dist ^ 0x22
        data1 = bytearray([0x3C, 0x22, dist, checksum, 0x6B])
        uart.write(data1)
        print("None")

    # blobs = img.find_blobs([red_threshold])  # 调用寻找色块函数

    # if detected:  # 如果检测到色块
    #     max_blob = find_max(detected)  # 找到最大色块
    #     x_error = round(max_blob[5])  # 横坐标取整数
    #     y_error = round(max_blob[6])
    #     width = round(max_blob[2])  # 宽度取整数
    #     hight = round(max_blob[3])  # 高度取整数

    #     # 绘制目标区域
    #     img.draw_rectangle(max_blob[0:4])
    #     img.draw_cross(max_blob[5], max_blob[6])

        # 打印调试信息
        # print(f"x_error: {x_error}, width: {width}, height: {hight}, distance: {dist} cm")



    # else:  # 如果未检测到色块
    #     # print(f"0x3c distance: {dist} cm")
    #     checksum = dist ^ 0x22
    #     data1 = bytearray([0x3C, 0x22, dist, checksum, 0x6B])
    #     uart.write(data1)

        # time.sleep_ms(20)
        # if uart.any():
        #     echo = uart.read()
        #     print("Echo:", echo)
        # else:
        #     print("No echo!")
