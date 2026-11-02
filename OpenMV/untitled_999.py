# Untitled - By: 23659 - Wed Oct 28 2026

import sensor, time
from pyb import UART, LED
from machine import Pin

# ============================================================
# 1. 摄像头 / 串口 / 超声波
# ============================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)      # 160x120，速度快
sensor.skip_frames(20)
sensor.set_auto_whitebal(False)
try:
    sensor.set_auto_gain(False)          # 颜色阈值才稳定
except Exception:
    pass

clock = time.clock()

IMG_W, IMG_H = 160, 120
IMG_CX = IMG_W // 2                       # 画面中心 80

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

# 超声波（可选，默认不用：它会阻塞，拖慢视觉环）
USE_ULTRASONIC = False
TRIGGER_PIN = Pin('P8', Pin.OUT_PP)
ECHO_PIN = Pin('P7', Pin.IN)

LED(1).on()
LED(2).on()
LED(3).on()

# ============================================================
# 2. 颜色阈值（沿用原工程，现场可再标）
# ============================================================
red_threshold   = [(58, 20, 14, 75, 66, -4)]      # 矿泉水瓶（红标）
black_threshold = [(0, 40, -20, 20, -20, 20)]      # 7号电池（黑壳）
gold_threshold  = [(50, 100, 10, 60, 40, 127)]     # 7号电池（金帽）
white_threshold = [(48, 63, 18, -15, 27, -6)]    # 纸杯（白）

# 目标色块区域（默认绿色，改成你场地上那个区域的颜色）
area_color = [(57, 73, 40, -18, 5, -37)]

# 指定要抓的物体：0=任意  1=矿泉水瓶  2=7号电池  3=纸杯
TARGET_TYPE = 0

# ============================================================
# 3. 控制参数
# ============================================================
CTRL_PERIOD_MS   = 50        # 20Hz 发一帧
SEND_EVERY_MS    = 50

V_MAX            = 100        # PWM 百分比量级
V_PUSH           = 60
V_CREEP          = 50        # 最小爬行速度
V_BACK           = 50
W_MAX            = 35
W_SEARCH         = 22

AREA_PUSH        = 1200      # w*h 达到这个认为“到物体面前”（默认/兜底）
# 不同物体在画面里的面积差很多，按类型分开设：
AREA_PUSH_KIND   = {1: 2500, 2: 600, 3: 2000}   # 1瓶子 2电池 3纸杯
AREA_MIN         = 60        # 太小的不要
AREA_MAX         = 9000

PUSH_MS          = 2200      # 推箱持续时间
PUSH_TIMEOUT_MS  = 9000
DONE_MS          = 600
BACK_MS          = 700

ALIGN_ERR_PX     = 28        # 偏大于这个先原地转
PUSH_BIAS_K      = 0.45      # 目标色块偏置增益
PUSH_BIAS_MAX    = 22

# PID 增益（输出直接是 PWM 百分比量级）
PID_X_KP, PID_X_KI, PID_X_KD = 0.50, 0.02, 0.12
PID_X_I_LIMIT = 25.0

PID_V_KP = 0.050             # 每像素面积 -> PWM
PID_V_I  = 0.0004
PID_V_I_LIMIT = 12.0

# 状态
ST_IDLE, ST_SEARCH, ST_APPROACH, ST_PUSH, ST_DONE, ST_BACK = 0, 1, 2, 3, 4, 5
ST_NAME = ["IDLE", "SEARCH", "APPROACH", "PUSH", "DONE", "BACK"]


# ============================================================
# 4. 固件兼容封装
# ============================================================
def vget(obj, name, default=0):
    """blob / circle 的字段：新固件是属性，旧固件是方法"""
    try:
        val = getattr(obj, name)
    except Exception:
        return default
    if callable(val):
        try:
            val = val()
        except Exception:
            return default
    try:
        return int(val)
    except Exception:
        return default


def draw_text(img, x, y, s, color=(255, 255, 255)):
    """draw_string 两种签名都试一遍"""
    try:
        img.draw_string((x, y), s, color=color)
        return
    except Exception:
        pass
    try:
        img.draw_string(x, y, s, color=color)
    except Exception:
        pass


def blob_geom(b):
    w = vget(b, 'w')
    h = vget(b, 'h')
    px = vget(b, 'pixels')
    x = vget(b, 'x')
    y = vget(b, 'y')
    if w < 1 or h < 1:
        return None
    return x, y, w, h, px


# ============================================================
# 5. 目标检测
# ============================================================
def _best(img, thresholds, min_px, aspect_min=0.0, aspect_max=99.0):
    """在一组阈值里找面积最大、且长宽比合适的色块"""
    best = None
    best_px = 0
    blobs = img.find_blobs(thresholds,
                           pixels_threshold=min_px,
                           area_threshold=min_px,
                           merge=True)
    for b in blobs:
        g = blob_geom(b)
        if g is None:
            continue
        x, y, w, h, px = g
        if px < min_px:
            continue
        lo = w if w < h else h
        hi = h if w < h else w
        ar = float(hi) / float(lo)
        if ar < aspect_min or ar > aspect_max:
            continue
        if px > best_px:
            best_px = px
            best = (x, y, w, h, px)
    return best


def detect_bottle(img):
    r = _best(img, red_threshold, 200, 1.2, 99.0)
    if r is None:
        return None
    return (r[0], r[1], r[2], r[3], r[4], 1)


def detect_battery(img):
    r = _best(img, black_threshold, 60, 1.6, 8.0)
    if r is None:
        r = _best(img, gold_threshold, 60, 1.6, 8.0)
    if r is None:
        return None
    return (r[0], r[1], r[2], r[3], r[4], 2)


def detect_cup(img):
    # 先按白色色块找
    r = _best(img, white_threshold, 250, 0.55, 2.2)
    if r is not None:
        return (r[0], r[1], r[2], r[3], r[4], 3)
    # 再用杯口圆形兜底
    try:
        circles = img.find_circles(threshold=2000, r_min=8, r_max=55)
    except Exception:
        return None
    best = None
    best_r = 0
    for c in circles:
        rr = vget(c, 'r')
        if rr > best_r:
            best_r = rr
            best = (vget(c, 'x') - rr, vget(c, 'y') - rr, 2 * rr, 2 * rr,
                    int(3.14159 * rr * rr), 3)
    return best


def detect_target(img):
    """返回 (cx, cy, w, h, area, kind) 或 None"""
    cands = []
    if TARGET_TYPE in (0, 1):
        b = detect_bottle(img)
        if b:
            cands.append(b)
    # if TARGET_TYPE in (0, 2):
    #     b = detect_battery(img)
    #     if b:
    #         cands.append(b)
    # if TARGET_TYPE in (0, 3):
    #     b = detect_cup(img)
    #     if b:
    #         cands.append(b)

    best = None
    best_px = 0
    for (x, y, w, h, px, kind) in cands:
        if px < AREA_MIN or px > AREA_MAX:
            continue
        if px > best_px:
            best_px = px
            best = (x, y, w, h, px, kind)
            print(kind)
    if best is None:
        return None
    x, y, w, h, px, kind = best
    return (x + w // 2, y + h // 2, w, h, px, kind)


def find_dest_area(img):
    """目标色块区域：返回 (cx, pixels) 或 None"""
    best = None
    best_px = 0
    blobs = img.find_blobs(area_color,
                           pixels_threshold=400,
                           area_threshold=400,
                           merge=True)
    for b in blobs:
        g = blob_geom(b)
        if g is None:
            continue
        x, y, w, h, px = g
        if px > 30000:
            continue
        if px > best_px:
            best_px = px
            best = (x + w // 2, px)
    return best


# ============================================================
# 6. PID
# ============================================================
class PID(object):
    def __init__(self, kp, ki, kd, out_min, out_max, i_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_limit = i_limit
        self.integ = 0.0
        self.prev = None
        self.out = 0.0

    def reset(self):
        self.integ = 0.0
        self.prev = None
        self.out = 0.0

    def update(self, err, dt):
        if dt <= 0.0:
            return self.out
        self.integ += self.ki * err * dt
        if self.integ > self.i_limit:
            self.integ = self.i_limit
        elif self.integ < -self.i_limit:
            self.integ = -self.i_limit

        if self.prev is None:
            deriv = 0.0
        else:
            deriv = (err - self.prev) / dt
        self.prev = err

        out = self.kp * err + self.integ + self.kd * deriv
        if out > self.out_max:
            out = self.out_max
        elif out < self.out_min:
            out = self.out_min
        self.out = out
        return out


pid_x = PID(PID_X_KP, PID_X_KI, PID_X_KD, -W_MAX, W_MAX, PID_X_I_LIMIT)
pid_v = PID(PID_V_KP, PID_V_I, 0.0, 0.0, V_MAX, PID_V_I_LIMIT)


# ============================================================
# 7. 串口帧
# ============================================================
def send_vel(v, w, state):
    """0x2C 0x13 v w state chk 0x5B"""
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
    chk = (0x2C ^ 0x13 ^ vb ^ wb ^ (state & 0xFF)) & 0xFF
    uart.write(bytearray([0x2C, 0x13, vb, wb, state & 0xFF, chk, 0x5B]))


# ============================================================
# 8. 状态机
# ============================================================
state = ST_IDLE
state_ms = 0
last_ctrl_ms = 0


def enter(st):
    global state, state_ms
    state = st
    state_ms = time.ticks_ms()
    pid_x.reset()
    pid_v.reset()
    print("->", ST_NAME[st])


def control(obj, area, now):
    """返回 (v, w, state)"""
    global state, state_ms, last_ctrl_ms

    if last_ctrl_ms == 0:
        last_ctrl_ms = now
    dt = time.ticks_diff(now, last_ctrl_ms) / 1000.0
    last_ctrl_ms = now
    if dt < 0.02:
        dt = 0.02
    elif dt > 0.20:
        dt = 0.20

    elapsed = time.ticks_diff(now, state_ms)

    # ---- 目标色块偏置：让物体往色块那一侧偏 ----
    bias = 0.0
    if obj is not None and area is not None:
        bias = PUSH_BIAS_K * (area[0] - obj[0])
        if bias > PUSH_BIAS_MAX:
            bias = PUSH_BIAS_MAX
        elif bias < -PUSH_BIAS_MAX:
            bias = -PUSH_BIAS_MAX
    cx_target = IMG_CX + bias

    # ★ err 的正负必须和 w 的符号一致：
    #   物体偏左（cx < 目标）-> err > 0 -> w > 0 -> 左转 -> 物体回到画面中间
    if obj is None:
        err = 0.0
        area_push = AREA_PUSH
    else:
        err = cx_target - obj[0]
        area_push = AREA_PUSH
        if obj[5] == 1:
            area_push = AREA_PUSH_KIND[1]
        elif obj[5] == 2:
            area_push = AREA_PUSH_KIND[2]
        elif obj[5] == 3:
            area_push = AREA_PUSH_KIND[3]

    # ---------------- 状态迁移 ----------------
    if state == ST_IDLE:
        enter(ST_SEARCH)
    elif state == ST_SEARCH:
        if obj is not None:
            enter(ST_APPROACH)
    elif state == ST_APPROACH:
        if obj is None:
            if elapsed > 500:
                enter(ST_SEARCH)
        elif obj[4] >= area_push:
            enter(ST_PUSH)
        elif elapsed > 12000:
            enter(ST_PUSH)
    elif state == ST_PUSH:
        if elapsed > PUSH_MS:
            enter(ST_DONE)
        elif obj is None and area is not None and elapsed > 800:
            enter(ST_DONE)          # 物体被推入色块、和色块“合并”后消失
        elif elapsed > PUSH_TIMEOUT_MS:
            enter(ST_DONE)
    elif state == ST_DONE:
        if elapsed > DONE_MS:
            enter(ST_BACK)
    elif state == ST_BACK:
        if elapsed > BACK_MS:
            enter(ST_SEARCH)

    # ---------------- 输出 v / w ----------------
    v = 0.0
    w = 0.0

    if state == ST_SEARCH:
        w = W_SEARCH

    elif state == ST_APPROACH:
        if obj is None:
            w = W_SEARCH
            v = 0.0
        else:
            w = pid_x.update(err, dt)
            if abs(err) > ALIGN_ERR_PX:
                v = 0.0                      # 先转正再走
            else:
                v = pid_v.update(area_push - obj[4], dt)
                if 0.0 < v < V_CREEP:
                    v = V_CREEP
                if v > V_MAX:
                    v = V_MAX

    elif state == ST_PUSH:
        if obj is not None:
            w = pid_x.update(err, dt)
            v = V_PUSH
            if abs(err) > 45:
                v = V_PUSH * 0.6
        else:
            v = V_PUSH
            w = 0.0

    elif state == ST_BACK:
        v = -V_BACK
        w = 0.0

    # 限幅
    if v > 100.0:
        v = 100.0
    elif v < -100.0:
        v = -100.0
    if w > W_MAX:
        w = W_MAX
    elif w < -W_MAX:
        w = -W_MAX

    return v, w, state


# ============================================================
# 9. 主循环
# ============================================================
print("vision PID ready | QQVGA %dx%d | target=%d" % (IMG_W, IMG_H, TARGET_TYPE))

last_send = 0
last_dbg = 0

# while True:
#     send_vel(70, 0, 1)   # 持续前进
#     time.sleep_ms(50)

while True:
    clock.tick()
    img = sensor.snapshot()
    now = time.ticks_ms()

    obj = detect_target(img)
    area = find_dest_area(img)

    v, w, st = control(obj, area, now)

    if time.ticks_diff(now, last_send) >= SEND_EVERY_MS:
        send_vel(v, w, st)
        last_send = now

    # ---- 画面标注 ----
    if area is not None:
        img.draw_cross((area[0], 60), color=(0, 255, 0))

    if obj is not None:
        img.draw_cross((obj[0], obj[1]), color=(255, 0, 0))
        img.draw_rectangle((obj[0] - obj[2] // 2, obj[1] - obj[3] // 2,
                            obj[2], obj[3]), color=(255, 255, 0))
        if state == ST_APPROACH:
            draw_text(img, 2, 2, "APP %d" % obj[4], color=(255, 255, 0))
        elif state == ST_PUSH:
            draw_text(img, 2, 2, "PUSH", color=(255, 64, 64))
    else:
        draw_text(img, 2, 2, ST_NAME[state], color=(0, 255, 255))

    # ---- 调试打印 ----
    if time.ticks_diff(now, last_dbg) > 300:
        last_dbg = now
        print(v, w, st)
        if obj is not None:
            print("%-9s cx=%3d area=%4d v=%3d w=%3d area_blob=%s" %
                  (ST_NAME[state], obj[0], obj[4], int(v), int(w),
                   "Y" if area is not None else "N"))
        else:
            print("%-9s no target" % ST_NAME[state])

    time.sleep_ms(5)
