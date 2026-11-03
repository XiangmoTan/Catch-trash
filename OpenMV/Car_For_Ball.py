# -*- coding: utf-8 -*-
# ============================================================
# 光电智能垃圾分拣车 —— OpenMV H7 Plus 视觉端
#
# 任务：
#   1) 在场地里搜索「红色小球」和「绿色盒子」
#   2) 对准并靠近目标
#   3) 把目标一路推到场地北侧「红色线框堆放区」里对应颜色的格子
#   4) 退回来，继续找下一个
#
# 本文件只做“看 + 决策”，电机由 STM32 执行。
#
# 串口帧（UART3 -> STM32 USART1，115200 8N1）：
#   0x2C 0x13 v w state chk 0x5B
#     v     : -100..100  线速度，正 = 前进
#     w     : -100..100  角速度，正 = 左转
#     state : 0 IDLE / 1 SEARCH / 2 APPROACH / 3 PUSH
#             4 DONE / 5 BACK / 6 STOP
#     chk   : 0x2C ^ 0x13 ^ v ^ w ^ state
#
# STM32 端行为（main.c / vision_ctrl.c）：
#   - 只有 temp == 2 才执行视觉控制
#   - 数码管闪 = 收到了校验正确的帧（通信正常）
#   - 400ms 收不到新帧会自动停车
# ============================================================

import sensor, time
from pyb import UART, LED


# ============================================================
# 1. 摄像头初始化
# ============================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)       # 160x120：帧率高，颜色也够用
sensor.skip_frames(30)                   # 等摄像头稳定

# 关掉自动白平衡/自动增益，否则颜色阈值会随环境变化
sensor.set_auto_whitebal(False)
try:
    sensor.set_auto_gain(False)
except Exception:
    pass

clock = time.clock()

IMG_W, IMG_H = 160, 120
IMG_CX = IMG_W // 2                      # 画面中心 x = 80

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

LED(1).on()
LED(2).on()
LED(3).on()


# ============================================================
# 2. 场地/垃圾定义
# ============================================================
KIND_BALL = 1                            # 红色小球
KIND_BOX = 2                             # 绿色盒子

# 堆放区（红色线框）里从左到右 4 个格子的顺序，见场地示意图图例：
#   0 = 可回收垃圾(蓝)  1 = 厨余垃圾(绿)
#   2 = 有害垃圾(红)    3 = 其他垃圾(灰)
#
# 颜色对应关系（如果裁判规则不同，只改这里）：
#   红色小球 -> 有害垃圾(红格)
#   绿色盒子 -> 厨余垃圾(绿格)
BIN_COL = {
    KIND_BALL: 2,
    KIND_BOX: 1,
}
BIN_NUM = 4                              # 每段红框里有 4 个格子
BIN_INNER_MARGIN = 0.02                  # 红框边框厚度约占宽度的比例

# 调试用：0 = 红球和绿盒都找；1 = 只找红球；2 = 只找绿盒
TARGET_KIND = 0


# ============================================================
# 3. 颜色阈值（LAB）
# ------------------------------------------------------------
# 标定方法（OpenMV IDE）：
#   工具 -> 机器视觉 -> 阈值编辑器(Threshold Editor)
#   把要识别的物体放在它比赛时会出现的位置/光照下，
#   拖动 6 个滑块，让物体在二值图里完整、干净，背景不要被选进来。
#   把得到的 (L Min, L Max, A Min, A Max, B Min, B Max) 抄到下面。
#
# 注意：
#   - 红色小球：红色在 LAB 里是 A 很正（红-绿轴）。
#   - 绿色盒子：绿色是 A 很负。场地底板也是绿色！如果两者颜色太接近，
#     一定要把 L(亮度) 范围收紧，只保留盒子的亮度；实在分不开时，
#     建议给盒子贴一圈黑色/白色胶带，或换成与底板差异更大的绿色。
#   - 堆放区红框：大块红色，用来做“投放口”的定位标志。
# ============================================================
BALL_RED   = [(11, 34, 16, 127, -26, 35)]    # 红色小球
BOX_GREEN  = [(7, 44, -37, -10, -5, 30)]    # 绿色盒子（场地底板也绿！）
DUMP_RED   = [(25, 100, 20, 95, -10, 75)]    # 投放区红色线框

# 调试时可以把阈值打印出来看，确认不是空列表
print("BALL_RED =", BALL_RED)
print("BOX_GREEN =", BOX_GREEN)


# ============================================================
# 4. 形状/尺寸过滤参数
# ============================================================
# 红球：近似圆形，宽高比接近 1
BALL_MIN_PX    = 40       # 最小像素数。设小一点才能看到远处的球（100 太严）
BALL_MIN_AR    = 0.65     # 最小宽高比
BALL_MAX_AR    = 1.55     # 最大宽高比
BALL_MIN_ROUND = 0.50     # 圆度下限（0~1，越圆越大；拿不到就跳过）
# 红色投放框在画面上方，红球在地面。这个值原本用来排除画面上方的红框，
# 但远处的球也会出现在画面上方，会把远距离目标全部误杀。
# 红框本身是宽扁的，已经由宽高比/圆度过滤掉了，所以这里默认关闭（0.0）。
# 如果确实把上方红框误认成球，再改成 0.20。
BALL_TOP_GUARD = 0.0

# 绿盒：长方形
BOX_MIN_PX     = 120
BOX_MIN_AR     = 0.5
BOX_MAX_AR     = 2.0
BOX_MIN_SOLID  = 0.60     # 实心度下限（盒子比较实心）

# 红框：宽扁的大块红色
DUMP_MIN_PX    = 80      # 红框是空心的，pixels 是边框像素数，别设太大
DUMP_MIN_WH    = 1.30     # 红框宽/高 >= 这个值，用来和红球区分
DUMP_ROI_TOP   = 0.75     # 只在画面上方 75% 找红框

# 目标面积范围（像素），用来过滤太远/太近/太大的干扰
# ★ 这个值必须 <= BALL_MIN_PX，否则远处的小球会被这里二次过滤掉。
#   旧值 80 会把 40~79 像素的远距离球全部丢掉。
AREA_MIN = 40
AREA_MAX = 9000

# 判定“已经靠近到可以推”的面积阈值（按类型分开，现场标定）
#   标定方法见 .md：打印 area，记录车头距离物体约 10~15cm 时的面积
AREA_PUSH = {
    KIND_BALL: 2100,
    KIND_BOX: 4800,
}
BOTTOM_MARGIN = 6         # 物体底边离画面底部小于这么多像素 -> 直接开始推


# ============================================================
# 5. 运动参数（单位：PWM 百分比）
# ------------------------------------------------------------
# 这些值必须大于“电机能转起来的最小占空比”！
# 先用 OpenMV/duty_test.py 测出最小占空比，再回来改。
# ============================================================
V_MAX       = 100
# 实测（2026-11）：前进最小约 55%，后退最小约 60%。
# 这里都留了余量；换电池/换地面后要重新测。
V_CREEP     = 70     # 直线最小速度（实测 55，留 15 的余量，别卡在阈值上）
V_PUSH      = 85     # 推物体时的速度（负载更大，要比 V_CREEP 高）
V_BACK      = 70     # 退回来的速度（实测 60，留余量）
W_SEARCH    = 75     # 搜索时的原地转速（必须 >= 实测“原地转最小占空比”）
W_MAX       = 100    # 转向输出上限。★不能小于能转起来的最小值★
                     # 旧值是 40，会把搜索和 PID 的输出全部砍到 40，
                     # 结果就是明明测出 80% 能转，程序却永远只发 40%。

# 转弯死区补偿：
#   = 0   不做补偿（默认）
#   > 0   只要需要转弯，|w| 就至少是这个值。例如实测原地转要 80% 才动，
#         就填 80，否则 PID 输出 20 时车纹丝不动。
W_MIN_TURN  = 75

# 弧线模式（原地转不动时的保底方案）：
#   = 0   搜索/对准时原地转（默认）
#   > 0   搜索和对准时保持这个前进速度，边走边转，例如 75。
#         两轮都向前、只是转速不同，阻力比原地转小得多。
#         如果 duty_test 里原地转 100% 都不动，就把这里设成 75。
# ★ 关键：内轮速度 = TURN_ARC_V - TURN_ARC_W，必须 >= 实测前进最小值（55）！
#   55-20=35 < 55 时，内轮不转但驱动板还在通电，等于拖着刹车，车会完全不动。
#   80-20=60 > 55，两个轮子都真正在转，车才会走。
TURN_ARC_V  = 80     # 弧线前进速度（内轮必须 >= 55）
TURN_ARC_W  = 20     # 弧线转向量（建议 20~25；太大会退化成原地转）

# 接近目标时的速度上限。V_CREEP 是最低速度（低于它电机可能起不来），
# 这里再压一个上限，防止远距离时 PID 直接给到 V_MAX 冲过去。
# 必须 >= 55（实测启动阈值）；想更慢就 60，但别低于 55。
V_APPROACH_MAX = 65

# ============================================================
# 5.1 走走停停（解决“车太快、来不及识别”）
# ------------------------------------------------------------
# 搜索时按「先停 STEP_PAUSE_MS → 再走 STEP_GO_MS」循环：
#   走的时候仍然是正常弧线（TURN_ARC_V/W），保证轮子高于启动阈值；
#   停的时候 v=w=0，电机断电自由滑行，车基本站住。
#
# 为什么这样有用：
#   1) 停车后图像不糊，远处的小目标才分得出来；
#   2) 停车期间能连续拍到 5~10 帧，足够完成 DETECT_CONFIRM 帧确认；
#   3) 平均速度 = TURN_ARC_V × GO/(GO+PAUSE)，但瞬时占空比不变，
#      不会掉到 55% 以下导致“通电但不转”。
#
# 调法：
#   还是冲过头 -> 减小 STEP_GO_MS（例如 200），或加大 STEP_PAUSE_MS；
#   走得太慢、半天到不了垃圾 -> 加大 STEP_GO_MS。
#   不想用这个功能就设 SEARCH_STEP = False。
# ============================================================
SEARCH_STEP    = True
STEP_GO_MS     = 300    # 每次走多久
STEP_PAUSE_MS  = 300    # 每次停多久（30fps 下 300ms ≈ 9 帧）


# ============================================================
# 6. PID 参数
# ------------------------------------------------------------
# 误差单位是“像素”，输出单位是 PWM 百分比
# ============================================================
PID_X_KP, PID_X_KI, PID_X_KD = 0.45, 0.01, 0.10   # 对准目标
PID_X_I_LIMIT = 20.0

PID_V_KP, PID_V_I, PID_V_KD = 0.02, 0.0003, 0.0  # 靠近目标（面积误差）
PID_V_I_LIMIT = 10.0

PID_B_KP, PID_B_KI, PID_B_KD = 0.10, 0.0, 0.03    # 推的时候对准投放格
PID_B_I_LIMIT = 8.0

ALIGN_ERR_PX = 12     # 偏离中心小于这个像素时才允许前进


# ============================================================
# 7. 时间参数（毫秒）
# ============================================================
CTRL_PERIOD_MS     = 20      # 控制周期（主循环里 sleep）
SEND_EVERY_MS      = 50      # 50ms 发一帧（STM32 400ms 收不到会停车）
LOST_MS            = 400     # 目标丢失多久后回到搜索
DETECT_CONFIRM     = 3       # 连续多少帧检测到才认为真的存在（防误判）
APPROACH_TIMEOUT_MS = 15000  # 接近超时：长时间接近不到就强行推
PUSH_MIN_MS        = 800     # 至少推这么久
PUSH_TIMEOUT_MS    = 8000    # 推的最长时间（防止推过头）
DONE_MS            = 500     # 推到后停顿
BACK_MS            = 1500    # 后退时间（要退回到垃圾区，现场调）
SEARCH_SWEEP_MS    = 3000    # 搜索时左右各转多久换方向
SEARCH_TIMEOUT_MS  = 90000   # 超过这么久没找到就停车（0=不超时）
FRAME_STOP_RATIO   = 0.72    # 红框底边超过画面高度的 72% -> 已到投放区


# ============================================================
# 8. 状态定义
# ============================================================
ST_IDLE, ST_SEARCH, ST_APPROACH, ST_PUSH, ST_DONE, ST_BACK, ST_STOP = range(7)
ST_NAME = ["IDLE", "SEARCH", "APPROACH", "PUSH", "DONE", "BACK", "STOP"]


# ============================================================
# 9. 通用小工具
# ============================================================
def _blob_find(img, thresholds, min_px, roi=None, merge=False):
    """兼容不同固件的 find_blobs 调用方式"""
    try:
        if roi is None:
            return img.find_blobs(thresholds, pixels_threshold=min_px,
                                  area_threshold=min_px, merge=merge)
        return img.find_blobs(thresholds, roi=roi, pixels_threshold=min_px,
                              area_threshold=min_px, merge=merge)
    except Exception:
        try:
            return img.find_blobs(thresholds, pixels_threshold=min_px,
                                  area_threshold=min_px, merge=merge)
        except Exception:
            return []


def _aspect(w, h):
    """宽高比，永远 >= 1"""
    if w < 1 or h < 1:
        return 99.0
    if w >= h:
        return float(w) / float(h)
    return float(h) / float(w)


# OpenMV 5.x 把 blob 的 rect/cx/cy/pixels 从“方法”改成了“属性”：
#   旧固件：b.rect()   b.cx()   b.pixels()
#   新固件：b.rect    b.cx    b.pixels
# 下面三个函数两种写法都兼容；不兼容的话会报
#   TypeError: 'tuple' object isn't callable
def _rect(b):
    """取 blob 的 (x, y, w, h)"""
    r = b.rect
    return r() if callable(r) else r


def _blob_int(b, name, default=0):
    """取 blob 的整数属性（cx/cy/pixels），兼容方法/属性两种写法"""
    try:
        v = getattr(b, name)
        return v() if callable(v) else v
    except Exception:
        # 个别固件没有 cx/cy，就用矩形自己算
        if name == "cx" or name == "cy":
            try:
                x, y, w, h = _rect(b)
                return (x + w // 2) if name == "cx" else (y + h // 2)
            except Exception:
                return default
        return default


def _roundness(b):
    """红球圆度；固件没有这个接口时返回 None"""
    try:
        v = b.roundness
    except AttributeError:
        return None
    try:
        return v() if callable(v) else v
    except Exception:
        return None


def _solidity(b):
    """盒子实心度；固件没有这个接口时返回 None"""
    try:
        v = b.solidity
    except AttributeError:
        return None
    try:
        return v() if callable(v) else v
    except Exception:
        return None


# ============================================================
# 10. 视觉识别
# ============================================================
def find_ball(img):
    """找红色小球。返回 (cx, cy, w, h, area, kind) 或 None"""
    if TARGET_KIND not in (0, KIND_BALL):
        return None

    best = None
    best_px = 0
    for b in _blob_find(img, BALL_RED, BALL_MIN_PX, merge=True):
        x, y, w, h = _rect(b)
        ar = _aspect(w, h)

        # 1) 形状要接近圆
        if ar < BALL_MIN_AR or ar > BALL_MAX_AR:
            continue

        px = _blob_int(b, "pixels")
        if px < BALL_MIN_PX:
            continue

        # 圆度对远处的小目标不可靠，只在够大时才用它过滤，
        # 否则远处的球会因为圆度抖动被漏检。
        if px >= 120:
            r = _roundness(b)
            if r is not None and r < BALL_MIN_ROUND:
                continue

        # 2) 排除掉画面上方的红色投放框（它是宽扁的大块）
        if y + h // 2 < IMG_H * BALL_TOP_GUARD:
            continue
        if px > best_px:
            best_px = px
            best = (_blob_int(b, "cx"), _blob_int(b, "cy"),
                    w, h, px, KIND_BALL)

    return best


def find_box(img):
    """找绿色盒子。返回 (cx, cy, w, h, area, kind) 或 None"""
    if TARGET_KIND not in (0, KIND_BOX):
        return None

    best = None
    best_px = 0
    for b in _blob_find(img, BOX_GREEN, BOX_MIN_PX, merge=False):
        x, y, w, h = _rect(b)
        ar = _aspect(w, h)

        # 1) 长方形
        if ar < BOX_MIN_AR or ar > BOX_MAX_AR:
            continue
        s = _solidity(b)
        if s is not None and s < BOX_MIN_SOLID:
            continue

        # 2) 太小的不要；特别大的多半是把绿色底板也算进来了
        px = _blob_int(b, "pixels")
        if px < BOX_MIN_PX or px > AREA_MAX:
            continue

        if px > best_px:
            best_px = px
            best = (_blob_int(b, "cx"), _blob_int(b, "cy"),
                    w, h, px, KIND_BOX)

    return best


def _inside(px, py, rect):
    """点 (px,py) 是否落在矩形 rect=(x,y,w,h) 内"""
    x, y, w, h = rect
    return x <= px <= x + w and y <= py <= y + h


def find_trash(img, frame=None):
    """同时找红球和绿盒，选画面里面积最大（最近）的那个。

    frame 是 find_dump_frame() 找到的投放区红框。
    ★ 关键：投放区里的「厨余垃圾」格子本身也是绿色的，和场上的绿色盒子同色！
      所以只要某个绿色候选落在红框内部，就认定它是格子，不是垃圾，直接排除。
      否则车会一路冲着投放区里的绿格子跑，永远找不到真正的绿盒。
    """
    cands = []
    b = find_ball(img)
    if b is not None:
        cands.append(b)
    b = find_box(img)
    if b is not None:
        cands.append(b)

    best = None
    best_px = 0
    for c in cands:
        if c[4] < AREA_MIN or c[4] > AREA_MAX:
            continue
        # 落在投放区红框内部的候选 = 投放格本身，不是待分拣垃圾
        if frame is not None and _inside(c[0], c[1], frame):
            continue
        if c[4] > best_px:
            best_px = c[4]
            best = c
    return best


def find_dump_frame(img):
    """找堆放区红色线框。返回 (x, y, w, h) 或 None"""
    roi = (0, 0, IMG_W, int(IMG_H * DUMP_ROI_TOP))
    best = None
    best_score = -1

    for b in _blob_find(img, DUMP_RED, DUMP_MIN_PX, roi=roi, merge=True):
        x, y, w, h = _rect(b)
        if w < 1 or h < 1:
            continue

        # 红框是宽扁的；红球是圆的，会被这个条件排除
        if float(w) / float(h) < DUMP_MIN_WH:
            continue

        px = _blob_int(b, "pixels")
        if px < DUMP_MIN_PX:
            continue

        # 两段红框都可能在画面里，选离画面中心最近的那一段
        score = -abs(_blob_int(b, "cx") - IMG_CX)
        if score > best_score:
            best_score = score
            best = (x, y, w, h)

    return best


def bin_center_x(frame, kind):
    """根据红框位置 + 格子序号，算出目标格子在画面里的中心 x"""
    x, y, w, h = frame
    col = BIN_COL.get(kind, 0)

    inner_x = x + w * BIN_INNER_MARGIN
    inner_w = w * (1.0 - 2.0 * BIN_INNER_MARGIN)

    return int(inner_x + inner_w * (col + 0.5) / BIN_NUM)


# ============================================================
# 11. PID
# ============================================================
class PID(object):
    def __init__(self, kp, ki, kd, out_min, out_max, i_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_limit = i_limit
        self.reset()

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
pid_v = PID(PID_V_KP, PID_V_I, PID_V_KD, 0.0, V_MAX, PID_V_I_LIMIT)
pid_b = PID(PID_B_KP, PID_B_KI, PID_B_KD, -W_MAX, W_MAX, PID_B_I_LIMIT)


# ============================================================
# 12. 串口发送
# ============================================================
def send_vel(v, w, st):
    """打包并发送一帧；数值限制在 -100..100"""
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


# ============================================================
# 13. 状态机
# ============================================================
state = ST_IDLE
state_ms = 0
last_ctrl_ms = 0
last_send_ms = 0

detect_cnt = 0          # 连续检测到目标的帧数
target_kind = KIND_BALL  # 正在推的垃圾类型（目标丢失后还要用它选格子）
search_dir = 1           # 搜索旋转方向：1=左转，-1=右转
search_dir_ms = 0

# ★ 注意：判断“目标丢了”要用“最后一次看到目标的时刻”，
#   不能用“进入 APPROACH 的时刻”。否则在 APPROACH 里待超过 400ms 后，
#   中间只要漏检一帧，就会立刻退回 SEARCH，来回抖动，永远靠近不了。
last_seen_ms = 0
last_cx = IMG_CX         # 目标最后出现的位置；丢失后朝这个方向找


def enter(st):
    """状态切换：记录时间、清 PID、切换搜索方向"""
    global state, state_ms, search_dir, search_dir_ms
    state = st
    state_ms = time.ticks_ms()
    pid_x.reset()
    pid_v.reset()
    pid_b.reset()

    if st == ST_SEARCH:
        search_dir = -search_dir       # 每次重新搜索换一个方向
        search_dir_ms = state_ms

    print("->", ST_NAME[st])


def control(obj, frame, now):
    """核心状态机：输入视觉结果，输出 (v, w, state)"""
    global state, state_ms, last_ctrl_ms
    global detect_cnt, target_kind, search_dir, search_dir_ms
    global last_seen_ms, last_cx

    # ---------- 计算 dt ----------
    if last_ctrl_ms == 0:
        last_ctrl_ms = now
    dt = time.ticks_diff(now, last_ctrl_ms) / 1000.0
    last_ctrl_ms = now
    if dt < 0.02:
        dt = 0.02
    elif dt > 0.20:
        dt = 0.20

    # 只要这一帧看到目标，就刷新“最后看到”的时刻和位置
    if obj is not None:
        last_seen_ms = now
        last_cx = obj[0]

    elapsed = time.ticks_diff(now, state_ms)

    # ================= 状态迁移 =================
    if state == ST_IDLE:
        enter(ST_SEARCH)

    elif state == ST_SEARCH:
        # 连续几帧都看到目标才切到接近，避免一闪而过的噪声
        if obj is not None:
            detect_cnt += 1
        else:
            detect_cnt = 0

        if detect_cnt >= DETECT_CONFIRM:
            target_kind = obj[5]
            detect_cnt = 0
            enter(ST_APPROACH)

        # 左右扫描，避免一直朝一个方向转、把自己转晕
        if time.ticks_diff(now, search_dir_ms) > SEARCH_SWEEP_MS:
            search_dir = -search_dir
            search_dir_ms = now

        # 超时保护：找太久就停车，省电也防止撞墙
        if SEARCH_TIMEOUT_MS > 0 and elapsed > SEARCH_TIMEOUT_MS:
            enter(ST_STOP)

    elif state == ST_APPROACH:
        if obj is None:
            # 用“最后一次看到目标”计时，而不是“进入 APPROACH”计时
            if time.ticks_diff(now, last_seen_ms) > LOST_MS:
                enter(ST_SEARCH)
                # 朝目标最后消失的方向找，不要盲翻转方向
                search_dir = 1 if last_cx < IMG_CX else -1
                search_dir_ms = now
        else:
            target_kind = obj[5]
            area_push = AREA_PUSH.get(target_kind, 1800)

            # 面积够大，或者物体已经贴到画面最底部 -> 开始推
            near_bottom = (obj[1] + obj[3] // 2) > (IMG_H - BOTTOM_MARGIN)
            if obj[4] >= area_push or near_bottom:
                enter(ST_PUSH)
            elif elapsed > APPROACH_TIMEOUT_MS:
                enter(ST_PUSH)

    elif state == ST_PUSH:
        # 红框底边已经接近画面底部：说明车头到投放区了
        if frame is not None and (frame[1] + frame[3]) > IMG_H * FRAME_STOP_RATIO:
            enter(ST_DONE)
        elif elapsed > PUSH_TIMEOUT_MS:
            enter(ST_DONE)

    elif state == ST_DONE:
        if elapsed > DONE_MS:
            enter(ST_BACK)

    elif state == ST_BACK:
        if elapsed > BACK_MS:
            enter(ST_SEARCH)

    elif state == ST_STOP:
        pass

    # ================= 输出 v / w =================
    v = 0.0
    w = 0.0

    if state == ST_SEARCH:
        # 搜索：TURN_ARC_V=0 时原地转；>0 时边走边转（弧线）
        if TURN_ARC_V > 0.0:
            v = TURN_ARC_V
            w = TURN_ARC_W * search_dir
        else:
            v = 0.0
            w = W_SEARCH * search_dir

        # 走走停停：每个周期先“停下看”，再“走一段”。
        # 停的时候发 0，STM32 把四路 PWM 全关，轮子自由滑行，
        # 车很快被地面摩擦刹住，这一段时间足够拍清楚并确认目标。
        if SEARCH_STEP:
            phase = time.ticks_diff(now, state_ms) % (STEP_GO_MS + STEP_PAUSE_MS)
            if phase < STEP_PAUSE_MS:
                v = 0.0
                w = 0.0

    elif state == ST_APPROACH:
        if obj is None:
            # 短暂丢失（还没超过 LOST_MS）：朝最后看到的方向继续找，
            # 不要急停。急停会让车停在原地，目标永远回不到画面里。
            if TURN_ARC_V > 0.0:
                v = TURN_ARC_V
                w = TURN_ARC_W if last_cx < IMG_CX else -TURN_ARC_W
            else:
                v = 0.0
                w = W_SEARCH if last_cx < IMG_CX else -W_SEARCH
        else:
            # 目标偏左(cx小) -> err>0 -> w>0 左转 -> 目标回到画面中间
            err = IMG_CX - obj[0]

            if abs(err) > ALIGN_ERR_PX:
                # 还没对准：原地转，或者弧线转（TURN_ARC_V>0）
                if TURN_ARC_V > 0.0:
                    v = TURN_ARC_V
                    w = TURN_ARC_W if err > 0 else -TURN_ARC_W
                else:
                    v = 0.0
                    w = pid_x.update(err, dt)
                    # PID 输出太小、低于电机启动死区时抬到 W_MIN_TURN
                    if W_MIN_TURN > 0.0 and abs(w) < W_MIN_TURN:
                        w = W_MIN_TURN if err > 0 else -W_MIN_TURN
            else:
                # 已经对准：直行靠近，同时做小幅修正
                w = pid_x.update(err, dt)
                area_push = AREA_PUSH.get(target_kind, 1800)
                v = pid_v.update(area_push - obj[4], dt)
                if 0.0 < v < V_CREEP:
                    v = V_CREEP
                # 再压一个上限：远距离时 PID 可能直接给到 100，
                # 车会“冲”过目标，来不及做下一次判断
                if v > V_APPROACH_MAX:
                    v = V_APPROACH_MAX

    elif state == ST_PUSH:
        v = V_PUSH

        # 推的过程中，用红框里的目标格子做“车道保持”：
        # 让车头始终指向对应颜色的格子，把垃圾送进去。
        if frame is not None:
            bx = bin_center_x(frame, target_kind)
            err_b = IMG_CX - bx
            w = pid_b.update(err_b, dt)
        else:
            w = 0.0

    elif state == ST_BACK:
        v = -V_BACK
        w = 0.0

    # ---------- 限幅 ----------
    if v > V_MAX:
        v = V_MAX
    elif v < -V_MAX:
        v = -V_MAX
    if w > W_MAX:
        w = W_MAX
    elif w < -W_MAX:
        w = -W_MAX

    return v, w, state


# ============================================================
# 14. 主循环
# ============================================================
print("vision ready | QQVGA %dx%d | target=%d" % (IMG_W, IMG_H, TARGET_KIND))

last_dbg_ms = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    now = time.ticks_ms()

    # 先找红框，再找垃圾：找垃圾时要用红框排除「绿色格子」
    frame = find_dump_frame(img)
    obj = find_trash(img, frame)

    v, w, st = control(obj, frame, now)

    # ---- 按固定周期发帧 ----
    if time.ticks_diff(now, last_send_ms) >= SEND_EVERY_MS:
        send_vel(v, w, st)
        last_send_ms = now

    # ---- 画面标注（调试用，不影响控制） ----
    if frame is not None:
        img.draw_rectangle(frame, color=(255, 0, 255))
        if state in (ST_PUSH, ST_DONE):
            bx = bin_center_x(frame, target_kind)
            img.draw_line((bx, 0, bx, IMG_H - 1), color=(255, 255, 0))

    if obj is not None:
        x = obj[0] - obj[2] // 2
        y = obj[1] - obj[3] // 2
        img.draw_cross((obj[0], obj[1]), color=(255, 0, 0))
        img.draw_rectangle((x, y, obj[2], obj[3]), color=(255, 255, 0))

    # 不同固件的 draw_string 参数形式不同：
    #   OpenMV 5.x（你的板子）：draw_string((x, y), text, ...)
    #   旧固件：                draw_string(x, y, text, ...)
    # 这里两种都兼容，避免换固件就报
    # “TypeError: object 'int' isn't a tuple or list”
    try:
        img.draw_string((2, 2), ST_NAME[state], color=(0, 255, 255))
    except TypeError:
        img.draw_string(2, 2, ST_NAME[state], color=(0, 255, 255))

    # ---- 每 300ms 打印一次，调参时看 OpenMV IDE 的串口终端 ----
    if time.ticks_diff(now, last_dbg_ms) > 300:
        last_dbg_ms = now
        if obj is not None:
            print("%-8s kind=%d cx=%3d area=%4d v=%3d w=%3d frame=%s" %
                  (ST_NAME[state], obj[5], obj[0], obj[4],
                   int(v), int(w), "Y" if frame is not None else "N"))
        else:
            print("%-8s no target  v=%3d w=%3d frame=%s" %
                  (ST_NAME[state], int(v), int(w),
                   "Y" if frame is not None else "N"))

    time.sleep_ms(CTRL_PERIOD_MS)
