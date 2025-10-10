# ******************************************************************************
#  YOLOv5 🚀 by Ultralytics, GPL-3.0 license#  相机坐标系X估计（圆柱中心），支持Open3D可视化与ΔX基线
#  @file 已修复：程序不能持续运行、点云截取、刘老师IP确认192.168.1.210 等
#  @data 2025-10-09
#  @author 2307014121 杨超
#  @version 2.4
#  @attention 更改算法
# ******************************************************************************

# @problem 我们拟合圆心在榴弹炮外侧 导致坐标上差了两个半径的距离

# @brief 回顾拟合圆心函数的原理：
# 1.从检测框 ROI 内提取点云 → xyz_map[mask]
# 2.对点云做统计滤波/体素下采样去噪
# 3.用 PCA 估计圆柱轴方向
# 4.沿轴切片 → 得到一个垂直于轴的截面点集
# 5.在这个 2D 截面上拟合圆心（使用 fit_circle_center_known_radius 或均值）
# 6.将 2D 圆心映射回 3D 空间 → 得到最终形心

# @solve 问题就出在第 5 步 —— “圆拟合”阶段。
# YOLO 检测框可能只覆盖了圆柱的一半（比如右侧）
# ROI mask 只保留了右侧的点
# 拟合圆时，算法看到的是一个“半圆弧”或“月牙形”点云
# 如果你使用的是 最小二乘圆拟合（least_squares），它会试图让所有点到圆心的距离 ≈ 半径
# 但由于左侧无点，算法会把圆心往右推，使得右侧点“看起来更符合半径”
# 结果：圆心被推向点云密集区 → 偏离真实中心

# 方案 1：强制使用“对称性先验”
# 既然知道目标是对称圆柱体，那就不要依赖点云分布，而是利用对称性约束！

# 方案 2：新增函数替换点云拟合函数
# fit_circle_center_known_radius 算法被告知：“这里有一堆点，请你找一个圆心，画一个半径为6cm的圆，让这个圆尽可能地穿过这些点。”
# 当面对一段圆弧时，算法为了让一个完整的圆去贴合这段不完整的弧线，它唯一的选择就是把圆心向弧线的反方向（也就是远离相机、远离物体表面的方向）移动大约一个半径的距离。

# erode_px=2 ROI侵蚀

import argparse

import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import sys
import time
from pathlib import Path
import numpy as np
import cv2
import torch
import torch.backends.cudnn as cudnn
import csv
from collections import deque
from queue import Queue
import socket
import threading
import yaml


# ===================== 修复导入路径：支持在任意目录运行 ======================
FILE = Path(__file__).resolve()
ROOT = None
for p in [FILE.parent] + list(FILE.parents):
    if (p / 'models').exists() and (p / 'utils').exists():
        ROOT = p
        break
if ROOT is None:
    yv = os.environ.get('YOLOV5_PATH', '')
    if yv and (Path(yv) / 'models').exists():
        ROOT = Path(yv)
if ROOT is None:
    ROOT = FILE.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
# ============================================================================

# 尝试导入 Orbbec SDK
try:
    import pyorbbecsdk as ob
    HAS_ORBBEC = True
except Exception:
    ob = None
    HAS_ORBBEC = False
    print('[警告] 未检测到 pyorbbecsdk，Orbbec 深度功能将被禁用。')

# 可选依赖：Open3D、SciPy
try:
    import open3d as o3d
    HAS_O3D = True
except Exception:
    HAS_O3D = False

try:
    from scipy.optimize import least_squares
    HAS_SCIPY = True
except Exception:
    HAS_SCIPY = False

# YOLOv5 相关导入
from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression_obb, print_args, scale_polys,
                           strip_optimizer)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, time_sync
from utils.rboxs_utils import rbox2poly

try:
    from utils.augmentations import letterbox
except Exception:
    from utils.general import letterbox
    LOGGER.warning("letterbox function imported from utils.general.")

# ===== Orbbec / 相机配置 =====
DEFAULT_DEPTH_SCALE = 0.001  # 默认深度单位 mm->m（某些机型会不同，尽量从 SDK 读取）
DEPTH_WIDTH, DEPTH_HEIGHT = 640, 480
DEPTH_FPS = 30

FALLBACK_CAMERA_MATRIX = np.array([
    [611.572, 0.0, 643.59109755],
    [0.0, 611.817, 341.29027134],
    [0.0, 0.0, 1.0]
], dtype=np.float32)
FALLBACK_DIST_COEFFS = np.array([-0.12393594, 0.0467302, 0.00263421, -0.00696342, -0.01053551], dtype=np.float32)

# 默认世界变换（相机->世界），平移单位米
TRANSFORM_MATRIX = np.array([
    [0.0, 1.0, 0.0, -100],
    [1.0, 0.0, 0.0, -31.55],
    [0.0, 0.0, 1.0, 1030],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float32)
TRANSFORM_MATRIX[:3, 3] /= 1000.0  # mm -> m


# ===== 区域划分（按已交换后的世界坐标：X=wx_mm, Y=wy_mm, Z=wz_mm）=====
ID_OFFSET = 1               # ID 输出从 1 开始；若想从 0 开始，改为 0
AREA_UNKNOWN_VALUE = 0      # 不在可生成点云区域时的 AREA 值

# 正面相机（Cam 0）：梯形视野（单位 mm）
FRONT_Z_MIN_MM = 1680.0
FRONT_Z_MAX_MM = 2280.0
FRONT_Y_TOP_ABS_MM = 500.0    # 靠近相机（上底）的 |Y| 上
FRONT_Y_BOTTOM_ABS_MM = 700.0 # 远离相机（下底）的 |Y| 上限
FRONT_LEFT_AREA_ID = 1        # Y>0 -> 正面1区
FRONT_RIGHT_AREA_ID = 2       # Y<=0 -> 正面2区

# 侧面相机（Cam 1）：矩形视野（单位 mm）
SIDE_Y_MIN_MM = -2325.0
SIDE_Y_MAX_MM = -1325.0
SIDE_Z_ABS_MAX_MM = 830.0
# 450 / 500 / 450 的左右宽度 -> Z 分界在 ±250 mm
SIDE_Z_SPLIT_CENTER_MM = 250.0
SIDE_LEFT_AREA_ID = 3          # Z in [+250, +830]
SIDE_MID_AREA_ID  = 4          # Z in [-250, +250]
SIDE_RIGHT_AREA_ID= 5          # Z in [-830, -250]


# 侧面相机（Cam 2）：矩形视野（单位 mm）
SIDE_R_Y_MAX_MM = 2325.0
SIDE_R_Y_MIN_MM = 1325.0
SIDE_R_Z_ABS_MAX_MM = 830.0
# 450 / 500 / 450 的左右宽度 -> Z 分界在 ±250 mm
SIDE_R_Z_SPLIT_CENTER_MM = 250.0
SIDE_R_LEFT_AREA_ID = 6          # Z in [+250, +830]
SIDE_R_MID_AREA_ID  = 7          # Z in [-250, +250]
SIDE_R_RIGHT_AREA_ID= 8          # Z in [-830, -250]

# ========区域Area 辅助划分函数======================
def _classify_front_area_swapped(wy_mm, wz_mm):
    """正面相机：按已交换后的世界坐标 X/Z（mm）判定 1/2 区"""
    if wz_mm < FRONT_Z_MIN_MM or wz_mm > FRONT_Z_MAX_MM:
        return AREA_UNKNOWN_VALUE
    denom = max(FRONT_Z_MAX_MM - FRONT_Z_MIN_MM, 1e-6)
    t = (wz_mm - FRONT_Z_MIN_MM) / denom
    y_limit = FRONT_Y_TOP_ABS_MM + t * (FRONT_Y_BOTTOM_ABS_MM - FRONT_Y_TOP_ABS_MM)  # 线性插值
    if abs(wy_mm) > y_limit:
        return AREA_UNKNOWN_VALUE
    return FRONT_LEFT_AREA_ID if wy_mm > 0 else FRONT_RIGHT_AREA_ID

def _classify_side_area_swapped(wy_mm, wz_mm):
    """侧面相机：按已交换后的世界坐标 Y/Z（mm）判定 3/4/5 区"""
    if wy_mm < SIDE_Y_MIN_MM or wy_mm > SIDE_Y_MAX_MM:
        return AREA_UNKNOWN_VALUE
    if abs(wz_mm) > SIDE_Z_ABS_MAX_MM:
        return AREA_UNKNOWN_VALUE
    if wz_mm >= SIDE_Z_SPLIT_CENTER_MM:
        return SIDE_LEFT_AREA_ID
    if wz_mm >= -SIDE_Z_SPLIT_CENTER_MM:
        return SIDE_MID_AREA_ID
    return SIDE_RIGHT_AREA_ID

def _classify_side_right_area_swapped(wy_mm, wz_mm):
    """侧面相机（右）：按已交换后的世界坐标 Y/Z（mm）判定 6/7/8 区"""
    y_min = min(SIDE_R_Y_MIN_MM, SIDE_R_Y_MAX_MM)
    y_max = max(SIDE_R_Y_MIN_MM, SIDE_R_Y_MAX_MM)
    if wy_mm < y_min or wy_mm > y_max:
        return AREA_UNKNOWN_VALUE
    if abs(wz_mm) > SIDE_R_Z_ABS_MAX_MM:
        return AREA_UNKNOWN_VALUE
    if wz_mm >= SIDE_R_Z_SPLIT_CENTER_MM:
        return SIDE_R_LEFT_AREA_ID      # 6
    if wz_mm >= -SIDE_R_Z_SPLIT_CENTER_MM:
        return SIDE_R_MID_AREA_ID       # 7
    return SIDE_R_RIGHT_AREA_ID         # 8


# def classify_area_for_cam(wx_mm, wy_mm, wz_mm, cam_idx):
#     """
#     cam_idx == 2 -> 正面(1/2) 2
#     cam_idx == 1 -> 左侧(3/4/5) 0
#     cam_idx == 0 -> 右侧(6/7/8) 1
#     如果你的设备顺序不同，在这里改映射即可。
#     """
#     if cam_idx == 2:
#         return _classify_front_area_swapped(wy_mm, wz_mm)
#     elif cam_idx == 1:
#         return _classify_side_area_swapped(wy_mm, wz_mm)
#     elif cam_idx == 0:
#         return _classify_side_right_area_swapped(wy_mm, wz_mm)
#     else:
#         # 兜底：按左侧规则
#         return _classify_side_area_swapped(wy_mm, wz_mm)

def classify_area_for_cam(wx_mm, wy_mm, wz_mm, serial):
    """
    根据相机序列号 serial 判定使用哪个区域划分函数。
    serial: str, 相机序列号（如 'CP3C641000GY'）
    """
    classifier_func = SERIAL_TO_AREA_CLASSIFIER.get(serial)
    if classifier_func is not None:
        return classifier_func(wy_mm, wz_mm)  # 注意参数顺序与原函数一致
    else:
        LOGGER.warning(f"[区域分类] 未找到序列号 {serial} 的区域分类器，使用默认左侧规则")
        return _classify_side_area_swapped(wy_mm, wz_mm)



# def target_areas_for_cam(cam_idx):
#     # 与 classify_area_for_cam 的映射保持一致
#     if cam_idx == 2:
#         return {FRONT_LEFT_AREA_ID, FRONT_RIGHT_AREA_ID}  # {1,2}
#     elif cam_idx == 1:
#         return {SIDE_LEFT_AREA_ID, SIDE_MID_AREA_ID, SIDE_RIGHT_AREA_ID}  # {3,4,5}
#     elif cam_idx == 0:
#         return {SIDE_R_LEFT_AREA_ID, SIDE_R_MID_AREA_ID, SIDE_R_RIGHT_AREA_ID}  # {6,7,8}
#     # 兜底
#     return {SIDE_LEFT_AREA_ID, SIDE_MID_AREA_ID, SIDE_RIGHT_AREA_ID}

def target_areas_for_cam(serial):
    """
    根据相机序列号返回该相机应覆盖的目标区域集合。
    serial: str, 相机序列号（如 'CP3C641000GY'）
    """
    areas = SERIAL_TO_TARGET_AREAS.get(serial)
    if areas is not None:
        return areas
    else:
        LOGGER.warning(f"[目标区域] 未找到序列号 {serial} 的目标区域配置，使用默认左侧区域")
        return {SIDE_LEFT_AREA_ID, SIDE_MID_AREA_ID, SIDE_RIGHT_AREA_ID}  # 兜底

# ===== Serial -> Area Classifier Mapping =====
SERIAL_TO_AREA_CLASSIFIER = {
    "CP3C641000GL": _classify_front_area_swapped,  # 正面相机
    "CP3F54200057": _classify_side_area_swapped,  # 左侧相机
    "CP3C641000GY": _classify_side_right_area_swapped  # 右侧相机
}

# ===== Serial -> Target Areas Mapping =====
SERIAL_TO_TARGET_AREAS = {
    "CP3C641000GL": {FRONT_LEFT_AREA_ID, FRONT_RIGHT_AREA_ID},  # 正面相机 → 区域 1,2
    "CP3F54200057": {SIDE_LEFT_AREA_ID, SIDE_MID_AREA_ID, SIDE_RIGHT_AREA_ID},  # 左侧相机 → 区域 3,4,5
    "CP3C641000GY": {SIDE_R_LEFT_AREA_ID, SIDE_R_MID_AREA_ID, SIDE_R_RIGHT_AREA_ID}  # 右侧相机 → 区域 6,7,8
}


def format_vision_line(wx_mm, wy_mm, wz_mm, cls_name_string, area_id, out_id):
    # 保持你现有格式里的空格与标点
    return f"vision_data:X={wx_mm:.1f} , Y={wy_mm:.1f},Z={wz_mm:.1f},CLASS={cls_name_string}, AREA={area_id} ,ID={out_id},END"

# ---------- UDP 发送辅助 ----------
def udp_send_text(line, ip, port, timeout=2.0, wait_ack=True):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 如果是广播地址，打开广播开关，并默认不等待 ACK（也可以保留等待，单接收端场景没问题）
    if ip == '255.255.255.255' or ip.endswith('.255'):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # 可选：广播场景建议不等 ACK，避免防火墙或多接收端带来的不确定性
        # wait_ack = False
    sock.settimeout(timeout)
    try:
        sock.sendto(line.encode('utf-8'), (ip, int(port)))
        if wait_ack:
            try:
                data, _ = sock.recvfrom(1024)
                if data != b"ACK_VISION":
                    LOGGER.warning(f"[UDP] 未收到 ACK_VISION 或收到其他数据: {data[:32]}")
            except socket.timeout:
                LOGGER.warning("[UDP] 等待 ACK_VISION 超时")
    finally:
        sock.close()


def udp_send_file(file_path, ip, port, chunk_size=1024, timeout=2.0, max_retries=20):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)

    file_name = os.path.basename(file_path)
    addr = (ip, port)

    # 1) 发送文件名并等待 ACK
    retries = 0
    while True:
        sock.sendto(f"FILENAME:{file_name}".encode('utf-8'), addr)
        try:
            data, _ = sock.recvfrom(1024)
            if data.decode('utf-8', errors='ignore') == "ACK_FILENAME":
                break
        except socket.timeout:
            retries += 1
            if retries > max_retries:
                sock.close()
                raise TimeoutError("META ACK timeout")

    # 2) 分片发送 + 停等重传
    seq = 0
    with open(file_path, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            packet = f"{seq:08d}".encode('utf-8') + chunk
            retries = 0
            while True:
                sock.sendto(packet, addr)
                try:
                    data, _ = sock.recvfrom(1024)
                    if data.decode('utf-8', errors='ignore') == f"ACK_{seq}":
                        break
                except socket.timeout:
                    retries += 1
                    if retries > max_retries:
                        sock.close()
                        raise TimeoutError(f"ACK timeout at seq={seq}")
            seq += 1

    # 3) 结束标志
    retries = 0
    while True:
        sock.sendto(b"END_OF_FILE", addr)
        try:
            data, _ = sock.recvfrom(1024)
            if data.decode('utf-8', errors='ignore') == "ACK_END":
                break
        except socket.timeout:
            retries += 1
            if retries > max_retries:
                sock.close()
                raise TimeoutError("END ACK timeout")

    sock.close()

class UdpFileSender(threading.Thread):
    def __init__(self, ip, port, mode='file_ack', chunk_size=1024, timeout=2.0):
        super().__init__(daemon=True)
        self.ip, self.port = ip, int(port)
        self.mode = str(mode).lower()  # 'raw' 或 'file_ack'
        self.chunk_size = int(chunk_size)
        self.timeout = float(timeout)
        self.q = Queue(maxsize=1)  # 只保留最新

    def enqueue(self, file_path):
        while not self.q.empty():
            try:
                self.q.get_nowait()
            except Exception:
                break
        self.q.put(file_path)

    def run(self):
        while True:
            path = self.q.get()
            try:
                if self.mode == 'raw':
                    with open(path, 'rb') as f:
                        data = f.read()
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.sendto(data, (self.ip, self.port))
                    sock.close()
                else:
                    udp_send_file(path, self.ip, self.port, chunk_size=self.chunk_size, timeout=self.timeout)
            except Exception as e:
                LOGGER.warning(f"[UDP] 发送异常: {e}")

# ---------- 语音占位 ----------
def parse_cmd_line(txt):
    import re
    m = re.findall(r'([一二三四五六七八九0-9])区', txt)
    cn2num = {'一':1,'二':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9}
    def to_num(s): return int(s) if s.isdigit() else cn2num.get(s, None)
    if len(m) >= 2:
        src, dst = to_num(m[0]), to_num(m[1])
        if src and dst:
            return src, dst
    return None, None

def voice_thread(sender, save_dir):
    while True:
        try:
            line = input("输入指令（例如：把榴弹炮从一区搬到二区）：").strip()
            s, d = parse_cmd_line(line)
            if s and d:
                voice_path = os.path.join(str(save_dir), 'voice_cmd.txt')
                with open(voice_path, 'w', encoding='utf-8') as f:
                    f.write(f"voice_cmd:MOVE_FROM={s},MOVE_TO={d},END\n")
                sender.enqueue(voice_path)
                print(f"[VOICE] 已发送 MOVE {s}->{d}")
            else:
                print("未能解析出 区域号")
        except Exception as e:
            LOGGER.warning(f"voice thread error: {e}")
            time.sleep(0.5)

# ---------- 外参读取 ----------
def load_extrinsics_yaml(path='extrinsics.yaml'):
    """
    读取每台相机的外参 Tcw（camera->world），按 serial 匹配。
    支持单位字段 units: 'mm' 或 'm'，若 mm 会自动转米。
    返回 {serial: Tcw(np.float32 4x4)}
    """
    out = {}
    try:
        if not os.path.exists(path):
            LOGGER.warning(f"未找到外参配置: {path}，将使用默认 TRANSFORM_MATRIX")
            return out
        cfg = yaml.safe_load(open(path, 'r', encoding='utf-8'))
        for it in cfg.get('cameras', []):
            serial = str(it.get('serial', '')).strip()
            T = np.array(it['Tcw'], dtype=np.float32)
            units = str(it.get('units', 'm')).lower()
            if units == 'mm':
                T[:3, 3] /= 1000.0
            out[serial] = T
        LOGGER.info(f"外参载入完成：{len(out)} 台相机")
    except Exception as e:
        LOGGER.warning(f"加载外参失败: {e}，使用默认 TRANSFORM_MATRIX")
    return out

def camera_to_world_ex(camera_coords, Tcw):
    if camera_coords is None or Tcw is None:
        return None
    p = np.append(camera_coords, 1.0)
    w = Tcw @ p
    return w[:3] / (w[3] if w.shape[0] > 3 else 1.0)

# ---------- 基础工具 ----------
def frame_to_bgr_image(frame):
    """
    将 Orbbec color_frame 转为 BGR np.ndarray。
    兼容 RGB/BGR/YUYV/UYVY/NV12/I420/MJPG 等常见格式。
    """
    if frame is None:
        return None
    try:
        try:
            fmt = frame.get_format()
        except Exception:
            fmt = None

        h, w = frame.get_height(), frame.get_width()
        buf = frame.get_data()
        if buf is None:
            return None
        data = np.frombuffer(buf, dtype=np.uint8)

        # 专用路径
        try:
            if fmt == getattr(ob.OBFormat, 'RGB', None):
                if data.size != h * w * 3:
                    raise ValueError("RGB size mismatch")
                arr = data.reshape((h, w, 3))
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

            if fmt == getattr(ob.OBFormat, 'BGR', None):
                if data.size != h * w * 3:
                    raise ValueError("BGR size mismatch")
                return data.reshape((h, w, 3))

            if fmt in (getattr(ob.OBFormat, 'YUYV', None), getattr(ob.OBFormat, 'YUY2', None)):
                if data.size != h * w * 2:
                    raise ValueError("YUYV size mismatch")
                arr = data.reshape((h, w, 2))
                return cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUY2)

            if fmt == getattr(ob.OBFormat, 'UYVY', None):
                if data.size != h * w * 2:
                    raise ValueError("UYVY size mismatch")
                arr = data.reshape((h, w, 2))
                return cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_UYVY)

            if fmt == getattr(ob.OBFormat, 'NV12', None):
                if data.size != int(h * w * 3 / 2):
                    raise ValueError("NV12 size mismatch")
                y = data[:h * w].reshape((h, w))
                uv = data[h * w:].reshape((h // 2, w // 2, 2))
                yuv = np.zeros((h + h // 2, w), dtype=np.uint8)
                yuv[:h, :] = y
                yuv[h:, :] = uv.reshape(-1, w)
                return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)

            if fmt == getattr(ob.OBFormat, 'I420', None):
                if data.size != int(h * w * 3 / 2):
                    raise ValueError("I420 size mismatch")
                yuv = data.reshape((h + h // 2, w))
                return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

            if fmt in (getattr(ob.OBFormat, 'MJPG', None), getattr(ob.OBFormat, 'MJPEG', None)):
                img = cv2.imdecode(data, cv2.IMREAD_COLOR)
                if img is not None:
                    return img
                else:
                    raise ValueError("cv2.imdecode failed for MJPG")
        except Exception as e:
            LOGGER.warning(f"frame_to_bgr_image format decode warning: {e}")

        if data.size == h * w * 3:
            return data.reshape((h, w, 3))
        if data.size == h * w:
            gray = data.reshape((h, w))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        LOGGER.warning(f"Unsupported color frame: fmt={fmt}, size={data.size}, h={h}, w={w}")
        return None
    except Exception as e:
        LOGGER.warning(f"frame_to_bgr_image failed: {e}")
        return None

def camera_to_world(camera_coords):
    if camera_coords is None:
        return None
    p = np.append(camera_coords, 1.0)
    wh = TRANSFORM_MATRIX @ p
    return wh[:3] / wh[3]

def get_camera_intrinsics(pipeline):
    """返回 depth_intrinsics_obj, depth_camera_matrix, depth_dist_coeffs（深度相机）"""
    try:
        frames = pipeline.wait_for_frames(1000)
        if frames is None:
            return None, None, None
        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            return None, None, None
        depth_profile = depth_frame.get_stream_profile()
        intr = depth_profile.as_video_stream_profile().get_intrinsic()
        K = np.array([[intr.fx, 0, intr.cx],
                      [0, intr.fy, intr.cy],
                      [0, 0, 1]], dtype=np.float32)
        dist = depth_profile.as_video_stream_profile().get_distortion()
        D = np.array([dist.k1, dist.k2, dist.p1, dist.p2, dist.k3], dtype=np.float32)
        return intr, K, D
    except Exception as e:
        LOGGER.warning(f"获取相机内参失败: {e}")
        return None, None, None

def get_color_intrinsics_from_pipeline(pipeline):
    """
    从 Orbbec pipeline 的 CameraParam 里获取彩色相机内参和畸变。
    兼容不同命名(color/rgb)，获取不到就返回 None。
    """
    try:
        cam_param = pipeline.get_camera_param()
    except Exception as e:
        LOGGER.warning(f"pipeline.get_camera_param() 失败: {e}")
        return None, None, None

    # 取彩色内参（不同固件可能叫 color_intrinsic 或 rgb_intrinsic）
    intr = None
    for attr in ('color_intrinsic', 'rgb_intrinsic'):
        intr = getattr(cam_param, attr, None)
        if intr is not None:
            break
    if intr is None:
        LOGGER.warning("未找到彩色相机内参，回退使用深度内参")
        intr = getattr(cam_param, 'depth_intrinsic', None)
        if intr is None:
            return None, None, None

    fx, fy, cx, cy = intr.fx, intr.fy, intr.cx, intr.cy
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]], dtype=np.float32)

    # 取彩色畸变（不同固件可能叫 color_distortion 或 rgb_distortion）
    D = None
    dist = None
    for attr in ('color_distortion', 'rgb_distortion'):
        dist = getattr(cam_param, attr, None)
        if dist is not None:
            break
    if dist is not None:
        try:
            D = np.array([dist.k1, dist.k2, dist.p1, dist.p2, dist.k3], dtype=np.float32)
        except Exception:
            D = None
    if D is None:
        D = np.zeros(5, dtype=np.float32)

    return K, D, intr

def pixel_to_camera(pixel_coords, depth_value, camera_matrix_or_intrinsics, dist_coeffs=None, pp_delta=(0.0, 0.0)):
    if depth_value is None or depth_value <= 0:
        return None
    u = float(pixel_coords[0]) + float(pp_delta[0])
    v = float(pixel_coords[1]) + float(pp_delta[1])
    if hasattr(camera_matrix_or_intrinsics, 'fx'):
        intr = camera_matrix_or_intrinsics
        fx, fy, cx, cy = intr.fx, intr.fy, intr.cx, intr.cy
    elif isinstance(camera_matrix_or_intrinsics, np.ndarray):
        K = camera_matrix_or_intrinsics
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    else:
        return None
    X = (u - cx) * depth_value / fx
    Y = (v - cy) * depth_value / fy
    Z = depth_value
    return np.array([X, Y, Z], dtype=np.float32)

def depth_to_xyz_map(depth_m, K):
    """将对齐到彩色后的深度图（米）转为 HxWx3 的点云（米）"""
    if depth_m is None or K is None:
        return None
    H, W = depth_m.shape[:2]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    Z = depth_m
    X = (u - cx) * Z / (fx + 1e-12)
    Y = (v - cy) * Z / (fy + 1e-12)
    xyz = np.stack([X, Y, Z], axis=-1)
    invalid = (Z <= 0) | (~np.isfinite(Z))
    xyz[invalid] = np.nan
    return xyz

def ensure_xyz_in_meters(xyz_map):
    """若检测到单位像毫米（Z 中位数 > 20），则转米"""
    try:
        z_med = np.nanmedian(np.abs(xyz_map[..., 2]))
        if np.isfinite(z_med) and z_med > 20.0:
            return xyz_map / 1000.0
    except Exception:
        pass
    return xyz_map

def project_point_to_pixel(K, pt3):
    """将相机坐标系下的 3D 点投影到像素"""
    if K is None or pt3 is None:
        return None, None
    X, Y, Z = float(pt3[0]), float(pt3[1]), float(pt3[2])
    if Z <= 1e-9:
        return None, None
    u = K[0, 0] * X / Z + K[0, 2]
    v = K[1, 1] * Y / Z + K[1, 2]
    return float(u), float(v)

def draw_axes_at_point(image, point3D, camera_matrix, dist_coeffs, axis_length=0.05):
    if point3D is None or camera_matrix is None:
        return image
    axis = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]).reshape(-1, 3)
    rvec = np.zeros((3, 1), dtype=np.float32)
    tvec = np.array(point3D, dtype=np.float32).reshape(3, 1)
    try:
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
        imgpts = imgpts.astype(int)
        p0 = tuple(imgpts[0].ravel()); pX = tuple(imgpts[1].ravel()); pY = tuple(imgpts[2].ravel()); pZ = tuple(imgpts[3].ravel())
        cv2.line(image, p0, pX, (0, 0, 255), 2)
        cv2.line(image, p0, pY, (0, 255, 0), 2)
        cv2.line(image, p0, pZ, (255, 0, 0), 2)
        cv2.circle(image, p0, 3, (0, 0, 0), -1)
    except Exception as e:
        LOGGER.warning(f"draw_axes failed: {e}")
    return image

def polygon_center(poly_xyxyxyxy, mode='poly'):
    pts = np.array(poly_xyxyxyxy, dtype=np.float32)
    if pts.ndim == 1 and pts.size == 8:
        pts = pts.reshape(-1, 2)
    if mode == 'bbox':
        x1, y1 = np.min(pts[:, 0]), np.min(pts[:, 1])
        x2, y2 = np.max(pts[:, 0]), np.max(pts[:, 1])
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0
    if mode in ('poly', 'mass'):
        M = cv2.moments(pts.astype(np.float32))
        if abs(M['m00']) < 1e-6:
            return polygon_center(pts, mode='bbox')
        cx = M['m10'] / M['m00']; cy = M['m01'] / M['m00']
        return cx, cy
    return polygon_center(pts, mode='bbox')

def depth_from_roi(depth_m, poly):
    h, w = depth_m.shape[:2]
    pts = np.array(poly, dtype=np.int32).reshape(-1, 2)
    x1 = np.clip(int(np.min(pts[:, 0])), 0, w - 1)
    y1 = np.clip(int(np.min(pts[:, 1])), 0, h - 1)
    x2 = np.clip(int(np.max(pts[:, 0])), 0, w - 1)
    y2 = np.clip(int(np.max(pts[:, 1])), 0, h - 1)
    roi = depth_m[y1:y2 + 1, x1:x2 + 1]
    valid = (roi > 0) & (roi < 10.0)
    vals = roi[valid]
    if vals.size == 0:
        return None
    return float(np.median(vals))

def get_fused_u_for_symmetric_cylinder(depth_m, poly, camera_matrix=None, weights=(0.3, 0.4, 0.3)):
    h, w = depth_m.shape
    pts = np.array(poly, dtype=np.int32).reshape(-1, 2)
    x_min, x_max = np.min(pts[:, 0]), np.max(pts[:, 0])
    y_min, y_max = np.min(pts[:, 1]), np.max(pts[:, 1])
    u_bbox = (x_min + x_max) / 2.0
    v_bbox = (y_min + y_max) / 2.0
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 1)
    ys, xs = np.where(mask > 0)
    valid_zs = depth_m[ys, xs]
    valid_mask = (valid_zs > 0.1) & (valid_zs < 5.0)
    if np.any(valid_mask):
        u_cloud = np.mean(xs[valid_mask]); v_cloud = np.mean(ys[valid_mask])
    else:
        u_cloud, v_cloud = u_bbox, v_bbox
    left_mask = xs[valid_mask] <= u_bbox; right_mask = xs[valid_mask] > u_bbox
    if np.any(left_mask) and np.any(right_mask):
        u_left = np.mean(xs[valid_mask][left_mask]); u_right = np.mean(xs[valid_mask][right_mask])
        u_symm = (u_left + u_right) / 2.0
    else:
        u_symm = u_bbox
    w1, w2, w3 = weights
    u_fused = w1 * u_bbox + w2 * u_cloud + w3 * u_symm
    v_fused = w1 * v_bbox + w2 * v_cloud + w2 * v_bbox
    return u_fused, v_fused

# ================== 点云几何（基于ROI点集） ==================
def polygon_to_mask(poly, shape_hw, erode_px=0):
    H, W = shape_hw
    pts = np.array(poly, dtype=np.int32).reshape(-1, 2)
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 1)
    if erode_px and erode_px > 0:
        k = int(erode_px)
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
    return mask.astype(bool)

def pca_axis(points):
    pts = points - points.mean(axis=0, keepdims=True)
    if pts.shape[0] < 3:
        return np.array([0, 0, 1], dtype=np.float32)
    _, _, Vt = np.linalg.svd(pts, full_matrices=False)
    axis = Vt[0]
    return (axis / (np.linalg.norm(axis) + 1e-12)).astype(np.float32)

def orthonormal_basis_from_axis(axis):
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    ref = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    if abs(np.dot(axis, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    u = np.cross(axis, ref); u /= (np.linalg.norm(u) + 1e-12)
    v = np.cross(axis, u); v /= (np.linalg.norm(v) + 1e-12)
    return u.astype(np.float32), v.astype(np.float32)

def robust_slice_by_median(points, axis, init_thickness=0.02, min_points=300, max_thickness=0.08):
    pc_mean = points.mean(axis=0)
    t = (points - pc_mean) @ axis
    t_med = np.median(t)
    thickness = init_thickness
    for _ in range(4):
        sel = np.abs(t - t_med) <= (thickness * 0.5)
        if np.count_nonzero(sel) >= min_points:
            return points[sel], (pc_mean + t_med * axis), thickness
        thickness = min(max_thickness, thickness * 2.0)
    return points, (pc_mean + t_med * axis), thickness

def fit_circle_center_known_radius(uv, r, c0=None, robust=True):
    pts = np.asarray(uv, dtype=np.float32)
    if pts.shape[0] < 20:
        return np.median(pts, axis=0)
    if c0 is None:
        c0 = np.median(pts, axis=0)
    if HAS_SCIPY:
        def resid(c): return np.linalg.norm(pts - c, axis=1) - r
        res = least_squares(resid, c0, method='trf', loss=('soft_l1' if robust else 'linear'),
                            f_scale=max(0.25 * r, 0.003), max_nfev=200)
        return res.x.astype(np.float32)
    c = c0.astype(np.float32); lr = 0.1
    for _ in range(80):
        diff = c - pts; di = np.linalg.norm(diff, axis=1) + 1e-6
        grad = 2.0 * np.sum(((di - r) / di)[:, None] * diff, axis=0)
        c_new = c - lr * grad / max(pts.shape[0], 1)
        if np.linalg.norm(c_new - c) < 1e-6: c = c_new; break
        c = c_new
    return c

def fit_circle_center_unknown_radius(uv, c0=None, r0=None, robust=True):
    pts = np.asarray(uv, dtype=np.float32)
    if pts.shape[0] < 20:
        return np.median(pts, axis=0), float(np.median(np.linalg.norm(pts - np.median(pts, axis=0), axis=1)))
    if c0 is None:
        c0 = np.median(pts, axis=0)
    if r0 is None:
        r0 = float(np.median(np.linalg.norm(pts - c0, axis=1)))
    if HAS_SCIPY:
        def resid(x):
            c = x[:2]; r = x[2]
            return np.linalg.norm(pts - c, axis=1) - r
        x0 = np.array([c0[0], c0[1], r0], dtype=np.float32)
        res = least_squares(resid, x0, method='trf',
                            bounds=([-np.inf, -np.inf, 0.0], [np.inf, np.inf, np.inf]),
                            loss=('soft_l1' if robust else 'linear'),
                            f_scale=max(0.25 * r0, 0.003), max_nfev=200)
        return res.x[:2].astype(np.float32), float(res.x[2])
    return c0.astype(np.float32), float(r0)



def estimate_cylinder_center_from_points(
        pts, known_radius_m=0.06,
        voxel_size=0.005, nb_neighbors=30, std_ratio=1.5,
        slice_thickness_m=0.02, min_points=200, max_points=60000,
        force_circle_fit=True, gate_z_max_m=None, gate_y_range_m=None
):
    pts = np.asarray(pts, dtype=np.float32)
    if pts.shape[0] < 50:
        return None, None, 0
    # 几何 gate（单位：米）
    if gate_z_max_m is not None:
        pts = pts[pts[:, 2] <= float(gate_z_max_m)]
    if gate_y_range_m is not None:
        y0, y1 = float(gate_y_range_m[0]), float(gate_y_range_m[1])
        pts = pts[(pts[:, 1] >= y0) & (pts[:, 1] <= y1)]
    if pts.shape[0] < 50:
        return None, None, 0
    if pts.shape[0] > max_points:
        pts = pts[np.random.choice(pts.shape[0], max_points, replace=False)]
    # 去噪
    if HAS_O3D:
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        try:
            pcd = pcd.voxel_down_sample(voxel_size=max(1e-3, float(voxel_size)))
        except Exception:
            pass
        try:
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=int(nb_neighbors), std_ratio=float(std_ratio))
            pcd = pcd.select_by_index(ind)
        except Exception:
            pass
        pts = np.asarray(pcd.points)
        if pts.shape[0] < 50: return None, None, 0
    else:
        med = np.median(pts, axis=0); mad = np.median(np.abs(pts - med), axis=0) + 1e-6
        pts = pts[np.all(np.abs(pts - med) <= 4.0 * mad, axis=1)]
        if pts.shape[0] < 50: return None, None, 0
    # 轴估计 + 切片
    axis = pca_axis(pts)
    slice_pts, base_point, _ = robust_slice_by_median(pts, axis, init_thickness=slice_thickness_m, min_points=min_points)
    if slice_pts.shape[0] < 20:
        return pts.mean(axis=0).astype(np.float32), axis.astype(np.float32), int(pts.shape[0])
    u, v = orthonormal_basis_from_axis(axis)
    vecs = slice_pts - base_point
    uv = np.stack([vecs @ u, vecs @ v], axis=1)


    # # 截面圆拟合
    # if force_circle_fit and (known_radius_m is not None) and (known_radius_m > 0):
    #     c2d = fit_circle_center_known_radius(uv, known_radius_m, c0=np.median(uv, axis=0), robust=True)
    # else:
    #     c2d = np.mean(uv, axis=0)

    # ========== 修复：强制利用圆柱对称性 ==========
    if force_circle_fit and (known_radius_m is not None) and (known_radius_m > 0):
        # 先计算当前点云的质心作为初始猜测
        c0 = np.median(uv, axis=0)

        # 计算每个点的方向向量（从 c0 指向点）
        vecs = uv - c0  # shape: (N, 2)
        norms = np.linalg.norm(vecs, axis=1, keepdims=True) + 1e-8
        unit_vecs = vecs / norms  # 单位方向向量

        # 根据已知半径，反推“理想圆心”应在每个点的反方向 radius 处
        # 即：ideal_center = point - unit_vec * radius
        candidate_centers = uv - unit_vecs * known_radius_m

        # 最终圆心 = 所有候选圆心的中位数（鲁棒）
        c2d = np.median(candidate_centers, axis=0)
    else:
        c2d = np.mean(uv, axis=0)


    center3d = base_point + c2d[0] * u + c2d[1] * v
    return center3d.astype(np.float32), axis.astype(np.float32), int(slice_pts.shape[0])


# 新增函数解决单视角下的偏移问题。
def estimate_cylinder_center_from_points_v2(
        pts, known_radius_m=0.06,
        voxel_size=0.005, nb_neighbors=30, std_ratio=1.5,
        slice_thickness_m=0.02, min_points=200, max_points=60000,
        gate_z_max_m=None, gate_y_range_m=None
):
    """
    改动点2：
    使用几何推算方法来估计圆柱中心，以解决单视角下的偏移问题。
    """
    pts = np.asarray(pts, dtype=np.float32)
    if pts.shape[0] < 50:
        return None, None, 0
    # 几何 gate（单位：米）
    if gate_z_max_m is not None:
        pts = pts[pts[:, 2] <= float(gate_z_max_m)]
    if gate_y_range_m is not None:
        y0, y1 = float(gate_y_range_m[0]), float(gate_y_range_m[1])
        pts = pts[(pts[:, 1] >= y0) & (pts[:, 1] <= y1)]
    if pts.shape[0] < 50:
        return None, None, 0
    if pts.shape[0] > max_points:
        pts = pts[np.random.choice(pts.shape[0], max_points, replace=False)]

    # 去噪（与原版相同）
    if HAS_O3D:
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        try:
            pcd = pcd.voxel_down_sample(voxel_size=max(1e-3, float(voxel_size)))
        except Exception:
            pass
        try:
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=int(nb_neighbors), std_ratio=float(std_ratio))
            pcd = pcd.select_by_index(ind)
        except Exception:
            pass
        pts = np.asarray(pcd.points)
        if pts.shape[0] < 50: return None, None, 0
    else:
        med = np.median(pts, axis=0)
        mad = np.median(np.abs(pts - med), axis=0) + 1e-6
        pts = pts[np.all(np.abs(pts - med) <= 4.0 * mad, axis=1)]
        if pts.shape[0] < 50: return None, None, 0

    # 轴估计 + 切片（与原版相同）
    axis = pca_axis(pts)
    slice_pts, _, _ = robust_slice_by_median(pts, axis, init_thickness=slice_thickness_m, min_points=min_points)
    if slice_pts.shape[0] < 20:
        # 如果切片点太少，回退到使用整个点云的均值并进行推算
        slice_pts = pts

    # ==================== 新的核心逻辑：几何推算 ====================
    # 1. 计算截面点云在相机坐标系下的表面中心（使用均值或中位数都可以）
    surface_center = np.mean(slice_pts, axis=0)

    # 2. 计算从相机原点(0,0,0)指向表面中心的方向向量
    #    这个向量近似于相机看向物体表面的视线方向
    view_direction = surface_center / (np.linalg.norm(surface_center) + 1e-9)

    # 3. 从表面中心点，沿着视线反方向移动一个半径的距离，得到真实的中心点
    #    这是整个修复方案的关键步骤
    center3d = surface_center + view_direction * known_radius_m
    # ===============================================================

    return center3d.astype(np.float32), axis.astype(np.float32), int(slice_pts.shape[0])




# ===== Open3D 可视化：ROI->滤波->球+轴 =====
def _make_o3d_axis_lines_mm(center_m, axis, half_len_mm=50.0, color=(1.0, 0.0, 0.0)):
    c = np.asarray(center_m, dtype=np.float32)
    a = np.asarray(axis, dtype=np.float32); a /= (np.linalg.norm(a) + 1e-12)
    L = float(half_len_mm) / 1000.0
    p0 = c - a * L; p1 = c + a * L
    line = o3d.geometry.LineSet()
    line.points = o3d.utility.Vector3dVector(np.stack([p0, p1], axis=0))
    line.lines = o3d.utility.Vector2iVector([[0, 1]])
    line.colors = o3d.utility.Vector3dVector([color])
    return line

def show_o3d_debug_like_user_pc(xyz_map, poly, center3d, axis3d,
                                voxel_mm=1.0, outlier_nn=20, outlier_std=2.0,
                                sphere_mm=2.0, axis_mm=50.0):
    if not HAS_O3D:
        LOGGER.warning("pc-viz 启用但未安装 Open3D，跳过可视化。")
        return
    if xyz_map is None:
        LOGGER.warning("pc-viz: xyz_map 不可用")
        return
    H, W = xyz_map.shape[:2]


    # 改点1：
    # 靠边缘时本来就丢点，ROI(框选区域)再侵蚀更容易小弧段、退化。
    # erode_px=2 改成直接用 erode_px=0
    # 或者做个“触边检测”，触边就用 erode_px=0，否则2。
    mask = polygon_to_mask(poly, (H, W), erode_px=0)




    pts = xyz_map[mask]
    valid = np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 1e-9)
    pts = pts[valid]
    if pts.shape[0] == 0:
        LOGGER.warning("pc-viz: ROI 内无有效点")
        return
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    o3d.visualization.draw_geometries([pcd], window_name="ROI raw (m)")
    try:
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=int(outlier_nn), std_ratio=float(outlier_std))
        pcd_f = pcd.select_by_index(ind)
    except Exception:
        pcd_f = pcd
    o3d.visualization.draw_geometries([pcd_f], window_name="ROI filtered")
    try:
        pcd_vis = pcd_f.voxel_down_sample(voxel_size=max(1e-4, float(voxel_mm) / 1000.0))
    except Exception:
        pcd_vis = pcd_f
    geoms = [pcd_vis]
    if center3d is not None:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=float(sphere_mm) / 1000.0)
        sphere.translate(center3d); sphere.paint_uniform_color([1.0, 0.0, 0.0])
        geoms.append(sphere)
    if center3d is not None and axis3d is not None:
        line = _make_o3d_axis_lines_mm(center3d, axis3d, half_len_mm=axis_mm, color=(1.0, 0.0, 0.0))
        geoms.append(line)
    o3d.visualization.draw_geometries(geoms, window_name="ROI + center + axis (m)")

# ---------- Orbbec 初始化（按设备索引） ----------
def init_orbbec_pipeline(mode='SW', enable_sync=True, width=DEPTH_WIDTH, height=DEPTH_HEIGHT, fps=DEPTH_FPS, device_index=0):
    if not HAS_ORBBEC:
        raise RuntimeError("pyorbbecsdk 未安装或不可用。")

    ctx = ob.Context()
    dev_list = ctx.query_devices()
    if dev_list.get_count() <= device_index:
        raise RuntimeError(f"未找到 Orbbec 设备 index={device_index}，已连接 {dev_list.get_count()} 台")
    dev = dev_list.get_device_by_index(device_index)
    serial = dev.get_device_info().get_serial_number()

    pipeline = ob.Pipeline(dev)
    config = ob.Config()

    # 选择 profile（深度 Y16；颜色优先 RGB）
    color_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
    depth_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)

    color_profile = None
    depth_profile = None
    if depth_profiles is not None:
        try:
            depth_profile = depth_profiles.get_video_stream_profile(width, height, ob.OBFormat.Y16, fps)
        except Exception:
            depth_profile = depth_profiles.get_default_video_stream_profile()

    if color_profiles is not None:
        try:
            color_profile = color_profiles.get_video_stream_profile(width, height, ob.OBFormat.RGB, fps)
        except Exception:
            try:
                color_profile = color_profiles.get_video_stream_profile(1280, 720, ob.OBFormat.RGB, fps)
            except Exception:
                color_profile = color_profiles.get_default_video_stream_profile()

    if color_profile is None or depth_profile is None:
        LOGGER.error("未能找到 color 或 depth profile，初始化失败")
        raise RuntimeError("No color/depth profile available")

    config.enable_stream(color_profile)
    config.enable_stream(depth_profile)

    # 对齐模式
    mode_upper = (mode or 'SW').upper()
    try:
        if mode_upper == 'HW':
            config.set_align_mode(ob.OBAlignMode.HW_MODE)
            LOGGER.info(f"[Cam {device_index}] 启用硬件 D2C 对齐")
        elif mode_upper == 'SW':
            config.set_align_mode(ob.OBAlignMode.SW_MODE)
            LOGGER.info(f"[Cam {device_index}] 启用软件 D2C 对齐")
        elif mode_upper == 'NONE':
            config.set_align_mode(ob.OBAlignMode.DISABLE)
            LOGGER.info(f"[Cam {device_index}] 禁用对齐")
        else:
            config.set_align_mode(ob.OBAlignMode.DISABLE)
    except Exception as e:
        LOGGER.warning(f"[Cam {device_index}] 无法设置对齐模式: {e}")
        config.set_align_mode(ob.OBAlignMode.DISABLE)

    if enable_sync:
        try:
            pipeline.enable_frame_sync()
        except Exception as e:
            LOGGER.warning(f"[Cam {device_index}] frame sync 失败: {e}")

    pipeline.start(config)

    intrinsics_obj, camera_matrix, dist_coeffs = get_camera_intrinsics(pipeline)
    if intrinsics_obj is None or camera_matrix is None:
        LOGGER.warning(f"[Cam {device_index}] get_camera_intrinsics 获取失败，使用回退内参")
        camera_matrix = FALLBACK_CAMERA_MATRIX.copy()
        dist_coeffs = FALLBACK_DIST_COEFFS.copy()
    else:
        LOGGER.info(f"[Cam {device_index}] 成功从设备获取相机内参（深度相机）- 将在首帧切换为彩色相机内参")

    depth_scale = DEFAULT_DEPTH_SCALE
    try:
        depth_profile = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
        depth_scale = getattr(depth_profile, "get_depth_scale", lambda: DEFAULT_DEPTH_SCALE)()
        if not isinstance(depth_scale, (int, float)) or depth_scale <= 0:
            depth_scale = DEFAULT_DEPTH_SCALE
    except Exception:
        depth_scale = DEFAULT_DEPTH_SCALE

    return pipeline, serial, camera_matrix, dist_coeffs, float(depth_scale), mode_upper

# ---------- 单相机处理（顺序模式的一次处理） ----------
@torch.no_grad()
def process_one_camera(model, device, names, device_index, opt, sender, extrinsics_map, session_save_dir: Path):
    # 解包常用选项
    imgsz = check_img_size(opt.imgsz, s=model.stride)
    half = opt.half and device.type != 'cpu'
    if hasattr(model, 'pt') and (model.pt or model.jit):
        model.model.half() if half else model.model.float()

    target_name_lc = (opt.target_name or '').strip().lower()
    save_csv = opt.save_csv
    view_img = opt.view_img
    mode = opt.mode
    enable_sync = opt.enable_sync

    # 初始化相机（指定索引）
    pipeline, serial_sn, camera_matrix, dist_coeffs, depth_scale, used_mode = init_orbbec_pipeline(
        mode=mode, enable_sync=enable_sync, width=DEPTH_WIDTH, height=DEPTH_HEIGHT, fps=DEPTH_FPS, device_index=device_index
    )
    LOGGER.info(f"[Cam {device_index}] 初始化成功: mode={used_mode}, depth_scale={depth_scale}, SN={serial_sn}")

    # 选择该相机的外参
    Tcw_use = extrinsics_map.get(serial_sn, TRANSFORM_MATRIX.copy())
    LOGGER.info(f"[Cam {device_index}] 使用外参: {'YAML' if serial_sn in extrinsics_map else 'DEFAULT'}")

    # 保存目录：会话目录下的 cam{idx}
    save_dir = session_save_dir / f"cam{device_index}"
    save_dir.mkdir(parents=True, exist_ok=True)


    # 每相机：累计每个区域的“最佳一条”（按 conf 最大）
    area_best = {}  # area_id -> (conf, line)
    # areas_target = target_areas_for_cam(device_index)

    areas_target = target_areas_for_cam(serial_sn)

    # ========== 持续模式提示 ==========
    if getattr(opt, 'continuous_mode', False):
        LOGGER.info(f"[Cam {device_index}] 运行于持续模式")

    # CSV
    csv_file = None
    csv_writer = None
    if save_csv:
        csv_path = os.path.join(str(save_dir), 'results.csv')
        csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['frame', 'cam_idx', 'serial', 'class', 'conf', 'u', 'v', 'depth_m',
                             'cam_x', 'cam_y', 'cam_z',
                             'world_x_mm_swapped', 'world_y_mm_swapped', 'world_z_mm', 'area_id'])

    # 视频保存（可选）
    video_path = os.path.join(str(save_dir), 'orbbec_detected_video.mp4')
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    try:
        video_writer = cv2.VideoWriter(video_path, fourcc, DEPTH_FPS, (DEPTH_WIDTH, DEPTH_HEIGHT))
    except Exception as e:
        LOGGER.warning(f"[Cam {device_index}] 无法创建视频写入器: {e}")
        video_writer = None

    # TXT 输出（仅写文件，不在此处发送；由会话聚合后统一发送）
    world_txt_path = os.path.join(str(save_dir), 'world_coords.txt')
    last_line = None

    # 参数预处理
    dt = [0.0, 0.0, 0.0]
    x_hist = deque(maxlen=opt.pc_smooth)
    x_baseline = None
    gate_z_max_m = None if opt.pc_gate_z_max_mm is None else float(opt.pc_gate_z_max_mm) / 1000.0
    if (opt.pc_gate_y_min_mm is not None) and (opt.pc_gate_y_max_mm is not None):
        gate_y_range_m = (float(opt.pc_gate_y_min_mm) / 1000.0, float(opt.pc_gate_y_max_mm) / 1000.0)
    else:
        gate_y_range_m = None

    # 官方对齐 + 点云滤镜
    try:
        align_filter = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
    except Exception:
        align_filter = None
        LOGGER.warning(f"[Cam {device_index}] AlignFilter 创建失败，将使用原始帧集")

    try:
        pc_filter = ob.PointCloudFilter()
        try:
            camera_param = pipeline.get_camera_param()
            pc_filter.set_camera_param(camera_param)
        except Exception:
            pass
        pc_filter.set_create_point_format(ob.OBFormat.POINT)  # 只取XYZ
    except Exception:
        pc_filter = None
        LOGGER.warning(f"[Cam {device_index}] PointCloudFilter 创建失败，将无法生成整帧点云")

    intr_ready = False  # 首帧切到彩色相机内参
    last_unit_checked = False

    # 会话控制：找到一次目标世界坐标即结束；否则最多等待 max_wait_s
    start_ts = time.time()
    processed_frames_count = 0
    got_valid_world = False

    try:
        while True:
            # 超时判断
            if (time.time() - start_ts) > float(opt.max_wait_s):
                LOGGER.info(f"[Cam {device_index}] 超时 {opt.max_wait_s}s，未获取有效世界坐标，写 NONE 后结束该相机会话。")
                break

            frameset = pipeline.wait_for_frames(1000)
            if frameset is None:
                LOGGER.warning(f"[Cam {device_index}] 未能获取到 Orbbec 帧集")
                time.sleep(0.01)
                continue

            frames_aligned = align_filter.process(frameset) if align_filter is not None else frameset
            color_frame = frames_aligned.get_color_frame()
            depth_frame = frames_aligned.get_depth_frame()
            if (color_frame is None) or (depth_frame is None):
                LOGGER.warning(f"[Cam {device_index}] 未能获取到彩色或深度帧")
                continue

            # 切换到彩色相机内参
            if not intr_ready:
                Kc, Dc, _ = get_color_intrinsics_from_pipeline(pipeline)
                if Kc is not None:
                    camera_matrix = Kc
                    dist_coeffs = Dc if Dc is not None else np.zeros(5, np.float32)
                    intr_ready = True
                    LOGGER.info(f"[Cam {device_index}] 使用彩色相机内参: fx={camera_matrix[0, 0]:.2f}, fy={camera_matrix[1, 1]:.2f}, "
                                f"cx={camera_matrix[0, 2]:.2f}, cy={camera_matrix[1, 2]:.2f}")
                else:
                    LOGGER.warning(f"[Cam {device_index}] 彩色相机内参获取失败，继续使用回退/深度内参")

            # 每帧 depth_scale -> 点云滤镜
            try:
                ds = getattr(depth_frame, "get_depth_scale", None)
                if callable(ds):
                    depth_scale = float(ds())
            except Exception:
                pass
            if pc_filter is not None:
                try:
                    pc_filter.set_position_data_scaled(float(depth_scale))
                except Exception:
                    pass

            # color image
            t1 = time_sync()
            im0 = frame_to_bgr_image(color_frame)
            if im0 is None:
                LOGGER.warning(f"[Cam {device_index}] 彩色帧转换失败")
                continue
            try:
                im0_rgb = cv2.cvtColor(im0, cv2.COLOR_BGR2RGB)
            except Exception:
                im0_rgb = im0

            # 深度图（米）
            try:
                d_h = depth_frame.get_height()
                d_w = depth_frame.get_width()
                depth_format = depth_frame.get_format()
                data_bytes = depth_frame.get_data()
                depth_meters = None
                if depth_format == ob.OBFormat.Y16:
                    depth_raw = np.frombuffer(data_bytes, dtype=np.uint16)
                    if depth_raw.size == d_h * d_w:
                        depth_raw = depth_raw.reshape((d_h, d_w))
                        depth_meters = depth_raw.astype(np.float32) * float(depth_scale)
                # resize 到彩色分辨率（对齐后通常一致；此处稳妥）
                ch, cw = color_frame.get_height(), color_frame.get_width()
                if (depth_meters is not None) and ((depth_meters.shape[0] != ch) or (depth_meters.shape[1] != cw)):
                    depth_meters = cv2.resize(depth_meters, (cw, ch), interpolation=cv2.INTER_NEAREST)
            except Exception:
                depth_meters = None

            # PointCloudFilter -> xyz_map（用彩色分辨率重构）
            xyz_map = None
            if pc_filter is not None:
                try:
                    pc_frame = pc_filter.process(frames_aligned)
                    if pc_frame is not None:
                        ch, cw = color_frame.get_height(), color_frame.get_width()
                        buf = pc_frame.get_data()
                        arr = np.frombuffer(buf, dtype=np.float32)
                        if arr.size == ch * cw * 3:
                            xyz_map = arr.reshape(ch, cw, 3)
                        else:
                            if (arr.size % 3) == 0 and (arr.size // 3) >= (ch * cw):
                                xyz_map = arr[:ch * cw * 3].reshape(ch, cw, 3)
                            else:
                                LOGGER.warning(f"[Cam {device_index}] 点云数据尺寸不匹配: got {arr.size} floats, expect {ch * cw * 3}")
                                xyz_map = None
                except Exception as e:
                    LOGGER.warning(f"[Cam {device_index}] PointCloudFilter 处理失败: {e}")
                    xyz_map = None

            # 兜底：若点云滤镜不可用，直接用深度图+彩色内参生成 xyz_map（米）
            if xyz_map is None and (depth_meters is not None) and (camera_matrix is not None):
                xyz_map = depth_to_xyz_map(depth_meters, camera_matrix)

            # 单位兜底：若发现像毫米，则转米（只需做一次）
            if (xyz_map is not None) and (not last_unit_checked):
                z_med = np.nanmedian(np.abs(xyz_map[..., 2]))
                if np.isfinite(z_med) and z_med > 20.0:
                    xyz_map = xyz_map / 1000.0
                    LOGGER.info(f"[Cam {device_index}] 检测到点云疑似毫米单位，已自动转换为米")
                else:
                    LOGGER.info(f"[Cam {device_index}] 点云单位确认：米")
                last_unit_checked = True

            # YOLO 前处理
            letterboxed_img = letterbox(im0_rgb, imgsz, stride=model.stride, auto=getattr(model, 'pt', False))[0]
            im_tensor = torch.from_numpy(letterboxed_img).permute(2, 0, 1).unsqueeze(0).to(device)
            im_tensor = im_tensor.half() if half else im_tensor.float()
            im_tensor /= 255.0
            t2 = time_sync()
            dt[0] += (t2 - t1)

            # YOLO 推理
            pred = model(im_tensor, augment=opt.augment, visualize=opt.visualize)
            t3 = time_sync()
            dt[1] += (t3 - t2)
            pred = non_max_suppression_obb(pred, conf_thres=opt.conf_thres, iou_thres=opt.iou_thres,
                                           classes=opt.classes, agnostic=opt.agnostic_nms,
                                           multi_label=True, max_det=opt.max_det)
            t4 = time_sync()
            dt[2] += (t4 - t3)

            # 处理结果
            im0_annotated = im0_rgb.copy()
            annotator = Annotator(im0_annotated, line_width=opt.line_thickness, example=str(names))

            found_this_frame = False

            if len(pred) and len(pred[0]):
                det = pred[0]
                pred_poly = rbox2poly(det[:, :5])
                pred_poly = scale_polys(im_tensor.shape[2:], pred_poly, im0_rgb.shape)
                det_numpy = torch.cat((pred_poly, det[:, -2:]), dim=1).cpu().numpy()

                for *poly, conf, cls in reversed(det_numpy):
                    poly_list = list(poly)
                    label_text = f'{names[int(cls)]} {conf:.2f}'
                    cam_coords = None
                    world_coords = None
                    depth_val_m = None
                    u = v = None

                    if xyz_map is not None:
                        H, W = xyz_map.shape[:2]
                        mask = polygon_to_mask(poly_list, (H, W), erode_px=0)
                        pts = xyz_map[mask]
                        valid = np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 1e-9)
                        pts = pts[valid]

                        # if pts.shape[0] >= 50:
                        #     center3d, axis3d, n_used = estimate_cylinder_center_from_points(
                        #         pts,
                        #         known_radius_m=opt.cyl_radius_m,
                        #         voxel_size=opt.pc_voxel_size,
                        #         nb_neighbors=opt.pc_nb_neighbors,
                        #         std_ratio=opt.pc_std_ratio,
                        #         slice_thickness_m=opt.pc_slice_thickness_m,
                        #         min_points=opt.pc_min_points,
                        #         max_points=opt.pc_max_points,
                        #         force_circle_fit=opt.pc_use_circle_fit,
                        #         gate_z_max_m=gate_z_max_m,
                        #         gate_y_range_m=gate_y_range_m
                        #     )
                        # else:
                        #     center3d, axis3d, n_used = (None, None, 0)

                        if pts.shape[0] >= 50:
                            # 使用新的 V2 版本函数进行计算
                            center3d, axis3d, n_used = estimate_cylinder_center_from_points_v2(
                                pts,
                                known_radius_m=opt.cyl_radius_m,
                                voxel_size=opt.pc_voxel_size,
                                nb_neighbors=opt.pc_nb_neighbors,
                                std_ratio=opt.pc_std_ratio,
                                slice_thickness_m=opt.pc_slice_thickness_m,
                                min_points=opt.pc_min_points,
                                max_points=opt.pc_max_points,
                                gate_z_max_m=gate_z_max_m,
                                gate_y_range_m=gate_y_range_m
                            )
                        else:
                            center3d, axis3d, n_used = (None, None, 0)


                        if center3d is not None:
                            cam_coords = center3d  # 单位米
                            depth_val_m = float(np.median(pts[:, 2]))
                            u, v = project_point_to_pixel(camera_matrix, cam_coords)
                            world_coords = camera_to_world_ex(center3d, Tcw_use)

                            x_hist.append(float(center3d[0]))
                            x_smooth = float(np.median(x_hist)) if len(x_hist) else float(center3d[0])
                            delta_x = (x_smooth - x_baseline) if x_baseline is not None else None

                            label_text += f' | Z:{depth_val_m:.2f}m | Pts:{n_used}'
                            if delta_x is not None:
                                label_text += f' ΔX:{delta_x:.3f}m'

                            im0_annotated = draw_axes_at_point(im0_annotated, cam_coords, camera_matrix, dist_coeffs, axis_length=0.05)

                            if opt.pc_viz and HAS_O3D and (pts.shape[0] >= 10):
                                try:
                                    show_o3d_debug_like_user_pc(
                                        xyz_map, poly_list, center3d, axis3d,
                                        voxel_mm=opt.pc_viz_voxel_mm, outlier_nn=opt.pc_viz_outlier_nn,
                                        outlier_std=opt.pc_viz_outlier_std,
                                        sphere_mm=opt.pc_viz_sphere_mm, axis_mm=opt.pc_viz_axis_mm
                                    )
                                except Exception as e:
                                    LOGGER.warning(f"[Cam {device_index}] pc-viz 可视化失败: {e}")
                        else:
                            # 回退：像素中心 + 深度回投
                            if depth_meters is not None:
                                u, v = get_fused_u_for_symmetric_cylinder(depth_meters, poly_list, camera_matrix)
                                depth_val_m = depth_from_roi(depth_meters, poly_list)
                                if depth_val_m is not None and depth_val_m > 0:
                                    cam_coords = pixel_to_camera((u, v), depth_val_m, camera_matrix, dist_coeffs, opt.pp_delta)
                                    if cam_coords is not None:
                                        world_coords = camera_to_world_ex(cam_coords, Tcw_use)
                                        label_text += f' | Z:{depth_val_m:.2f}m'
                                        im0_annotated = draw_axes_at_point(im0_annotated, cam_coords, camera_matrix, dist_coeffs, axis_length=0.05)
                    else:
                        # xyz_map 不可用时，回退像素法
                        if depth_meters is not None:
                            u, v = get_fused_u_for_symmetric_cylinder(depth_meters, poly_list, camera_matrix)
                            depth_val_m = depth_from_roi(depth_meters, poly_list)
                            if depth_val_m is not None and depth_val_m > 0:
                                cam_coords = pixel_to_camera((u, v), depth_val_m, camera_matrix, dist_coeffs, opt.pp_delta)
                                if cam_coords is not None:
                                    world_coords = camera_to_world_ex(cam_coords, Tcw_use)
                                    label_text += f' | Z:{depth_val_m:.2f}m'
                                    im0_annotated = draw_axes_at_point(im0_annotated, cam_coords, camera_matrix, dist_coeffs, axis_length=0.05)
                        else:
                            label_text += ' | Depth:N/A'

                    annotator.poly_label(poly_list, label_text, color=colors(int(cls), True))

                    # CSV写入（世界坐标 X/Y 交换后写出，单位 mm）
                    if csv_writer is not None:
                        row = [processed_frames_count, device_index, serial_sn, names[int(cls)], float(conf),
                               None if u is None else float(u),
                               None if v is None else float(v),
                               None if depth_val_m is None else float(depth_val_m)]
                        if cam_coords is not None:
                            row += [float(cam_coords[0]) * 1000, float(cam_coords[1]) * 1000,
                                    float(cam_coords[2]) * 1000]
                        else:
                            row += [None, None, None]
                        if world_coords is not None:
                            wx_mm = float(world_coords[0] * 1000.0)  # X_out
                            wy_mm = float(world_coords[1] * 1000.0)  # Y_out
                            wz_mm = float(world_coords[2] * 1000.0)  # Z_out
                            area_id_csv = classify_area_for_cam(wx_mm, wy_mm, wz_mm, serial_sn)
                            row += [wx_mm, wy_mm, wz_mm, area_id_csv]
                        else:
                            row += [None, None, None, None]
                        csv_writer.writerow(row)



                    # 聚合：仅当匹配目标类别时记录一条世界坐标（交换后）
                    if world_coords is not None:
                        cls_name = names[int(cls)]
                        if cls_name.lower() == target_name_lc:

                            # (1)已交换后的世界坐标
                            wx_mm = float(world_coords[0] * 1000.0)  # swapped
                            wy_mm = float(world_coords[1] * 1000.0)  # swapped
                            wz_mm = float(world_coords[2] * 1000.0)

                            # (2)区域判定(基于已交换的世界坐标)
                            # area_id = classify_area_for_cam(wx_mm, wy_mm, wz_mm, device_index)

                            area_id = classify_area_for_cam(wx_mm, wy_mm, wz_mm, serial_sn)

                            # (3)ID 从 1 开始
                            out_id = int(device_index + ID_OFFSET)

                            if names[int(cls)] == "dan":
                                cls_name_string = "榴弹炮"
                            else:
                                cls_name_string = "灭火器"

                            line = format_vision_line(wx_mm, wy_mm, wz_mm, cls_name_string, area_id, out_id)
                            conf_val = float(conf)
                            prev = area_best.get(area_id)
                            if (prev is None) or (conf_val > prev[0]):
                                area_best[area_id] = (conf_val, line)

                            found_this_frame = True

                            # 可选：兼容旧行为，逐检测即时发送（默认关闭）
                            if getattr(opt, 'send_per_detect', False):
                                try:
                                    udp_send_text(line, opt.udp_ip, opt.udp_port, timeout=opt.udp_timeout,
                                                  wait_ack=True)
                                    LOGGER.info(f"[UDP] 即时发送 -> {opt.udp_ip}:{opt.udp_port}")
                                except Exception as e:
                                    LOGGER.warning(f"[UDP] 即时发送失败: {e}")


                # 如果本帧已找到目标且要求“找到一次就结束”，则跳出
                if found_this_frame and opt.break_after_first:
                    got_valid_world = True
                    LOGGER.info(f"[Cam {device_index}] 已获取一次有效世界坐标，结束该相机会话。")
                    break


            if getattr(opt, 'areas_complete_stop', True):
                if set(area_best.keys()) >= areas_target:
                    LOGGER.info(f"[Cam {device_index}] 已覆盖目标区域 {sorted(list(areas_target))}，提前结束。")
                    got_valid_world = True  # 可选：标记一下
                    break
            elif getattr(opt, 'break_after_first', True) and found_this_frame:
                LOGGER.info(f"[Cam {device_index}] 检测到一条后结束（break_after_first）")
                got_valid_world = True
                break

            im0_annotated = annotator.result()

            if view_img:
                try:
                    cv2.imshow(f"YOLOv5-OBB Detection | Cam {device_index}", cv2.cvtColor(im0_annotated, cv2.COLOR_RGB2BGR))
                except Exception:
                    cv2.imshow(f"YOLOv5-OBB Detection | Cam {device_index}", im0_annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    raise KeyboardInterrupt

            if video_writer is not None and not opt.nosave:
                if video_writer.isOpened():
                    try:
                        video_writer.write(cv2.cvtColor(im0_annotated, cv2.COLOR_RGB2BGR))
                    except Exception:
                        video_writer.write(im0_annotated)

            processed_frames_count += 1
            LOGGER.info(f"[Cam {device_index}] Frame {processed_frames_count}: pre:{(t2 - t1):.3f}s, infer:{(t3 - t2):.3f}s, nms:{(t4 - t3):.3f}s")

    except KeyboardInterrupt:
        LOGGER.info(f"[Cam {device_index}] 用户中断。")
    except Exception as e:
        LOGGER.error(f"[Cam {device_index}] 处理循环出错: {e}")
    finally:
        LOGGER.info(f"[Cam {device_index}] 停止相机...")
        try:
            pipeline.stop()
        except Exception:
            pass
        if video_writer is not None:
            try:
                video_writer.release()
                LOGGER.info(f"[Cam {device_index}] 检测视频已保存到 {video_path}")
            except Exception:
                pass
        if csv_file is not None:
            csv_file.close()
        cv2.destroyAllWindows()

        # 写各自相机的 world_coords.txt（不发送）
        try:
            with open(world_txt_path, 'w', encoding='utf-8') as f:
                # 按区域编号排序写入
                for aid, (_, line) in sorted(area_best.items()):
                    f.write(line + "\n")
            LOGGER.info(f"[Cam {device_index}] 已写入 {world_txt_path}")
        except Exception as e:
            LOGGER.warning(f"[Cam {device_index}] 写 world_coords.txt 失败: {e}")


    return {
        'cam_idx': device_index,
        'serial': serial_sn,
        'save_dir': str(save_dir),
        'lines_by_area': {aid: line for aid, (conf, line) in area_best.items()}
    }

# ---------- 顺序多相机主流程 ----------
@torch.no_grad()
def run_sequential_multicams(weights=ROOT / 'runs/best.pt',
                             device='',
                             project=ROOT / 'runs/detect',
                             name='exp',
                             exist_ok=False,
                             imgsz=(640, 640),
                             conf_thres=0.70,
                             iou_thres=0.45,
                             max_det=1000,
                             view_img=True,
                             save_txt=False,
                             save_conf=False,
                             save_crop=False,
                             nosave=False,
                             classes=None,
                             agnostic_nms=False,
                             augment=False,
                             visualize=False,
                             update=False,
                             line_thickness=3,
                             hide_labels=False,
                             hide_conf=False,
                             half=False,
                             dnn=False,
                             mode='SW',
                             enable_sync=True,
                             center_mode='poly',
                             depth_percentiles=(25, 75),
                             pp_delta=(0.0, 0.0),
                             save_csv=True,
                             pc_backend='open3d',
                             cyl_radius_m=0.06,
                             pc_voxel_size=0.005,
                             pc_nb_neighbors=30,
                             pc_std_ratio=1.5,
                             pc_slice_thickness_m=0.02,
                             pc_min_points=200,
                             pc_max_points=60000,
                             pc_use_circle_fit=True,
                             pc_smooth=5,
                             pc_viz=True,
                             pc_viz_like_user=True,
                             pc_viz_sphere_mm=2.0,
                             pc_viz_axis_mm=50.0,
                             pc_viz_voxel_mm=1.0,
                             pc_viz_outlier_nn=20,
                             pc_viz_outlier_std=2.0,
                             pc_gate_z_max_mm=1800,
                             pc_gate_y_min_mm=80,
                             pc_gate_y_max_mm=400,
                             udp_ip='192.168.1.210',
                             udp_port=11001,
                             udp_chunk=1024,
                             udp_timeout=2.0,
                             udp_mode='file_ack',
                             target_name='dan',
                             extrinsics='extrinsics.yaml',
                             voice_stdin=False,
                             # 新增的4个参数（要与 argparse 一致）
                             send_agg_text=True,
                             send_agg_file=False,
                             send_per_detect=False,
                             areas_complete_stop=True,
                             # 顺序多相机控制
                             max_cams=2,
                             max_wait_s=40.0,
                             break_after_first=True,

                             continuous_mode=True,
                             send_interval=15.0):

    if not HAS_ORBBEC:
        raise RuntimeError("pyorbbecsdk 未安装或不可用。")

    # 设备选择与模型加载
    device_t = select_device(device)
    model = DetectMultiBackend(weights, device=device_t, dnn=dnn)
    stride, names = model.stride, model.names
    imgsz = check_img_size(imgsz, s=stride)
    if hasattr(model, 'pt') and (model.pt or model.jit):
        model.model.half() if (half and device_t.type != 'cpu') else model.model.float()

    # 外参
    extrinsics_map = load_extrinsics_yaml(extrinsics)

    # 会话目录（全局 exp，一次会话一个目录）
    session_save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
    session_save_dir.mkdir(parents=True, exist_ok=True)
    LOGGER.info(f"[Session] 保存目录: {str(session_save_dir)}")

    # 设备数量
    ctx = ob.Context()
    dev_list = ctx.query_devices()
    dev_count = dev_list.get_count()
    if dev_count <= 0:
        raise RuntimeError("未找到 Orbbec 设备")
    n_use = min(int(max_cams), dev_count)
    LOGGER.info(f"共检测到 {dev_count} 台设备，按顺序处理前 {n_use} 台。")

    # 单线程 UDP 发送器
    sender = UdpFileSender(udp_ip, int(udp_port), mode=str(udp_mode).lower(),
                           chunk_size=int(udp_chunk), timeout=float(udp_timeout))
    sender.start()

    # 会话开始（可选，不等ACK）
    # try:
    #     udp_send_text("VISION_SESSION:START", udp_ip, udp_port, timeout=udp_timeout, wait_ack=False)
    # except Exception as e:
    #     LOGGER.warning(f"[UDP] 会话开始标记发送失败: {e}")


    # 语音线程（如果需要），保存到会话目录
    if voice_stdin:
        threading.Thread(target=voice_thread, args=(sender, session_save_dir), daemon=True).start()

    # ========== 新增：持续运行模式控制 ==========
    iteration = 0  # 迭代计数器

    while True:  # 外层循环
        iteration += 1
        cycle_start_time = time.time()

        if continuous_mode:
            LOGGER.info(f"\n{'=' * 60}")
            LOGGER.info(f"[持续模式] 第 {iteration} 轮处理开始")
            LOGGER.info(f"{'=' * 60}\n")

        # ========== 原有的相机处理逻辑 ==========
        results = []
        for cam_idx in range(n_use):
            LOGGER.info(f"========== 开始处理相机 Cam {cam_idx} ==========")
            res = process_one_camera(model, device_t, names, cam_idx,
                                     argparse.Namespace(**locals()), sender, extrinsics_map, session_save_dir)
            results.append(res)
            LOGGER.info(
                f"========== 完成相机 Cam {cam_idx}：areas={sorted(list(res.get('lines_by_area', {}).keys()))} ==========\n")
            time.sleep(0.5)

        # 1) 聚合所有相机的区域行
        agg_lines = []
        results_sorted = sorted(results, key=lambda x: x['cam_idx'])
        for r in results_sorted:
            for aid, line in sorted(r.get('lines_by_area', {}).items()):
                agg_lines.append(line)

        # 2) 写会话聚合文件（持续模式下添加时间戳）
        if continuous_mode:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            agg_txt_path = session_save_dir / f'world_coords_agg_{timestamp}.txt'
        else:
            agg_txt_path = session_save_dir / 'world_coords_agg.txt'

        try:
            with open(agg_txt_path, 'w', encoding='utf-8') as f:
                for line in agg_lines:
                    f.write(line + '\n')
            LOGGER.info(f"[Session] 已聚合写入 {str(agg_txt_path)}")
        except Exception as e:
            LOGGER.warning(f"[Session] 聚合写文件失败: {e}")

        # 3) 一次性 UDP 发送（文本）
        try:
            if send_agg_text and len(agg_lines) > 0:
                payload = "\n".join(agg_lines)
                udp_send_text(payload, udp_ip, udp_port, timeout=udp_timeout, wait_ack=True)
                LOGGER.info(f"[UDP] 已一次性发送 {len(agg_lines)} 行聚合文本")
        except Exception as e:
            LOGGER.warning(f"[UDP] 发送聚合文本失败: {e}")

        # 4) 可选：通过文件模式再发聚合文件
        try:
            if send_agg_file and len(agg_lines) > 0:
                sender.enqueue(str(agg_txt_path))
                LOGGER.info(f"[UDP] 已发送聚合文件 {str(agg_txt_path)}（{udp_mode}）")
        except Exception as e:
            LOGGER.warning(f"[UDP] 发送聚合文件失败: {e}")

        # ========== 持续模式控制逻辑 ==========
        if not continuous_mode:
            # 非持续模式：处理一次后退出
            LOGGER.info("顺序多相机处理结束。")
            break
        else:
            # 持续模式：计算等待时间
            cycle_elapsed = time.time() - cycle_start_time
            wait_time = max(0, send_interval - cycle_elapsed)

            LOGGER.info(f"\n[持续模式] 第 {iteration} 轮完成")
            LOGGER.info(f"[持续模式] 本轮耗时: {cycle_elapsed:.2f}秒")
            LOGGER.info(f"[持续模式] 等待 {wait_time:.2f}秒 后开始下一轮...")
            LOGGER.info(
                f"[持续模式] 下次发送时间: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time() + wait_time))}\n")

            # 检查是否有键盘中断
            try:
                time.sleep(wait_time)
            except KeyboardInterrupt:
                LOGGER.info("[持续模式] 用户中断，退出持续运行。")
                break
    # ======================================

    return results

# ---------- 单相机（保持兼容） ----------
@torch.no_grad()
def run(weights=ROOT / 'runs/best.pt',
        source='1',
        imgsz=(640, 640),
        conf_thres=0.70,
        iou_thres=0.45,
        max_det=1000,
        device='',
        view_img=True,
        save_txt=False,
        save_conf=False,
        save_crop=False,
        nosave=False,
        classes=None,
        agnostic_nms=False,
        augment=False,
        visualize=False,
        update=False,
        project=ROOT / 'runs/detect',
        name='exp',
        exist_ok=False,
        line_thickness=3,
        hide_labels=False,
        hide_conf=False,
        half=False,
        dnn=False,
        mode='SW',
        enable_sync=True,
        center_mode='poly',
        depth_percentiles=(25, 75),
        pp_delta=(0.0, 0.0),
        save_csv=True,
        # 点云拟合参数
        pc_backend='open3d',
        cyl_radius_m=0.06,
        pc_voxel_size=0.005,
        pc_nb_neighbors=30,
        pc_std_ratio=1.5,
        pc_slice_thickness_m=0.02,
        pc_min_points=200,
        pc_max_points=60000,
        pc_use_circle_fit=True,
        pc_smooth=5,
        # 可视化
        pc_viz=True,
        pc_viz_like_user=True,
        pc_viz_sphere_mm=2.0,
        pc_viz_axis_mm=50.0,
        pc_viz_voxel_mm=1.0,
        pc_viz_outlier_nn=20,
        pc_viz_outlier_std=2.0,
        # 几何阈值（mm）
        pc_gate_z_max_mm=1400,
        pc_gate_y_min_mm=80,
        pc_gate_y_max_mm=400,
        # UDP/外参/语音
        udp_ip='192.168.1.210',
        udp_port=11002,
        udp_chunk=1024,
        udp_timeout=2.0,
        udp_mode='file_ack',
        target_name='dan',
        extrinsics='extrinsics.yaml',
        voice_stdin=False,
        # 单相机模式下，也支持“获取一次就退出/超时退出”以保持一致体验
        max_wait_s=40.0,
        break_after_first=True,
        continuous_mode=True,
        send_interval=15.0):
    # 调为“顺序模式但只处理一台”
    return run_sequential_multicams(weights=weights, device=device, project=project, name=name, exist_ok=exist_ok,
                                    imgsz=imgsz, conf_thres=conf_thres, iou_thres=iou_thres, max_det=max_det,
                                    view_img=view_img, save_txt=save_txt, save_conf=save_conf, save_crop=save_crop, nosave=nosave,
                                    classes=classes, agnostic_nms=agnostic_nms, augment=augment, visualize=visualize, update=update,
                                    line_thickness=line_thickness, hide_labels=hide_labels, hide_conf=hide_conf, half=half, dnn=dnn,
                                    mode=mode, enable_sync=enable_sync, center_mode=center_mode, depth_percentiles=depth_percentiles,
                                    pp_delta=pp_delta, save_csv=save_csv, pc_backend=pc_backend, cyl_radius_m=cyl_radius_m,
                                    pc_voxel_size=pc_voxel_size, pc_nb_neighbors=pc_nb_neighbors, pc_std_ratio=pc_std_ratio,
                                    pc_slice_thickness_m=pc_slice_thickness_m, pc_min_points=pc_min_points, pc_max_points=pc_max_points,
                                    pc_use_circle_fit=pc_use_circle_fit, pc_smooth=pc_smooth, pc_viz=pc_viz, pc_viz_like_user=pc_viz_like_user,
                                    pc_viz_sphere_mm=pc_viz_sphere_mm, pc_viz_axis_mm=pc_viz_axis_mm, pc_viz_voxel_mm=pc_viz_voxel_mm,
                                    pc_viz_outlier_nn=pc_viz_outlier_nn, pc_viz_outlier_std=pc_viz_outlier_std,
                                    pc_gate_z_max_mm=pc_gate_z_max_mm, pc_gate_y_min_mm=pc_gate_y_min_mm, pc_gate_y_max_mm=pc_gate_y_max_mm,
                                    udp_ip=udp_ip, udp_port=udp_port, udp_chunk=udp_chunk, udp_timeout=udp_timeout, udp_mode=udp_mode,
                                    target_name=target_name, extrinsics=extrinsics, voice_stdin=voice_stdin,
                                    max_cams=1, max_wait_s=max_wait_s, break_after_first=break_after_first,continuous_mode=continuous_mode, send_interval=send_interval)

# ---------- argparse ----------
def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=str(ROOT / 'runs/best.pt'), help='model path(s)')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or cpu')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.70, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--view-img', action='store_true', default=True, help='show results')
    parser.add_argument('--save-txt', action='store_true', default=False, help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', default=False, help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', default=False, help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', default=False, help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, default=None, help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', default=False, help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', default=False, help='augmented inference')
    parser.add_argument('--visualize', action='store_true', default=False, help='visualize features')
    parser.add_argument('--update', action='store_true', default=False, help='update all models')
    parser.add_argument('--project', default=str(ROOT / 'runs/detect'), help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', default=False, help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', dest='half', default=False, help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', default=False, help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--mode', type=str, default='SW', help='Orbbec align mode: HW / SW / NONE')
    parser.add_argument('--enable_sync', type=bool, default=True, help='enable frame sync')

    # 额外
    parser.add_argument('--center-mode', type=str, default='poly', choices=['poly', 'bbox', 'mass'], help='像素中心计算方式')
    parser.add_argument('--depth-percentiles', nargs=2, type=float, default=[25, 75], help='ROI 深度百分位（兜底像素法）')
    parser.add_argument('--pp-delta', nargs=2, type=float, default=[0.0, 0.0], help='主点微调 du dv（像素）')
    parser.add_argument('--save-csv', action='store_true', default=True, help='保存 3D 结果到 CSV')

    # 点云拟合参数
    parser.add_argument('--pc-backend', type=str, default='open3d', choices=['auto', 'open3d', 'numpy'], help='点云处理后端')
    parser.add_argument('--cyl-radius-m', type=float, default=0.06, help='圆柱半径(米)')
    parser.add_argument('--pc-voxel-size', type=float, default=0.005, help='体素下采样(米)')
    parser.add_argument('--pc-nb-neighbors', type=int, default=30, help='统计滤波邻居数')
    parser.add_argument('--pc-std-ratio', type=float, default=1.5, help='统计滤波std阈值')
    parser.add_argument('--pc-slice-thickness-m', type=float, default=0.02, help='截面厚度(米)')
    parser.add_argument('--pc-min-points', type=int, default=200, help='截面最少点数')
    parser.add_argument('--pc-max-points', type=int, default=60000, help='ROI最大点数')
    parser.add_argument('--pc-use-circle-fit', action='store_true', default=True, help='使用圆拟合求截面中心')
    parser.add_argument('--pc-smooth', type=int, default=5, help='X平滑窗口')

    # 可视化
    parser.add_argument('--pc-viz', action='store_true', default=True, help='Open3D可视化（阻塞）')
    parser.add_argument('--pc-viz-like-user', action='store_true', default=True, help='三步可视化风格')
    parser.add_argument('--pc-viz-sphere-mm', type=float, default=2.0, help='红球半径（mm）')
    parser.add_argument('--pc-viz-axis-mm', type=float, default=50.0, help='轴线半长（mm）')
    parser.add_argument('--pc-viz-voxel-mm', type=float, default=1.0, help='体素（mm）')
    parser.add_argument('--pc-viz-outlier-nn', type=int, default=20, help='离群滤波邻居数')
    parser.add_argument('--pc-viz-outlier-std', type=float, default=2.0, help='离群滤波阈值')

    # 几何阈值（mm）
    parser.add_argument('--pc-gate-z-max-mm', type=float, default=1600, help='Z<=此值(mm)')
    parser.add_argument('--pc-gate-y-min-mm', type=float, default=100, help='Y>=此值(mm)')
    parser.add_argument('--pc-gate-y-max-mm', type=float, default=420, help='Y<=此值(mm)')

    # UDP/外参/语音
    parser.add_argument('--udp-ip', type=str, default='127.0.0.1', help='UDP receiver IP')
    parser.add_argument('--udp-port', type=int, default=11002, help='UDP receiver port')
    parser.add_argument('--udp-chunk', type=int, default=1024, help='UDP chunk size bytes')
    parser.add_argument('--udp-timeout', type=float, default=2.0, help='UDP timeout seconds')
    parser.add_argument('--udp-mode', type=str, default='file_ack', choices=['raw', 'file_ack'], help='UDP send mode')
    parser.add_argument('--target-name', type=str, default='dan', help='仅对该类别写 TXT/UDP')
    parser.add_argument('--extrinsics', type=str, default='extrinsics.yaml', help='per-camera extrinsics yaml')
    parser.add_argument('--voice-stdin', action='store_true', default=False, help='启用stdin语音占位->UDP')

    parser.add_argument('--send-agg-text', action='store_true', default=True, help='会话结束时一次性发送聚合文本')
    parser.add_argument('--send-agg-file', action='store_true', default=False, help='会话结束时发送聚合文件 world_coords_agg.txt')
    parser.add_argument('--send-per-detect', action='store_true', default=False, help='逐检测即时发送（兼容旧行为）；默认关闭')
    parser.add_argument('--areas-complete-stop', action='store_true', default=True, help='相机侧：收齐该相机的所有目标区域后提前结束')

    # 顺序多相机
    parser.add_argument('--max-cams', type=int, default=3, help='顺序处理的相机数量上限')
    parser.add_argument('--max-wait-s', type=float, default=5.0, help='每台相机最长等待秒数（无有效世界坐标则记 NONE）')
    parser.add_argument('--break-after-first', action='store_true', default=True, help='找到一次有效世界坐标后立即结束该相机')

    parser.add_argument('--continuous-mode', action='store_true', default=True,help='启用持续运行模式，程序会循环处理所有相机并定期发送结果')
    parser.add_argument('--send-interval', type=float, default=15.0, help='持续模式下的发送间隔（秒），必须大于单次处理所有相机的时间')


    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1
    print_args(FILE.stem, opt)
    return opt

def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    # 直接进入“顺序多相机”模式（若只插一台，--max-cams 设为1或程序会检测只有1台）
    run_sequential_multicams(**vars(opt))

if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
