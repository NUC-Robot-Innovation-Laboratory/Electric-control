# ******************************************************************************
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
YOLOv5 + Orbbec Depth 支持版 + 世界坐标转换
基于 Ultralytics YOLOv5-OBB
"""
# ******************************************************************************

import argparse
import os
import sys
import time
from pathlib import Path
import numpy as np
import cv2
import torch
import torch.backends.cudnn as cudnn
import csv
import math

# ===================== 修复导入路径：支持在任意目录运行 ======================
FILE = Path(__file__).resolve()

# 尝试在本文件向上搜索包含 'models' 和 'utils' 的 YOLOv5 根目录
ROOT = None
for p in [FILE.parent] + list(FILE.parents):
    if (p / 'models').exists() and (p / 'utils').exists():
        ROOT = p
        break
# 环境变量兜底
if ROOT is None:
    yv = os.environ.get('YOLOV5_PATH', '')
    if yv and (Path(yv) / 'models').exists():
        ROOT = Path(yv)

if ROOT is None:
    # 最后兜底为当前目录
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

# YOLOv5 相关导入（保持与你工程一致）
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

# （可选）默认相机内参（如未能从设备读取时采用）
# FALLBACK_CAMERA_MATRIX = np.array([
#     [468.13320439, 0.0, 649.32089063],
#     [0.0, 474.38540911, 367.82500508],
#     [0.0, 0.0, 1.0]
# ], dtype=np.float32)

FALLBACK_CAMERA_MATRIX = np.array([
    [611.572, 0.0, 643.59109755],
    [0.0, 611.817, 341.29027134],
    [0.0, 0.0, 1.0]
], dtype=np.float32)


FALLBACK_DIST_COEFFS = np.array([-0.12393594, 0.0467302, 0.00263421, -0.00696342, -0.01053551], dtype=np.float32)

# 世界变换矩阵（示例），平移单位以米为准
TRANSFORM_MATRIX = np.array([
    [9.30724545e-01, -2.57750005e-0, -2.59454727e-01, 68],
    [6.86573375e-02, 8.19962375e-01, -5.68285028e-01, 1200],
    [3.59218583e-01, 5.11103353e-01,  7.80855538e-01, 1520],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float32)
TRANSFORM_MATRIX[:3, 3] /= 1000.0  # 将 mm -> m

# ---------- 帮助函数 ----------
def frame_to_bgr_image(frame):
    if frame is None:
        return None
    try:
        data = frame.get_data()
        if data is None:
            return None
        arr = np.frombuffer(data, dtype=np.uint8)
        h, w = frame.get_height(), frame.get_width()
        if arr.size == h * w * 3:
            img = arr.reshape((h, w, 3))
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            try:
                img = arr.reshape((h, w))
                img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                return img_color
            except Exception as e:
                LOGGER.warning(f"frame_to_bgr_image reshape failed: {e}")
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


# def pixel_to_camera(pixel_coords, depth_value, camera_matrix, dist_coeffs, pp_delta=(0.0, 0.0)):
#     if depth_value is None or depth_value <= 0:
#         return None
#     u = float(pixel_coords[0]) + float(pp_delta[0])
#     v = float(pixel_coords[1]) + float(pp_delta[1])
#     if camera_matrix is None:
#         LOGGER.warning("相机内参缺失")
#         return None
#     pts = np.array([[[u, v]]], dtype=np.float32)
#     try:
#         # und = cv2.undistortPoints(pts, camera_matrix, dist_coeffs, P=None)
#         # xn, yn = und[0, 0]

#         # # X = xn * depth_value
#         # # Y = yn * depth_value
#         # # Z = depth_value

#         # X = xn
#         # Y = yn
#         # Z = depth_value
#         # Z_start = depth_value + 0.06
#         # # 方位角
#         # result_θ = math.atan(X)
#         # print("方位角：", result_θ)
#         # # 俯仰角
#         # result_α = math.atan(Y)
#         # print("方位角：", result_α)
#         # X_end = Z_start * math.sin(result_α)

#         fx = camera_matrix[0, 0] * 0.5
#         fy = camera_matrix[1, 1]
#         # cx = camera_matrix[0, 2]
#         # cy = camera_matrix[1, 2]
#         cx = 320
#         cy = 320

#         # X = (u - cx) * depth_value / fx
#         # Y = (v - cy) * depth_value / fy
#         # Z = depth_value
#         X = (u - cx) / fx
#         Y = (v - cy) / fy
#         Z = depth_value
#         Z_start = depth_value + 0.06

#         # 方位角
#         result_θ = math.atan(X)
#         print("方位角：", result_θ)
#         # 俯仰角
#         result_α = math.atan(Y)
#         print("俯仰角：", result_α)

#         X_end = Z_start * math.sin(result_θ)

#         return np.array([X_end, Y, Z], dtype=np.float32)
#     except Exception as e:
#         LOGGER.warning(f"undistortPoints failed: {e}. Fallback to raw pinhole.")
#         fx = camera_matrix[0, 0]
#         fy = camera_matrix[1, 1]
#         cx = camera_matrix[0, 2]
#         cy = camera_matrix[1, 2]

#         # X = (u - cx) * depth_value / fx
#         # Y = (v - cy) * depth_value / fy
#         # Z = depth_value
#         X = (u - cx)  / fx
#         Y = (v - cy) / fy
#         Z = depth_value
#         Z_start = depth_value + 0.06

#         # 方位角
#         result_θ = math.atan(X)
#         print("方位角：", result_θ)
#         # 俯仰角
#         result_α = math.atan(Y)
#         print("俯仰角：", result_α)

#         X_end = Z_start * math.sin(result_θ)

#         return np.array([X_end, Y, Z], dtype=np.float32)

def get_camera_intrinsics(pipeline):
    """
    从 Orbbec pipeline 获取相机内参
    返回内参对象、相机矩阵、畸变系数
    """
    try:
        frames = pipeline.wait_for_frames(1000)
        if frames is None:
            LOGGER.warning("无法获取帧")
            return None, None, None

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if depth_frame is None:
            LOGGER.warning("无法获取深度帧")
            return None, None, None

        # 获取深度相机内参
        depth_profile = depth_frame.get_stream_profile()
        depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsic()

        # 打印深度相机内参
        print("\n===== Depth Camera Intrinsics =====")
        print(f"fx: {depth_intrinsics.fx}, fy: {depth_intrinsics.fy}")
        print(f"cx: {depth_intrinsics.cx}, cy: {depth_intrinsics.cy}")
        print(f"width: {depth_intrinsics.width}, height: {depth_intrinsics.height}")

        # 构建相机矩阵
        camera_matrix = np.array([
            [depth_intrinsics.fx, 0, depth_intrinsics.cx],
            [0, depth_intrinsics.fy, depth_intrinsics.cy],
            [0, 0, 1]

        ], dtype=np.float32)

        # 获取畸变系数
        depth_distortion = depth_profile.as_video_stream_profile().get_distortion()
        dist_coeffs = np.array([
            depth_distortion.k1,
            depth_distortion.k2,
            depth_distortion.p1,
            depth_distortion.p2,
            depth_distortion.k3
        ], dtype=np.float32)

        # 如果有彩色相机，也打印彩色相机内参
        if color_frame is not None:
            color_profile = color_frame.get_stream_profile()
            color_intrinsics = color_profile.as_video_stream_profile().get_intrinsic()
            print("\n===== Color Camera Intrinsics =====")
            print(f"fx: {color_intrinsics.fx}, fy: {color_intrinsics.fy}")
            print(f"cx: {color_intrinsics.cx}, cy: {color_intrinsics.cy}")
            print(f"width: {color_intrinsics.width}, height: {color_intrinsics.height}")

        return depth_intrinsics, camera_matrix, dist_coeffs

    except Exception as e:
        LOGGER.warning(f"获取相机内参失败: {e}")
        return None, None, None


def pixel_to_camera(pixel_coords, depth_value, camera_matrix_or_intrinsics, dist_coeffs=None, pp_delta=(0.0, 0.0)):
    """
    将像素坐标转换为相机坐标系

    参数:
        pixel_coords: 像素坐标 (u, v)
        depth_value: 深度值
        camera_matrix_or_intrinsics: 相机矩阵(numpy array) 或 Orbbec SDK的内参对象
        dist_coeffs: 畸变系数
        pp_delta: 主点偏移
    """
    if depth_value is None or depth_value <= 0:
        return None

    u = float(pixel_coords[0]) + float(pp_delta[0])
    v = float(pixel_coords[1]) + float(pp_delta[1])

    # 判断输入类型并提取内参
    if hasattr(camera_matrix_or_intrinsics, 'fx'):  # Orbbec SDK 内参对象
        intrinsics = camera_matrix_or_intrinsics
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.cx
        cy = intrinsics.cy

        # 打印使用的内参（可选）
        print(f"使用SDK内参: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

    elif isinstance(camera_matrix_or_intrinsics, np.ndarray):  # numpy 相机矩阵
        camera_matrix = camera_matrix_or_intrinsics
        if camera_matrix is None:
            LOGGER.warning("相机内参缺失")
            return None
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
    else:
        LOGGER.warning("未知的内参类型")
        return None

    try:
        fx = fx * 0.5  # 根据您的需求调整
        cx = 320  
        cy = 320  

        X = (u - cx) / fx
        Y = (v - cy) / fy
        Z = depth_value
        Z_start = depth_value + 0.06

        # 方位角
        result_θ = math.atan(X)
        print(f"方位角: {math.degrees(result_θ):.2f}°")

        # 俯仰角
        result_α = math.atan(Y)
        print(f"俯仰角: {math.degrees(result_α):.2f}°")

        # X_end = Z_start * math.sin(result_θ)
        X_end = X + 0.06 * math.sin(result_θ) 

        return np.array([X, Y, Z], dtype=np.float32)

    except Exception as e:
        LOGGER.warning(f"坐标转换失败: {e}. 使用回退计算")

        # 回退计算（不使用特殊的 fx 缩放）
        if hasattr(camera_matrix_or_intrinsics, 'fx'):
            fx = intrinsics.fx
            fy = intrinsics.fy
            cx = intrinsics.cx
            cy = intrinsics.cy
        else:
            fx = camera_matrix[0, 0]
            fy = camera_matrix[1, 1]
            cx = camera_matrix[0, 2]
            cy = camera_matrix[1, 2]

        X = (u - cx) / fx
        Y = (v - cy) / fy
        Z = depth_value
        Z_start = depth_value + 0.06

        # 方位角
        result_θ = math.atan(X)
        print(f"方位角: {math.degrees(result_θ):.2f}°")

        # 俯仰角
        result_α = math.atan(Y)
        print(f"俯仰角: {math.degrees(result_α) :.2f}°")

        # X_end = Z_start * math.sin(result_θ)
        X_end = X + 0.06 * math.sin(result_θ) 
        Z_start * math.sin(result_θ)

        return np.array([X, Y, Z], dtype=np.float32)


def draw_axes_at_point(image, point3D, camera_matrix, dist_coeffs, axis_length=0.05):
    if point3D is None or camera_matrix is None:
        return image
    axis = np.float32([[0,0,0],[axis_length,0,0],[0,axis_length,0],[0,0,axis_length]]).reshape(-1,3)
    rvec = np.zeros((3,1), dtype=np.float32)
    tvec = np.array(point3D, dtype=np.float32).reshape(3,1)
    try:
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
        imgpts = imgpts.astype(int)
        p0 = tuple(imgpts[0].ravel()); pX = tuple(imgpts[1].ravel()); pY = tuple(imgpts[2].ravel()); pZ = tuple(imgpts[3].ravel())
        cv2.line(image, p0, pX, (0,0,255), 2)
        cv2.line(image, p0, pY, (0,255,0), 2)
        cv2.line(image, p0, pZ, (255,0,0), 2)
        cv2.circle(image, p0, 3, (0,0,0), -1)
    except Exception as e:
        LOGGER.warning(f"draw_axes failed: {e}")
    return image

def polygon_center(poly_xyxyxyxy, mode='poly'):
    """poly: (8,) 或 (4,2)。mode: poly|bbox|mass"""
    pts = np.array(poly_xyxyxyxy, dtype=np.float32)
    if pts.ndim == 1 and pts.size == 8:
        pts = pts.reshape(-1, 2)
    if mode == 'bbox':
        x1, y1 = np.min(pts[:, 0]), np.min(pts[:, 1])
        x2, y2 = np.max(pts[:, 0]), np.max(pts[:, 1])
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0
    # centroid（多边形形心）
    if mode in ('poly', 'mass'):
        M = cv2.moments(pts.astype(np.float32))
        if abs(M['m00']) < 1e-6:
            # 退化则用 bbox
            return polygon_center(pts, mode='bbox')
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        return cx, cy
    # 默认
    return polygon_center(pts, mode='bbox')

# def depth_from_roi(depth_m, poly, percentiles=(25, 75)):
#     """在 OBB 多边形内部取一个中心窗口，进行百分位截断后取中位数"""
#     h, w = depth_m.shape[:2]
#     pts = np.array(poly, dtype=np.int32)
#     if pts.ndim == 1 and pts.size == 8:
#         pts = pts.reshape(-1, 2)
#     mask = np.zeros((h, w), dtype=np.uint8)
#     cv2.fillPoly(mask, [pts], 1)
#     ys, xs = np.where(mask > 0)
#     if len(xs) == 0:
#         return None
#     cx = int(xs.mean())
#     cy =  int(ys.mean())
#     win = max(3, int(0.15 * min(h, w)))
#     xlo, xhi = np.clip([cx - win, cx + win], 0, w - 1)
#     ylo, yhi = np.clip([cy - win, cy + win], 0, h - 1)
#     roi = depth_m[ylo:yhi + 1, xlo:xhi + 1]
#     valid = (roi > 0) & (roi < 10.0)
#     vals = roi[valid]
#     if vals.size == 0:
#         return None
#     p0, p1 = np.percentile(vals, [float(percentiles[0]), float(percentiles[1])])
#     clipped = vals[(vals >= p0) & (vals <= p1)]
#     if clipped.size == 0:
#         clipped = vals
#     return float(np.median(clipped))

def depth_from_roi(depth_m, poly):
    """
    传入深度图和多边形顶点，返回多边形外接矩形区域内的深度中位数。
    过滤无效深度（<=0或>10m）。
    """
    h, w = depth_m.shape[:2]
    pts = np.array(poly, dtype=np.int32).reshape(-1, 2)

    # 计算多边形的外接矩形 bbox
    x_coords = pts[:, 0]
    y_coords = pts[:, 1]
    x1, y1 = np.clip(int(np.min(x_coords)), 0, w - 1), np.clip(int(np.min(y_coords)), 0, h - 1)
    x2, y2 = np.clip(int(np.max(x_coords)), 0, w - 1), np.clip(int(np.max(y_coords)), 0, h - 1)

    # 取 bbox 区域深度
    roi = depth_m[y1:y2 + 1, x1:x2 + 1]

    # 过滤有效深度
    valid = (roi > 0) & (roi < 10.0)
    vals = roi[valid]

    if vals.size == 0:
        return None

    # 直接返回中位数
    return float(np.median(vals))

def get_fused_u_for_symmetric_cylinder(depth_m, poly, camera_matrix=None, weights=(0.3, 0.4, 0.3)):
    """
    专为对称柱状物体设计的 u 计算
    融合：外接框中心 + 深度点云质心 + 对称强制中心
    """
    h, w = depth_m.shape
    pts = np.array(poly, dtype=np.int32).reshape(-1, 2)

    # ===== 1. 外接矩形中心 (u_bbox) =====
    x_min, x_max = np.min(pts[:, 0]), np.max(pts[:, 0])
    y_min, y_max = np.min(pts[:, 1]), np.max(pts[:, 1])
    u_bbox = (x_min + x_max) / 2.0
    v_bbox = (y_min + y_max) / 2.0

    # ===== 2. 深度点云质心 (u_cloud) =====
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 1)
    ys, xs = np.where(mask > 0)
    valid_zs = depth_m[ys, xs]
    valid_mask = (valid_zs > 0.1) & (valid_zs < 5.0)

    if np.any(valid_mask):
        u_cloud = np.mean(xs[valid_mask])
        v_cloud = np.mean(ys[valid_mask])
    else:
        u_cloud, v_cloud = u_bbox, v_bbox  # 回退

    # ===== 3. 对称强制中心 (u_symm) =====
    # 柱体左右对称，强制使用左右边界的中点
    left_mask = xs[valid_mask] <= u_bbox
    right_mask = xs[valid_mask] > u_bbox
    if np.any(left_mask) and np.any(right_mask):
        u_left = np.mean(xs[valid_mask][left_mask])
        u_right = np.mean(xs[valid_mask][right_mask])
        u_symm = (u_left + u_right) / 2.0
    else:
        u_symm = u_bbox

    # ===== 融合 =====
    w1, w2, w3 = weights
    u_fused = w1 * u_bbox + w2 * u_cloud + w3 * u_symm
    v_fused = w1 * v_bbox + w2 * v_cloud + w2 * v_bbox  # v 不重要，简单处理

    return u_fused, v_fused


# ---------- Orbbec 初始化（尽量读取内参与深度尺度） ----------
# def init_orbbec_pipeline(mode='SW', enable_sync=True, width=DEPTH_WIDTH, height=DEPTH_HEIGHT, fps=DEPTH_FPS):
#     if not HAS_ORBBEC:
#         raise RuntimeError("pyorbbecsdk 未安装或不可用。")

#     pipeline = ob.Pipeline()
#     config = ob.Config()
#     info = {}

#     # 获取设备信息
#     try:
#         device = pipeline.get_device()
#         device_info = device.get_device_info()
#         info['device'] = {
#             'name': device_info.get_name(),
#             'serial_number': device_info.get_serial_number(),
#             'pid': device_info.get_pid(),
#             'firmware': device_info.get_firmware_version()
#         }
#     except Exception as e:
#         info['device'] = {'error': str(e)}

#     # stream profiles
#     try:
#         color_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
#         depth_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
#     except Exception as e:
#         color_profiles = None
#         depth_profiles = None
#         LOGGER.warning(f"get_stream_profile_list failed: {e}")

#     color_profile = None
#     depth_profile = None
#     try:
#         if color_profiles is not None:
#             color_profile = color_profiles.get_video_stream_profile(width, height, ob.OBFormat.RGB, fps)
#         if depth_profiles is not None:
#             depth_profile = depth_profiles.get_video_stream_profile(width, height, ob.OBFormat.Y16, fps)
#     except Exception as e:
#         LOGGER.warning(f"selecting video profile failed: {e}")

#     if color_profile is None and color_profiles is not None:
#         color_profile = color_profiles.get_default_video_stream_profile()
#     if depth_profile is None and depth_profiles is not None:
#         depth_profile = depth_profiles.get_default_video_stream_profile()

#     if color_profile is None or depth_profile is None:
#         LOGGER.error("未能找到 color 或 depth profile，初始化失败")
#         raise RuntimeError("No color/depth profile available")

#     config.enable_stream(color_profile)
#     config.enable_stream(depth_profile)

#     # 对齐模式
#     mode_upper = (mode or 'SW').upper()
#     try:
#         if mode_upper == 'HW':
#             config.set_align_mode(ob.OBAlignMode.HW_MODE)
#             LOGGER.info("启用硬件 D2C 对齐")
#         elif mode_upper == 'SW':
#             config.set_align_mode(ob.OBAlignMode.SW_MODE)
#             LOGGER.info("启用软件对齐模式")
#         elif mode_upper == 'NONE':
#             config.set_align_mode(ob.OBAlignMode.DISABLE)
#             LOGGER.info("禁用对齐")
#         else:
#             config.set_align_mode(ob.OBAlignMode.DISABLE)
#             LOGGER.info("默认禁用对齐")
#     except Exception as e:
#         LOGGER.warning(f"无法设置对齐模式: {e}")
#         config.set_align_mode(ob.OBAlignMode.DISABLE)

#     if enable_sync:
#         try:
#             pipeline.enable_frame_sync()
#         except Exception as e:
#             LOGGER.warning(f"frame sync 失败: {e}")

#     # 启动
#     pipeline.start(config)

#     # 读取彩色相机内参（若 SDK 提供）
#     camera_matrix = FALLBACK_CAMERA_MATRIX.copy()
#     dist_coeffs = FALLBACK_DIST_COEFFS.copy()
#     try:
#         cprof = color_profile.as_video_stream_profile()
#         intr = cprof.get_intrinsics()
#         camera_matrix = np.array([[intr.fx, 0, intr.cx],
#                                   [0, intr.fy, intr.cy],
#                                   [0, 0, 1]], dtype=np.float32)
#         dist_coeffs = np.array([intr.k1, intr.k2, intr.p1, intr.p2, intr.k3], dtype=np.float32)
#         LOGGER.info(f"相机内参(Orbbec): fx={intr.fx:.2f}, fy={intr.fy:.2f}, cx={intr.cx:.2f}, cy={intr.cy:.2f}")
#     except Exception as e:
#         LOGGER.warning(f"读取 Orbbec 内参失败，改用回退内参: {e}")

#     # 深度scale（若 SDK 提供）
#     depth_scale = DEFAULT_DEPTH_SCALE
#     try:
#         # 有的 SDK 在 depth frame 里提供 get_depth_scale；这里先记录，后面每帧再兜底
#         depth_scale = getattr(depth_profile, "get_depth_scale", lambda: DEFAULT_DEPTH_SCALE)()
#         if not isinstance(depth_scale, (int, float)) or depth_scale <= 0:
#             depth_scale = DEFAULT_DEPTH_SCALE
#    except Exception:
#        depth_scale = DEFAULT_DEPTH_SCALE

#    return pipeline, camera_matrix, dist_coeffs, float(depth_scale), mode_upper

def init_orbbec_pipeline(mode='SW', enable_sync=True, width=DEPTH_WIDTH, height=DEPTH_HEIGHT, fps=DEPTH_FPS):
    if not HAS_ORBBEC:
        raise RuntimeError("pyorbbecsdk 未安装或不可用。")

    pipeline = ob.Pipeline()
    config = ob.Config()
    info = {}

    # 获取设备信息
    try:
        device = pipeline.get_device()
        device_info = device.get_device_info()
        info['device'] = {
            'name': device_info.get_name(),
            'serial_number': device_info.get_serial_number(),
            'pid': device_info.get_pid(),
            'firmware': device_info.get_firmware_version()
        }
    except Exception as e:
        info['device'] = {'error': str(e)}

    # stream profiles
    try:
        color_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        depth_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
    except Exception as e:
        color_profiles = None
        depth_profiles = None
        LOGGER.warning(f"get_stream_profile_list failed: {e}")

    color_profile = None
    depth_profile = None
    try:
        if color_profiles is not None:
            color_profile = color_profiles.get_video_stream_profile(width, height, ob.OBFormat.RGB, fps)
        if depth_profiles is not None:
            depth_profile = depth_profiles.get_video_stream_profile(width, height, ob.OBFormat.Y16, fps)
    except Exception as e:
        LOGGER.warning(f"selecting video profile failed: {e}")

    if color_profile is None and color_profiles is not None:
        color_profile = color_profiles.get_default_video_stream_profile()
    if depth_profile is None and depth_profiles is not None:
        depth_profile = depth_profiles.get_default_video_stream_profile()

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
            LOGGER.info("启用硬件 D2C 对齐")
        elif mode_upper == 'SW':
            config.set_align_mode(ob.OBAlignMode.SW_MODE)
            LOGGER.info("启用软件对齐模式")
        elif mode_upper == 'NONE':
            config.set_align_mode(ob.OBAlignMode.DISABLE)
            LOGGER.info("禁用对齐")
        else:
            config.set_align_mode(ob.OBAlignMode.DISABLE)
            LOGGER.info("默认禁用对齐")
    except Exception as e:
        LOGGER.warning(f"无法设置对齐模式: {e}")
        config.set_align_mode(ob.OBAlignMode.DISABLE)

    if enable_sync:
        try:
            pipeline.enable_frame_sync()
        except Exception as e:
            LOGGER.warning(f"frame sync 失败: {e}")

    # 启动
    pipeline.start(config)

    # ===== 使用 get_camera_intrinsics 获取相机内参 =====
    intrinsics_obj, camera_matrix, dist_coeffs = get_camera_intrinsics(pipeline)

    if intrinsics_obj is None or camera_matrix is None:
        # 如果从设备获取失败，尝试从 profile 获取
        LOGGER.warning("get_camera_intrinsics 获取失败，尝试从 profile 获取内参")
        camera_matrix = FALLBACK_CAMERA_MATRIX.copy()
        dist_coeffs = FALLBACK_DIST_COEFFS.copy()

        try:
            # 尝试从 color_profile 获取（作为备选方案）
            cprof = color_profile.as_video_stream_profile()
            intr = cprof.get_intrinsics()
            camera_matrix = np.array([[intr.fx, 0, intr.cx],
                                      [0, intr.fy, intr.cy],
                                      [0, 0, 1]], dtype=np.float32)
            dist_coeffs = np.array([intr.k1, intr.k2, intr.p1, intr.p2, intr.k3], dtype=np.float32)
            LOGGER.info(
                f"从 color profile 获取内参成功: fx={intr.fx:.2f}, fy={intr.fy:.2f}, cx={intr.cx:.2f}, cy={intr.cy:.2f}")
        except Exception as e:
            LOGGER.warning(f"从 profile 读取内参也失败，使用默认回退内参: {e}")
    else:
        LOGGER.info("成功从设备获取相机内参")
        # 可选：打印获取到的内参信息
        if intrinsics_obj:
            LOGGER.info(f"内参详情: fx={intrinsics_obj.fx:.2f}, fy={intrinsics_obj.fy:.2f}, "
                        f"cx={intrinsics_obj.cx:.2f}, cy={intrinsics_obj.cy:.2f}, "
                        f"分辨率={intrinsics_obj.width}x{intrinsics_obj.height}")

    # 深度scale（若 SDK 提供）
    depth_scale = DEFAULT_DEPTH_SCALE
    try:
        # 有的 SDK 在 depth frame 里提供 get_depth_scale；这里先记录，后面每帧再兜底
        depth_scale = getattr(depth_profile, "get_depth_scale", lambda: DEFAULT_DEPTH_SCALE)()
        if not isinstance(depth_scale, (int, float)) or depth_scale <= 0:
            depth_scale = DEFAULT_DEPTH_SCALE
    except Exception:
        depth_scale = DEFAULT_DEPTH_SCALE

    return pipeline, camera_matrix, dist_coeffs, float(depth_scale), mode_upper


# ---------- 主运行函数 ----------
@torch.no_grad()
def run(weights=ROOT / 'runs/best.pt',
        source='1',
        imgsz=(640, 640),
        conf_thres=0.85,
        iou_thres=0.45,
        max_det=1000,
        device='',
        view_img=False,
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
        save_csv=False):

    source = str(source)

    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)

    half &= (pt or jit or engine) and device.type != 'cpu'
    if pt or jit:
        model.model.half() if half else model.model.float()

    use_orbbec = (source.strip() == '1') and HAS_ORBBEC

    # CSV
    csv_file = None
    csv_writer = None

    # 如果要使用 Orbbec
    pipeline = None
    camera_matrix = FALLBACK_CAMERA_MATRIX.copy()
    dist_coeffs = FALLBACK_DIST_COEFFS.copy()
    depth_scale = DEFAULT_DEPTH_SCALE

    if use_orbbec:
        LOGGER.info("准备使用 Orbbec 摄像头")
        try:
            pipeline, camera_matrix, dist_coeffs, depth_scale, used_mode = init_orbbec_pipeline(
                mode=mode, enable_sync=enable_sync, width=DEPTH_WIDTH, height=DEPTH_HEIGHT, fps=DEPTH_FPS
            )
            LOGGER.info(f"Orbbec 初始化成功: mode={used_mode}, depth_scale={depth_scale}")
            use_orbbec = True
        except Exception as e:
            LOGGER.error(f"Orbbec 相机初始化失败，回退到标准模式: {e}")
            use_orbbec = False

    # 非 Orbbec 分支：使用标准加载器
    if not use_orbbec:
        LOGGER.info(f"使用标准数据加载器处理源: {source}")
        cudnn.benchmark = True
        is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
        is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
        webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)

        if webcam:
            view_img = check_imshow()
            dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
            bs = len(dataset)
        else:
            dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)
            bs = 1

        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
        (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)
        vid_path, vid_writer = [None] * bs, [None] * bs

    # Orbbec 分支：实时检测
    if use_orbbec:
        LOGGER.info("开始 Orbbec 摄像头实时检测")
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
        save_dir.mkdir(parents=True, exist_ok=True)
        if save_csv:
            csv_path = os.path.join(str(save_dir), 'results.csv')
            csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['frame', 'class', 'conf', 'u', 'v', 'depth_m', 'cam_x', 'cam_y', 'cam_z', 'world_x', 'world_y', 'world_z'])

        video_path = os.path.join(str(save_dir), 'orbbec_detected_video.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        try:
            video_writer = cv2.VideoWriter(video_path, fourcc, DEPTH_FPS, (DEPTH_WIDTH, DEPTH_HEIGHT))
        except Exception as e:
            LOGGER.warning(f"无法创建视频写入器: {e}")
            video_writer = None

        processed_frames_count = 0
        dt = [0.0, 0.0, 0.0]

        try:
            while True:
                frameset = pipeline.wait_for_frames(1000)
                if frameset is None:
                    LOGGER.warning("未能获取到 Orbbec 帧集")
                    time.sleep(0.01)
                    continue

                color_frame = frameset.get_color_frame()
                depth_frame = frameset.get_depth_frame()

                if (color_frame is None) or (depth_frame is None):
                    LOGGER.warning("未能获取到彩色或深度帧")
                    continue

                # --- color image ---
                t1 = time_sync()
                im0 = frame_to_bgr_image(color_frame)
                if im0 is None:
                    LOGGER.warning("彩色帧转换失败")
                    continue
                try:
                    im0_rgb = cv2.cvtColor(im0, cv2.COLOR_BGR2RGB)
                except Exception:
                    im0_rgb = im0

                # --- depth ---
                try:
                    d_h = depth_frame.get_height()
                    d_w = depth_frame.get_width()
                    depth_format = depth_frame.get_format()
                    data_bytes = depth_frame.get_data()

                    # Orbbec Y16 → uint16 → m
                    if depth_format == ob.OBFormat.Y16:
                        depth_raw = np.frombuffer(data_bytes, dtype=np.uint16)
                        if depth_raw.size != d_h * d_w:
                            LOGGER.warning("Y16 数据大小不匹配")
                            depth_meters = None
                        else:
                            depth_raw = depth_raw.reshape((d_h, d_w))
                            depth_meters = depth_raw.astype(np.float32) * float(depth_scale)
                    else:
                        LOGGER.warning(f"未知深度格式: {depth_format}")
                        depth_meters = None

                    if depth_meters is not None and depth_meters.shape != (DEPTH_HEIGHT, DEPTH_WIDTH):
                        depth_meters = cv2.resize(depth_meters, (DEPTH_WIDTH, DEPTH_HEIGHT), interpolation=cv2.INTER_NEAREST)

                except Exception as e:
                    LOGGER.warning(f"深度解码失败: {e}")
                    depth_meters = None

                # --- image preprocessing for model ---
                letterboxed_img = letterbox(im0_rgb, imgsz, stride=stride, auto=pt)[0]
                im_tensor = torch.from_numpy(letterboxed_img).permute(2, 0, 1).unsqueeze(0).to(device)
                im_tensor = im_tensor.half() if half else im_tensor.float()
                im_tensor /= 255.0
                t2 = time_sync()
                dt[0] += (t2 - t1)

                # model inference
                pred = model(im_tensor, augment=augment, visualize=visualize)
                t3 = time_sync()
                dt[1] += (t3 - t2)

                pred = non_max_suppression_obb(pred, conf_thres=conf_thres, iou_thres=iou_thres,
                                               classes=classes, agnostic=agnostic_nms,
                                               multi_label=True, max_det=max_det)
                t4 = time_sync()
                dt[2] += (t4 - t3)

                # 处理检测结果
                im0_annotated = im0_rgb.copy()
                annotator = Annotator(im0_annotated, line_width=line_thickness, example=str(names))

                if len(pred) and len(pred[0]):
                    det = pred[0]
                    pred_poly = rbox2poly(det[:, :5])
                    pred_poly = scale_polys(im_tensor.shape[2:], pred_poly, im0_rgb.shape)
                    det_numpy = torch.cat((pred_poly, det[:, -2:]), dim=1).cpu().numpy()

                    for c in np.unique(det_numpy[:, -1]):
                        n = (det_numpy[:, -1] == c).sum()
                        LOGGER.info(f"Detected: {n} {names[int(c)]}{'s' * (n > 1)}")

                    for *poly, conf, cls in reversed(det_numpy):
                        poly_list = list(poly)
                        label_text = f'{names[int(cls)]} {conf:.2f}'
                        cam_coords = None
                        world_coords = None
                        depth_val_m = None
                        u = v = None

                        if depth_meters is not None:
                            # 多边形中心（支持 bbox/centroid）
                            pts = np.array(poly_list, dtype=np.float32).reshape(-1, 2)

                            # 只采用目标检测框
                            # u, v = polygon_center(pts, mode=center_mode)


                            u, v = get_fused_u_for_symmetric_cylinder(depth_meters, poly_list, camera_matrix)


                            # ROI 深度（百分位截断）
                            depth_val_m = depth_from_roi(depth_meters, poly_list)

                            if depth_val_m is not None and depth_val_m > 0:
                                cam_coords = pixel_to_camera((u, v), depth_val_m, camera_matrix, dist_coeffs, pp_delta)
                                if cam_coords is not None:
                                    world_coords = camera_to_world(cam_coords)
                                    label_text += f' | Z:{depth_val_m:.2f}m'
                                    label_text += f' | Cam:({cam_coords[0]:.2f},{cam_coords[1]:.2f},{cam_coords[2]:.2f})'
                                    if world_coords is not None:
                                        label_text += f' | W:({world_coords[0]:.2f},{world_coords[1]:.2f},{world_coords[2]:.2f})'
                                    im0_annotated = draw_axes_at_point(im0_annotated, cam_coords, camera_matrix, dist_coeffs, axis_length=0.05)

                                    # 打印相机坐标和世界坐标
                                    print("\n---目标检测详情---")
                                    print(f"类别: {names[int(cls)]}")
                                    print(f"置信度: {conf:.2f}")
                                    print(f"边界框坐标: ({poly_list[0]:.0f}, {poly_list[1]:.0f}, {poly_list[2]:.0f}, {poly_list[3]:.0f})")
                                    print(f"相机坐标系: X={cam_coords[0]:.3f}m, Y={cam_coords[1]:.3f}m, Z={cam_coords[2]:.3f}m")
                                    if world_coords is not None:
                                        print(f"世界坐标系: X={world_coords[0]:.3f}m, Y={world_coords[1]:.3f}m, Z={world_coords[2]:.3f}m")
                                    else:
                                        print("世界坐标系: 无法转换")
                            else:
                                label_text += ' | Depth:N/A'
                        else:
                            label_text += ' | Depth:N/A'

                        annotator.poly_label(poly_list, label_text, color=colors(int(cls), True))

                        if csv_writer is not None:
                            row = [processed_frames_count, int(cls), float(conf),
                                   None if u is None else float(u),
                                   None if v is None else float(v),
                                   None if depth_val_m is None else float(depth_val_m)]
                            if cam_coords is not None:
                                row += [float(cam_coords[0]), float(cam_coords[1]), float(cam_coords[2])]
                            else:
                                row += [None, None, None]
                            if world_coords is not None:
                                row += [float(world_coords[0]), float(world_coords[1]), float(world_coords[2])]
                            else:
                                row += [None, None, None]
                            csv_writer.writerow(row)

                im0_annotated = annotator.result()

                if view_img:
                    try:
                        cv2.imshow("YOLOv5-OBB Detection", cv2.cvtColor(im0_annotated, cv2.COLOR_RGB2BGR))
                    except Exception:
                        cv2.imshow("YOLOv5-OBB Detection", im0_annotated)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt

                if video_writer is not None and not nosave:
                    if video_writer.isOpened():
                        try:
                            video_writer.write(cv2.cvtColor(im0_annotated, cv2.COLOR_RGB2BGR))
                        except Exception:
                            video_writer.write(im0_annotated)

                processed_frames_count += 1
                LOGGER.info(f"Frame {processed_frames_count}: pre:{(t2 - t1):.3f}s, infer:{(t3 - t2):.3f}s, nms:{(t4 - t3):.3f}s")

        except KeyboardInterrupt:
            LOGGER.info("用户中断。")
        except Exception as e:
            LOGGER.error(f"Orbbec 摄像头处理循环出错: {e}")
        finally:
            LOGGER.info("停止 Orbbec 摄像头...")
            try:
                pipeline.stop()
            except Exception:
                pass
            if video_writer is not None:
                try:
                    video_writer.release()
                    LOGGER.info(f"检测视频已保存到 {video_path}")
                except Exception:
                    pass
            if csv_file is not None:
                csv_file.close()
            cv2.destroyAllWindows()

    else:
        # 非 Orbbec 标准流程（与原文件相同）
        LOGGER.info("使用标准 YOLOv5 检测流程...")
        dt, seen = [0.0, 0.0, 0.0], 0
        model.warmup(imgsz=(1, 3, *imgsz), half=half)

        for path, im, im0s, vid_cap, s in dataset:
            t1 = time_sync()
            im = torch.from_numpy(im).to(device)
            im = im.half() if half else im.float()
            im /= 255
            if len(im.shape) == 3:
                im = im[None]
            t2 = time_sync()
            dt[0] += t2 - t1

            visualize_dir = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(im, augment=augment, visualize=visualize_dir)
            t3 = time_sync()
            dt[1] += t3 - t2

            pred = non_max_suppression_obb(pred, conf_thres, iou_thres, classes, agnostic_nms, multi_label=True,
                                           max_det=max_det)
            dt[2] += time_sync() - t3

            save_img_standard = not nosave and not (isinstance(path, list) and all(p.endswith('.txt') for p in path))

            for i, det in enumerate(pred):
                seen += 1
                if isinstance(path, list):
                    p_current, im0_current, frame = Path(path[i]), im0s[i].copy(), dataset.count
                    s_current_log = f'{i}: '
                else:
                    p_current, im0_current, frame = Path(path), im0s.copy(), getattr(dataset, 'frame', 0)
                    s_current_log = ''

                save_path_str = str(save_dir / p_current.name)
                txt_path = str(save_dir / 'labels' / p_current.stem) + (f'_{frame}' if dataset.mode != 'image' else '')

                s_log_line = s_current_log + '%gx%g ' % im.shape[2:]

                imc = im0_current.copy() if save_crop else im0_current

                annotator = Annotator(im0_current, line_width=line_thickness, example=str(names))

                if len(det):
                    pred_poly = rbox2poly(det[:, :5])
                    pred_poly = scale_polys(im.shape[2:], pred_poly, im0_current.shape)
                    det_numpy = torch.cat((pred_poly, det[:, -2:]), dim=1).cpu().numpy()

                    for c in np.unique(det_numpy[:, -1]):
                        n = (det_numpy[:, -1] == c).sum()
                        s_log_line += f"{n} {names[int(c)]}{'s' * (n > 1)}, "

                    for *poly, conf, cls in reversed(det_numpy):
                        poly_list = list(poly)
                        if save_txt:
                            line = (int(cls), *poly_list, float(conf)) if save_conf else (int(cls), *poly_list)
                            with open(txt_path + '.txt', 'a') as f:
                                f.write(('%g ' * len(line)).rstrip() % line + '\n')

                        if save_img_standard or save_crop or view_img:
                            c = int(cls)
                            label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                            annotator.poly_label(poly_list, label, color=colors(c, True))

                LOGGER.info(f'{s_log_line}Done. (Inference: {t3 - t2:.3f}s)')

                im0_display = annotator.result()
                if view_img:
                    cv2.imshow(str(p_current), im0_display)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                if save_img_standard:
                    if dataset.mode == 'image':
                        cv2.imwrite(save_path_str, im0_display)
                    else:
                        if vid_path[i] != save_path_str:
                            vid_path[i] = save_path_str
                            if isinstance(vid_writer[i], cv2.VideoWriter):
                                vid_writer[i].release()
                            if vid_cap:
                                fps = vid_cap.get(cv2.CAP_PROP_FPS)
                                w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                                h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            else:
                                if not save_path_str.endswith('.mp4'):
                                    save_path_str += '.mp4'
                                fps, w, h = 30, im0_display.shape[1], im0_display.shape[0]
                            vid_writer[i] = cv2.VideoWriter(save_path_str, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                        vid_writer[i].write(im0_display)

        t_avg = tuple(x / seen * 1E3 for x in dt)
        LOGGER.info(
            f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t_avg)
        if save_txt or save_img_standard:
            s_info = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
            LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s_info}")
        if update:
            strip_optimizer(weights)
        cv2.destroyAllWindows()

# ---------- argparse ----------
def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=str(ROOT / 'runs/best.pt'),
                        help='model path(s)')
    parser.add_argument('--source', type=str, default='1',
                        help='file/dir/URL/glob, 0 for webcam, "1" for Orbbec camera')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640],
                        help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.85, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', default=True, help='show results')
    parser.add_argument('--save-txt', action='store_true', default=True, help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', default=False, help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=str(ROOT / 'runs/detect'), help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--mode', type=str, default='SW', help='Orbbec align mode: HW / SW / NONE')
    parser.add_argument('--enable_sync', type=bool, default=True, help='enable frame sync between color and depth')

    # 新增功能参数
    parser.add_argument('--center-mode', type=str, default='poly',
                        choices=['poly', 'bbox', 'mass'],
                        help='像素中心计算方式：poly(多边形形心)/bbox(外接框中心)/mass(同poly)')
    parser.add_argument('--depth-percentiles', nargs=2, type=float, default=[25, 75],
                        help='ROI 深度百分位截断，两个数字，如 25 75')
    parser.add_argument('--pp-delta', nargs=2, type=float, default=[0.0, 0.0],
                        help='主点微调 du dv（像素）')
    parser.add_argument('--save-csv', action='store_true', default= True,help='保存 3D 结果到 CSV')

    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1
    print_args(FILE.stem, opt)
    return opt

def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))

if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
