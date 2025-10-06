# ******************************************************************************
#  相机坐标系X估计（圆柱中心），支持Open3D可视化与ΔX基线
#  已修复：彩色相机内参、点云单位（米）、PointCloudFilter 比例、xyz_map 兜底生成、像素投影等
#  @file YOLOv5 + Orbbec Depth 支持版 + 基于官方 AlignFilter+PointCloudFilter 的点云ROI拟合
#  @author 2307014121 杨超
#  @attention 基于生产者-消费者方式处理多个相机带来的线程管理问题
#  @data 2025-9-12
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
from collections import deque
from queue import Queue
from collections import deque
import socket    # 用于UDP网络通信传输文本文件
import threading
import yaml

# 加载YAML参数
def load_extrinsics_yaml(path='extrinsics.yaml'):
    if not os.path.exists(path):
        LOGGER.warning(f"未找到外参配置: {path}，将使用默认 TRANSFORM_MATRIX")
        return {}
    cfg = yaml.safe_load(open(path, 'r', encoding='utf-8'))
    out = {}
    for it in cfg.get('cameras', []):
        serial = str(it.get('serial', '')).strip()
        T = np.array(it['Tcw'], dtype=np.float32)
        units = str(it.get('units', 'm')).lower()
        if units == 'mm':
            T[:3, 3] /= 1000.0
        out[serial] = T
    return out

def camera_to_world_ex(camera_coords, Tcw):
    if camera_coords is None or Tcw is None:
        return None
    p = np.append(camera_coords, 1.0)
    w = Tcw @ p
    return w[:3] / (w[3] if w.shape[0] > 3 else 1.0)


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

ESC_KEY = 27
MAX_DEVICES = 2   # 最大允许启动两个相机
MAX_QUEUE_SIZE = 5

FALLBACK_CAMERA_MATRIX = np.array([
    [611.572, 0.0, 643.59109755],
    [0.0, 611.817, 341.29027134],
    [0.0, 0.0, 1.0]
], dtype=np.float32)
FALLBACK_DIST_COEFFS = np.array([-0.12393594, 0.0467302, 0.00263421, -0.00696342, -0.01053551], dtype=np.float32)

# 世界变换矩阵（示例），平移单位以米为准
# TRANSFORM_MATRIX = np.array([
#     [9.30724545e-01, -2.57750005e-01, -2.59454727e-01, 68],
#     [6.86573375e-02,  8.19962375e-01, -5.68285028e-01, 1200],
#     [3.59218583e-01,  5.11103353e-01,  7.80855538e-01, 1520],
#     [0.0, 0.0, 0.0, 1.0]
# ], dtype=np.float32)
# TRANSFORM_MATRIX[:3, 3] /= 1000.0  # 将 mm -> m

# TRANSFORM_MATRIX = np.array([
#     [0.03684, -0.90316564, -0.42770836, 68],
#     [-0.98326815, -0.10916939, -0.14582797, 1200],
#     [0.17839947,  -0.41517932, -0.8920761 , 1020],
#     [0.0, 0.0, 0.0, 1.0]
# ], dtype=np.float32)
# TRANSFORM_MATRIX[:3, 3] /= 1000.0  # 将 mm -> m


TRANSFORM_MATRIX = np.array([
    [0.0, 1.0, 0.0, -100],
    [1.0, 0.0, 0.0, -31.55],
    [0.0, 0.0, 1.0, 1030],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float32)
TRANSFORM_MATRIX[:3, 3] /= 1000.0  # 将 mm -> m


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
        p0 = tuple(imgpts[0].ravel());
        pX = tuple(imgpts[1].ravel());
        pY = tuple(imgpts[2].ravel());
        pZ = tuple(imgpts[3].ravel())
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
        cx = M['m10'] / M['m00'];
        cy = M['m01'] / M['m00']
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
        u_cloud = np.mean(xs[valid_mask]);
        v_cloud = np.mean(ys[valid_mask])
    else:
        u_cloud, v_cloud = u_bbox, v_bbox
    left_mask = xs[valid_mask] <= u_bbox;
    right_mask = xs[valid_mask] > u_bbox
    if np.any(left_mask) and np.any(right_mask):
        u_left = np.mean(xs[valid_mask][left_mask]);
        u_right = np.mean(xs[valid_mask][right_mask])
        u_symm = (u_left + u_right) / 2.0
    else:
        u_symm = u_bbox
    w1, w2, w3 = weights
    u_fused = w1 * u_bbox + w2 * u_cloud + w3 * u_symm
    # 这里保持原逻辑（v 基本无需对称项），避免行为变化
    v_fused = w1 * v_bbox + w2 * v_cloud + w2 * v_bbox
    return u_fused, v_fused


# ================== 点云几何（基于ROI点集） ==================
def polygon_to_mask(poly, shape_hw, erode_px=0):
    H, W = shape_hw
    pts = np.array(poly, dtype=np.int32).reshape(-1, 2)
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 1)
    if erode_px and erode_px > 0:
        k = int(erode_px);
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
    u = np.cross(axis, ref);
    u /= (np.linalg.norm(u) + 1e-12)
    v = np.cross(axis, u);
    v /= (np.linalg.norm(v) + 1e-12)
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
    c = c0.astype(np.float32);
    lr = 0.1
    for _ in range(80):
        diff = c - pts;
        di = np.linalg.norm(diff, axis=1) + 1e-6
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
            c = x[:2];
            r = x[2]
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
        med = np.median(pts, axis=0);
        mad = np.median(np.abs(pts - med), axis=0) + 1e-6
        pts = pts[np.all(np.abs(pts - med) <= 4.0 * mad, axis=1)]
        if pts.shape[0] < 50: return None, None, 0
    # 轴估计 + 切片
    axis = pca_axis(pts)
    slice_pts, base_point, _ = robust_slice_by_median(pts, axis, init_thickness=slice_thickness_m,
                                                      min_points=min_points)
    if slice_pts.shape[0] < 20:
        return pts.mean(axis=0).astype(np.float32), axis.astype(np.float32), int(pts.shape[0])
    u, v = orthonormal_basis_from_axis(axis)
    vecs = slice_pts - base_point
    uv = np.stack([vecs @ u, vecs @ v], axis=1)
    # 截面圆拟合
    if force_circle_fit and (known_radius_m is not None) and (known_radius_m > 0):
        c2d = fit_circle_center_known_radius(uv, known_radius_m, c0=np.median(uv, axis=0), robust=True)
    else:
        c2d = np.mean(uv, axis=0)
    center3d = base_point + c2d[0] * u + c2d[1] * v
    return center3d.astype(np.float32), axis.astype(np.float32), int(slice_pts.shape[0])


# ===== Open3D 可视化：ROI->滤波->球+轴 =====
def _make_o3d_axis_lines_mm(center_m, axis, half_len_mm=50.0, color=(1.0, 0.0, 0.0)):
    c = np.asarray(center_m, dtype=np.float32)
    a = np.asarray(axis, dtype=np.float32);
    a /= (np.linalg.norm(a) + 1e-12)
    L = float(half_len_mm) / 1000.0
    p0 = c - a * L;
    p1 = c + a * L
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
        sphere.translate(center3d);
        sphere.paint_uniform_color([1.0, 0.0, 0.0])
        geoms.append(sphere)
    if center3d is not None and axis3d is not None:
        line = _make_o3d_axis_lines_mm(center3d, axis3d, half_len_mm=axis_mm, color=(1.0, 0.0, 0.0))
        geoms.append(line)
    o3d.visualization.draw_geometries(geoms, window_name="ROI + center + axis (m)")


# ---------- Orbbec 初始化 ----------
def init_orbbec_pipeline(mode='SW', enable_sync=True, width=DEPTH_WIDTH, height=DEPTH_HEIGHT, fps=DEPTH_FPS):
    if not HAS_ORBBEC:
        raise RuntimeError("pyorbbecsdk 未安装或不可用。")

    pipeline = ob.Pipeline()
    config = ob.Config()

    # 选择 profile（深度 Y16；颜色优先 RGB）
    color_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
    depth_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)

    color_profile = None
    depth_profile = None
    if depth_profiles is not None:
        try:
            depth_profile = depth_profiles.get_video_stream_profile(DEPTH_WIDTH, DEPTH_HEIGHT, ob.OBFormat.Y16,
                                                                    DEPTH_FPS)
        except Exception:
            depth_profile = depth_profiles.get_default_video_stream_profile()

    if color_profiles is not None:
        try:
            color_profile = color_profiles.get_video_stream_profile(DEPTH_WIDTH, DEPTH_HEIGHT, ob.OBFormat.RGB,
                                                                    DEPTH_FPS)
        except Exception:
            try:
                color_profile = color_profiles.get_video_stream_profile(1280, 720, ob.OBFormat.RGB, DEPTH_FPS)
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
            LOGGER.info("启用硬件 D2C 对齐")
        elif mode_upper == 'SW':
            config.set_align_mode(ob.OBAlignMode.SW_MODE)
            LOGGER.info("启用软件 D2C 对齐")
        elif mode_upper == 'NONE':
            config.set_align_mode(ob.OBAlignMode.DISABLE)
            LOGGER.info("禁用对齐")
        else:
            config.set_align_mode(ob.OBAlignMode.DISABLE)
    except Exception as e:
        LOGGER.warning(f"无法设置对齐模式: {e}")
        config.set_align_mode(ob.OBAlignMode.DISABLE)

    if enable_sync:
        try:
            pipeline.enable_frame_sync()
        except Exception as e:
            LOGGER.warning(f"frame sync 失败: {e}")

    pipeline.start(config)

    intrinsics_obj, camera_matrix, dist_coeffs = get_camera_intrinsics(pipeline)
    if intrinsics_obj is None or camera_matrix is None:
        LOGGER.warning("get_camera_intrinsics 获取失败，使用回退内参")
        camera_matrix = FALLBACK_CAMERA_MATRIX.copy()
        dist_coeffs = FALLBACK_DIST_COEFFS.copy()
    else:
        LOGGER.info("成功从设备获取相机内参（深度相机）- 将在首帧切换为彩色相机内参")

    depth_scale = DEFAULT_DEPTH_SCALE
    try:
        depth_profile = pipeline.get_stream_profile_list(
            ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
        depth_scale = getattr(depth_profile, "get_depth_scale", lambda: DEFAULT_DEPTH_SCALE)()
        if not isinstance(depth_scale, (int, float)) or depth_scale <= 0:
            depth_scale = DEFAULT_DEPTH_SCALE
    except Exception:
        depth_scale = DEFAULT_DEPTH_SCALE

    return pipeline, camera_matrix, dist_coeffs, float(depth_scale), mode_upper

def q_get_latest(q: Queue):
    """取队列里最新的一帧，丢弃旧帧，避免积压"""
    last = None
    while not q.empty():
        try:
            last = q.get_nowait()
        except Exception:
            break
    return last

def which_zone(u, v, W, H, rows=3, cols=3):
    """把像素(u,v)映射成第几区：按 rows×cols 栅格划分，返回(第N区, row, col)"""
    if u is None or v is None:
        return None, None, None
    col = int(np.clip(u / max(W, 1e-6) * cols, 0, cols - 1))
    row = int(np.clip(v / max(H, 1e-6) * rows, 0, rows - 1))
    zone_id = row * cols + col + 1
    return zone_id, row, col

def on_new_frame_callback_multicam(frames, index, align_filter, color_queues, depth_queues):
    """回调：对齐并推入每个相机的彩色/深度队列"""
    try:
        frames = align_filter.process(frames).as_frame_set()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is not None:
            if color_queues[index].qsize() >= MAX_QUEUE_SIZE:
                color_queues[index].get()
            color_queues[index].put(color_frame)
        if depth_frame is not None:
            if depth_queues[index].qsize() >= MAX_QUEUE_SIZE:
                depth_queues[index].get()
            depth_queues[index].put(depth_frame)
    except Exception as e:
        LOGGER.warning(f"[Cam {index}] frame callback error: {e}")



# UDP发送文件的封装函数
def udp_send_file_async(file_path, ip, port, chunk_size=1024, timeout=2.0):
    threading.Thread(
        target=udp_send_file,
        args=(file_path, ip, port),
        kwargs=dict(chunk_size=chunk_size, timeout=timeout),
        daemon=True
    ).start()


def udp_send_file(file_path, ip, port, chunk_size=1024, timeout=2.0, max_retries=20):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)

    file_size = os.path.getsize(file_path)
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


# 两个相机同时使用
@torch.no_grad()
def run_multicam(weights=ROOT / 'runs/best.pt',
                 imgsz=(640, 640),
                 conf_thres=0.60,
                 iou_thres=0.45,
                 max_det=1000,
                 device='',
                 view_img=True,
                 augment=False,
                 visualize=False,
                 line_thickness=3,
                 hide_labels=False,
                 hide_conf=False,
                 half=False,
                 dnn=False,
                 mode='SW',
                 enable_sync=True,
                 zones_rows=3,
                 zones_cols=3,
                 # 你已有的几何/3D参数按需透传
                 pc_gate_z_max_mm=1800,
                 pc_gate_y_min_mm=80,
                 pc_gate_y_max_mm=400,
                 cyl_radius_m=0.06,
                 pc_voxel_size=0.005,
                 pc_nb_neighbors=30,
                 pc_std_ratio=1.5,
                 pc_slice_thickness_m=0.02,
                 pc_min_points=200,
                 pc_max_points=60000,
                 pc_use_circle_fit=True,
                 # 输出与UDP配置
                 project=ROOT / 'runs/detect',
                 name='exp',
                 exist_ok=False,
                 udp_ip='192.168.171.136',
                 udp_port=8080,
                 udp_period=10.0,
                 udp_chunk=1024,
                 udp_timeout=2.0,
                 target_name='miehuoqi',
                 **kwargs):
    if not HAS_ORBBEC:
        raise RuntimeError("pyorbbecsdk 未安装或不可用。")

    # 1) 模型
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)
    half &= (pt or jit or engine) and device.type != 'cpu'
    if pt or jit:
        model.model.half() if half else model.model.float()

    # 2) 相机枚举与启动
    ctx = ob.Context()
    dev_list = ctx.query_devices()
    cam_count = min(dev_list.get_count(), MAX_DEVICES)
    if cam_count == 0:
        LOGGER.error("No Orbbec device connected.")
        return

    cams = []
    color_queues = [Queue() for _ in range(cam_count)]
    depth_queues = [Queue() for _ in range(cam_count)]

    for i in range(cam_count):
        dev = dev_list.get_device_by_index(i)
        info = dev.get_device_info()
        serial = info.get_serial_number()

        pipeline = ob.Pipeline(dev)
        config = ob.Config()

        # 选择 profile
        color_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        depth_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
        # Color
        try:
            color_profile = color_profiles.get_video_stream_profile(640, 480, ob.OBFormat.RGB, 30)
        except Exception:
            color_profile = color_profiles.get_default_video_stream_profile()
        config.enable_stream(color_profile)
        # Depth
        try:
            depth_profile = depth_profiles.get_video_stream_profile(640, 480, ob.OBFormat.Y16, 30)
        except Exception:
            depth_profile = depth_profiles.get_default_video_stream_profile()
        config.enable_stream(depth_profile)

        # 对齐模式
        try:
            m = (mode or 'SW').upper()
            if m == 'HW':
                config.set_align_mode(ob.OBAlignMode.HW_MODE)
            elif m == 'SW':
                config.set_align_mode(ob.OBAlignMode.SW_MODE)
            else:
                config.set_align_mode(ob.OBAlignMode.DISABLE)
        except Exception as e:
            LOGGER.warning(f"[Cam {i}] set align mode failed: {e}")

        if enable_sync:
            try:
                pipeline.enable_frame_sync()
            except Exception:
                pass

        align_filter = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)

        # 每个相机获取彩色内参
        Kc, Dc, _ = get_color_intrinsics_from_pipeline(pipeline)
        if Kc is None:
            LOGGER.warning(f"[Cam {i}] 彩色内参获取失败，使用回退")
            Kc = FALLBACK_CAMERA_MATRIX.copy()
            Dc = FALLBACK_DIST_COEFFS.copy()
        LOGGER.info(f"[Cam {i}] SN={serial} | Kc: fx={Kc[0,0]:.1f}, fy={Kc[1,1]:.1f}, cx={Kc[0,2]:.1f}, cy={Kc[1,2]:.1f}")

        # 启动并注册回调
        pipeline.start(
            config,
            lambda fs, idx=i, af=align_filter: on_new_frame_callback_multicam(fs, idx, af, color_queues, depth_queues)
        )

        cams.append({
            'idx': i,
            'serial': serial,
            'pipeline': pipeline,
            'K': Kc,
            'D': Dc,
            'x_hist': deque(maxlen=5),  # 你原有的 X 平滑
        })

    LOGGER.info(f"Started {len(cams)} Orbbec cameras.")

    # 3) 主循环：逐相机取最新帧 -> YOLO 推理 -> 输出相机ID/序列号/分区
    try:
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
        save_dir.mkdir(parents=True, exist_ok=True)
        world_txt_path = os.path.join(str(save_dir), 'world_coords.txt')
        last_targets = {}
        target_name_lc = (target_name or '').strip().lower()
        last_send_ts = time.time()

        while True:
            any_frame = False
            for cam in cams:
                i = cam['idx']
                if color_queues[i].empty() or depth_queues[i].empty():
                    continue

                any_frame = True
                color_frame = q_get_latest(color_queues[i])
                depth_frame = q_get_latest(depth_queues[i])
                if color_frame is None or depth_frame is None:
                    continue

                # 彩色图
                im0_bgr = frame_to_bgr_image(color_frame)
                if im0_bgr is None:
                    continue
                im0_rgb = cv2.cvtColor(im0_bgr, cv2.COLOR_BGR2RGB)
                H, W = im0_rgb.shape[:2]

                # 深度（米）
                try:
                    d_h, d_w = depth_frame.get_height(), depth_frame.get_width()
                    depth_raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(d_h, d_w)
                    depth_scale = float(getattr(depth_frame, "get_depth_scale", lambda: DEFAULT_DEPTH_SCALE)())
                    depth_m = depth_raw.astype(np.float32) * depth_scale
                    if (d_h != H) or (d_w != W):
                        depth_m = cv2.resize(depth_m, (W, H), interpolation=cv2.INTER_NEAREST)
                except Exception:
                    depth_m = None

                # xyz_map（兜底：用深度+彩色内参重建）
                xyz_map = depth_to_xyz_map(depth_m, cam['K']) if depth_m is not None else None

                # YOLO 预处理 + 推理
                letterboxed = letterbox(im0_rgb, imgsz, stride=stride, auto=pt)[0]
                im_tensor = torch.from_numpy(letterboxed).permute(2, 0, 1).unsqueeze(0).to(device)
                im_tensor = im_tensor.half() if half else im_tensor.float()
                im_tensor /= 255.0
                pred = model(im_tensor, augment=augment, visualize=visualize)
                det_list = non_max_suppression_obb(pred, conf_thres=conf_thres, iou_thres=iou_thres,
                                                   classes=None, agnostic=False, multi_label=True, max_det=max_det)

                im_anno = im0_rgb.copy()
                annotator = Annotator(im_anno, line_width=line_thickness, example=str(names))

                if len(det_list) and len(det_list[0]):
                    det = det_list[0]
                    polys = rbox2poly(det[:, :5])
                    polys = scale_polys(im_tensor.shape[2:], polys, im0_rgb.shape)
                    det_numpy = torch.cat((polys, det[:, -2:]), dim=1).cpu().numpy()

                    for *poly, conf, cls in reversed(det_numpy):
                        poly_list = list(poly)
                        label = f'{names[int(cls)]} {conf:.2f}'
                        u = v = None
                        cam_coords = None
                        world_coords = None

                        if xyz_map is not None:
                            # ROI 点云 -> 3D圆柱中心（与你原先一致）
                            mask = polygon_to_mask(poly_list, (H, W), erode_px=2)
                            pts = xyz_map[mask]
                            valid = np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 1e-9)
                            pts = pts[valid]

                            if pts.shape[0] >= 50:
                                center3d, axis3d, n_used = estimate_cylinder_center_from_points(
                                    pts,
                                    known_radius_m=cyl_radius_m,
                                    voxel_size=pc_voxel_size,
                                    nb_neighbors=pc_nb_neighbors,
                                    std_ratio=pc_std_ratio,
                                    slice_thickness_m=pc_slice_thickness_m,
                                    min_points=pc_min_points,
                                    max_points=pc_max_points,
                                    force_circle_fit=pc_use_circle_fit,
                                    gate_z_max_m=None if pc_gate_z_max_mm is None else float(pc_gate_z_max_mm)/1000.0,
                                    gate_y_range_m=None if (pc_gate_y_min_mm is None or pc_gate_y_max_mm is None)
                                                       else (float(pc_gate_y_min_mm)/1000.0, float(pc_gate_y_max_mm)/1000.0)
                                )
                            else:
                                center3d, axis3d, n_used = (None, None, 0)

                            if center3d is not None:
                                cam_coords = center3d
                                u, v = project_point_to_pixel(cam['K'], cam_coords)
                                world_coords = camera_to_world(cam_coords)
                                label += f' | Pts:{n_used}'
                            else:
                                # 回退：像素中心 + ROI 深度
                                if depth_m is not None:
                                    u, v = get_fused_u_for_symmetric_cylinder(depth_m, poly_list, cam['K'])
                                    depth_val_m = depth_from_roi(depth_m, poly_list)
                                    if depth_val_m is not None and depth_val_m > 0:
                                        cam_coords = pixel_to_camera((u, v), depth_val_m, cam['K'], cam['D'])
                                        world_coords = camera_to_world(cam_coords)
                        else:
                            # 没有点云：像素法
                            if depth_m is not None:
                                u, v = get_fused_u_for_symmetric_cylinder(depth_m, poly_list, cam['K'])
                                depth_val_m = depth_from_roi(depth_m, poly_list)
                                if depth_val_m is not None and depth_val_m > 0:
                                    cam_coords = pixel_to_camera((u, v), depth_val_m, cam['K'], cam['D'])
                                    world_coords = camera_to_world(cam_coords)

                        # 分区判定
                        zone_id, zr, zc = which_zone(u, v, W, H, rows=zones_rows, cols=zones_cols)
                        # if zone_id is not None:
                        #     print(f"[Cam {i} | SN {cam['serial']}] Zone 第{zone_id}区 -> {names[int(cls)]} conf={conf:.2f}")
                        # else:
                        #     print(f"[Cam {i} | SN {cam['serial']}] -> {names[int(cls)]} conf={conf:.2f}")

                        # [MOD] 只对指定目标类别输出与聚合（默认 '榴弹'）
                        try:
                            cls_name = names[int(cls)]
                        except Exception:
                             cls_name = str(int(cls))
                        zone_val = int(zone_id) if zone_id is not None else 0

                        if world_coords is not None and cls_name.lower() == target_name_lc:
                            # 与单相机一致：交换x/y并对x取反，再转mm
                            wx_mm = float(world_coords[1] * 1000.0)
                            wy_mm = float(-world_coords[0] * 1000.0)
                            wz_mm = float(world_coords[2] * 1000.0)

                            # 你要求的输出格式（包含：世界坐标、类别、区域、相机ID、END）
                            data_line = (
                                f"vision_data:X={wx_mm:.1f} ，Y={wy_mm:.1f},Z={wz_mm:.1f},"
                                f"CLASS={cls_name}，AREA={zone_val} ,ID={i},END"
                            )
                            print(data_line)
                            # 聚合：每路相机只保留最近一次
                            last_targets[i] = data_line
                        else:
                            # 非目标类别（或无世界坐标）时，可选打印调试信息
                            pass

                        annotator.poly_label(poly_list, label, color=colors(int(cls), True))

                # 显示
                if view_img:
                    cv2.imshow(f"Cam {i} | SN {cam['serial']}", cv2.cvtColor(annotator.result(), cv2.COLOR_RGB2BGR))

            # 键盘退出
            if view_img:
                key = cv2.waitKey(1) & 0xFF
                if key in [ord('q'), ESC_KEY]:
                    break

            # 周期性触发，写world_coords.txt 并通过UDP发送
            now = time.time()
            if now - last_send_ts >= float(udp_period):
                try:
                    with open(world_txt_path, 'w', encoding='utf-8') as f:
                        if len(last_targets) == 0:
                            f.write("NONE\n")
                        else:
                            for cam_idx in sorted(last_targets.keys()):
                                f.write(last_targets[cam_idx] + "\n")
                    # udp_send_file(world_txt_path, udp_ip, int(udp_port),
                    #               chunk_size=int(udp_chunk), timeout=float(udp_timeout))
                    udp_send_file_async(world_txt_path, udp_ip, int(udp_port),
                                  chunk_size=int(udp_chunk), timeout=float(udp_timeout))

                    LOGGER.info(
                        f"[UDP] 已发送 {world_txt_path} -> {udp_ip}:{udp_port} | 条目: {len(last_targets) if last_targets else 'NONE'}")
                except Exception as e:
                    LOGGER.warning(f"[UDP] 发送失败: {e}")
                finally:
                    last_targets.clear()
                    last_send_ts = now

            # 若本轮没有任何帧，可小睡避免空转
            if not any_frame:
                time.sleep(0.005)

    finally:
        for cam in cams:
            try:
                cam['pipeline'].stop()
            except Exception:
                pass
        cv2.destroyAllWindows()
        LOGGER.info("Stopped all Orbbec pipelines.")


# ---------- 主运行函数 ----------
@torch.no_grad()
def run(weights=ROOT / 'runs/best.pt',
        source='1',
        imgsz=(640, 640),
        conf_thres=0.60,
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
        save_csv=False,
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
        pc_gate_y_max_mm=400):
    source = str(source)
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)
    half &= (pt or jit or engine) and device.type != 'cpu'
    if pt or jit:
        model.model.half() if half else model.model.float()

    use_orbbec = (source.strip() == '1') and HAS_ORBBEC

    csv_file = None
    csv_writer = None
    txt_file = None
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

    if use_orbbec:
        LOGGER.info("开始 Orbbec 摄像头实时检测")

        # 保存目录与TXT路径，10秒聚合器
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
        save_dir.mkdir(parents=True, exist_ok=True)
        world_txt_path = os.path.join(str(save_dir), 'world_coords.txt')

        # 周期聚合器：记录10秒内每路相机最新的目标世界坐标
        last_send_ts = time.time()
        last_targets = {}  # {cam_idx: 'vision_data:...'}
        target_name_lc = (target_name or '').strip().lower()

        if save_csv:
            csv_path = os.path.join(str(save_dir), 'results.csv')
            csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['frame', 'class', 'conf', 'u', 'v', 'depth_m',
                                 'cam_x', 'cam_y', 'cam_z', 'world_x', 'world_y', 'world_z'])

            # 同时创建TXT文件
            txt_path = os.path.join(str(save_dir), 'world_coords.txt')
            txt_file = open(txt_path, 'w', encoding='utf-8')

        video_path = os.path.join(str(save_dir), 'orbbec_detected_video.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        try:
            video_writer = cv2.VideoWriter(video_path, fourcc, DEPTH_FPS, (DEPTH_WIDTH, DEPTH_HEIGHT))
        except Exception as e:
            LOGGER.warning(f"无法创建视频写入器: {e}")
            video_writer = None

        processed_frames_count = 0
        dt = [0.0, 0.0, 0.0]
        x_hist = deque(maxlen=pc_smooth)
        x_baseline = None
        use_open3d_flag = (pc_backend != 'numpy') and HAS_O3D

        # 毫米阈值 -> 米
        gate_z_max_m = None if pc_gate_z_max_mm is None else float(pc_gate_z_max_mm) / 1000.0
        if (pc_gate_y_min_mm is not None) and (pc_gate_y_max_mm is not None):
            gate_y_range_m = (float(pc_gate_y_min_mm) / 1000.0, float(pc_gate_y_max_mm) / 1000.0)
        else:
            gate_y_range_m = None

        # 官方对齐 + 点云滤镜
        try:
            align_filter = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
        except Exception:
            align_filter = None
            LOGGER.warning("AlignFilter 创建失败，将使用原始帧集")

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
            LOGGER.warning("PointCloudFilter 创建失败，将无法生成整帧点云")

        intr_ready = False  # 首帧用彩色相机内参
        last_unit_checked = False

        try:
            while True:
                frameset = pipeline.wait_for_frames(1000)
                if frameset is None:
                    LOGGER.warning("未能获取到 Orbbec 帧集")
                    time.sleep(0.01)
                    continue

                frames_aligned = align_filter.process(frameset) if align_filter is not None else frameset
                color_frame = frames_aligned.get_color_frame()
                depth_frame = frames_aligned.get_depth_frame()
                if (color_frame is None) or (depth_frame is None):
                    LOGGER.warning("未能获取到彩色或深度帧")
                    continue

                # 新的（从 pipeline 取彩色相机内参）
                if not intr_ready:
                    Kc, Dc, _ = get_color_intrinsics_from_pipeline(pipeline)
                    if Kc is not None:
                        camera_matrix = Kc
                        dist_coeffs = Dc if Dc is not None else np.zeros(5, np.float32)
                        intr_ready = True
                        LOGGER.info(f"使用彩色相机内参: fx={camera_matrix[0, 0]:.2f}, fy={camera_matrix[1, 1]:.2f}, "
                                    f"cx={camera_matrix[0, 2]:.2f}, cy={camera_matrix[1, 2]:.2f}")
                    else:
                        LOGGER.warning("彩色相机内参获取失败，继续使用回退/深度内参")

                # 每帧拿 depth_scale，并告知点云滤镜（保证 xyz 单位为米）
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
                    LOGGER.warning("彩色帧转换失败")
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
                                    LOGGER.warning(f"点云数据尺寸不匹配: got {arr.size} floats, expect {ch * cw * 3}")
                                    xyz_map = None
                    except Exception as e:
                        LOGGER.warning(f"PointCloudFilter 处理失败: {e}")
                        xyz_map = None

                # 兜底：若点云滤镜不可用，直接用深度图+彩色内参生成 xyz_map（米）
                if xyz_map is None and (depth_meters is not None) and (camera_matrix is not None):
                    xyz_map = depth_to_xyz_map(depth_meters, camera_matrix)

                # 单位兜底：若发现像毫米，则转米（只需做一次）
                if (xyz_map is not None) and (not last_unit_checked):
                    z_med = np.nanmedian(np.abs(xyz_map[..., 2]))
                    if np.isfinite(z_med) and z_med > 20.0:
                        xyz_map = xyz_map / 1000.0
                        LOGGER.info("检测到点云疑似毫米单位，已自动转换为米")
                    else:
                        LOGGER.info("点云单位确认：米")
                    last_unit_checked = True

                # YOLO 前处理
                letterboxed_img = letterbox(im0_rgb, imgsz, stride=stride, auto=pt)[0]
                im_tensor = torch.from_numpy(letterboxed_img).permute(2, 0, 1).unsqueeze(0).to(device)
                im_tensor = im_tensor.half() if half else im_tensor.float()
                im_tensor /= 255.0
                t2 = time_sync()
                dt[0] += (t2 - t1)

                # YOLO 推理
                pred = model(im_tensor, augment=augment, visualize=visualize)
                t3 = time_sync()
                dt[1] += (t3 - t2)
                pred = non_max_suppression_obb(pred, conf_thres=conf_thres, iou_thres=iou_thres,
                                               classes=classes, agnostic=agnostic_nms,
                                               multi_label=True, max_det=max_det)
                t4 = time_sync()
                dt[2] += (t4 - t3)

                # 处理结果
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

                        if xyz_map is not None:
                            H, W = xyz_map.shape[:2]
                            mask = polygon_to_mask(poly_list, (H, W), erode_px=2)
                            pts = xyz_map[mask]
                            valid = np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 1e-9)
                            pts = pts[valid]
                            LOGGER.info(
                                f"[pc-viz] pc_viz={pc_viz}, HAS_O3D={HAS_O3D}, xyz_map={'ok' if xyz_map is not None else 'None'}, ROI_pts={pts.shape[0]}")

                            if pts.shape[0] >= 50:
                                center3d, axis3d, n_used = estimate_cylinder_center_from_points(
                                    pts,
                                    known_radius_m=cyl_radius_m,
                                    voxel_size=pc_voxel_size,
                                    nb_neighbors=pc_nb_neighbors,
                                    std_ratio=pc_std_ratio,
                                    slice_thickness_m=pc_slice_thickness_m,
                                    min_points=pc_min_points,
                                    max_points=pc_max_points,
                                    force_circle_fit=pc_use_circle_fit,
                                    gate_z_max_m=gate_z_max_m,
                                    gate_y_range_m=gate_y_range_m
                                )
                            else:
                                center3d, axis3d, n_used = (None, None, 0)

                            if center3d is not None:
                                cam_coords = center3d  # 单位米
                                depth_val_m = float(np.median(pts[:, 2]))
                                # 将 3D 中心投影回像素，用于 CSV/调试
                                u, v = project_point_to_pixel(camera_matrix, cam_coords)

                                x_hist.append(float(center3d[0]))
                                x_smooth = float(np.median(x_hist)) if len(x_hist) else float(center3d[0])
                                delta_x = (x_smooth - x_baseline) if x_baseline is not None else None

                                world_coords = camera_to_world(center3d)
                                label_text += f' | Pts:{n_used} | Z:{depth_val_m:.2f}m | X:{x_smooth:.3f}m'
                                if delta_x is not None:
                                    label_text += f' ΔX:{delta_x:.3f}m'

                                im0_annotated = draw_axes_at_point(im0_annotated, cam_coords, camera_matrix,
                                                                   dist_coeffs, axis_length=0.05)

                                if pc_viz and HAS_O3D and (pts.shape[0] >= 10):
                                    try:
                                        show_o3d_debug_like_user_pc(
                                            xyz_map, poly_list,
                                            center3d,
                                            axis3d,
                                            voxel_mm=pc_viz_voxel_mm,
                                            outlier_nn=pc_viz_outlier_nn,
                                            outlier_std=pc_viz_outlier_std,
                                            sphere_mm=pc_viz_sphere_mm,
                                            axis_mm=pc_viz_axis_mm
                                        )
                                    except Exception as e:
                                        LOGGER.warning(f"pc-viz 可视化失败: {e}")
                            else:
                                # 回退：像素中心 + 深度回投
                                if depth_meters is not None:
                                    u, v = get_fused_u_for_symmetric_cylinder(depth_meters, poly_list, camera_matrix)
                                    depth_val_m = depth_from_roi(depth_meters, poly_list)
                                    if depth_val_m is not None and depth_val_m > 0:
                                        cam_coords = pixel_to_camera((u, v), depth_val_m, camera_matrix, dist_coeffs,
                                                                     pp_delta)
                                        if cam_coords is not None:
                                            world_coords = camera_to_world(cam_coords)
                                            label_text += f' | Z:{depth_val_m:.2f}m'
                                            label_text += f' | Cam:({cam_coords[0]:.2f},{cam_coords[1]:.2f},{cam_coords[2]:.2f})'
                                            if world_coords is not None:
                                                label_text += f' | W:({world_coords[0]:.2f},{world_coords[1]:.2f},{world_coords[2]:.2f})'
                                            im0_annotated = draw_axes_at_point(im0_annotated, cam_coords, camera_matrix,
                                                                               dist_coeffs, axis_length=0.05)
                        else:
                            # xyz_map 不可用时，回退像素法
                            if depth_meters is not None:
                                u, v = get_fused_u_for_symmetric_cylinder(depth_meters, poly_list, camera_matrix)
                                depth_val_m = depth_from_roi(depth_meters, poly_list)
                                if depth_val_m is not None and depth_val_m > 0:
                                    cam_coords = pixel_to_camera((u, v), depth_val_m, camera_matrix, dist_coeffs,
                                                                 pp_delta)
                                    if cam_coords is not None:
                                        world_coords = camera_to_world(cam_coords)
                                        label_text += f' | Z:{depth_val_m:.2f}m'
                                        label_text += f' | Cam:({cam_coords[0]:.2f},{cam_coords[1]:.2f},{cam_coords[2]:.2f})'
                                        if world_coords is not None:
                                            label_text += f' | W:({world_coords[0]:.2f},{world_coords[1]:.2f},{world_coords[2]:.2f})'
                                        im0_annotated = draw_axes_at_point(im0_annotated, cam_coords, camera_matrix,
                                                                           dist_coeffs, axis_length=0.05)
                            else:
                                label_text += ' | Depth:N/A'

                        annotator.poly_label(poly_list, label_text, color=colors(int(cls), True))

                        if csv_writer is not None:
                            row = [processed_frames_count, int(cls), float(conf),
                                   None if u is None else float(u),
                                   None if v is None else float(v),
                                   None if depth_val_m is None else float(depth_val_m)]
                            if cam_coords is not None:
                                row += [float(cam_coords[0]) * 1000, float(cam_coords[1]) * 1000,
                                        float(cam_coords[2]) * 1000]
                            else:
                                row += [None, None, None]
                            if world_coords is not None:

                                world_x_val = float(world_coords[1] * 1000)
                                world_y_val = (-1) * float(world_coords[0] * 1000)
                                world_z_val = float(world_coords[2] * 1000)
                                row += [float(world_coords[1] * 1000), (-1) * float(world_coords[0] * 1000),
                                        float(world_coords[2]) * 1000]
                                if txt_file is not None:
                                    # 假设 zone_id=0（单相机没有 zones区域），相机ID=0
                                    data_line = (
                                        f"vision_data:X={world_x_val:.1f} ，Y={world_y_val:.1f},Z={world_z_val:.1f},"
                                        f"CLASS={names[int(cls)]}，AREA={0} ,ID={0},END"
                                    )
                                    txt_file.write(data_line + "\n")
                            else:
                                row += [None, None, None]
                            csv_writer.writerow(row)

                im0_annotated = annotator.result()

                if view_img:
                    try:
                        cv2.imshow("YOLOv5-OBB Detection", cv2.cvtColor(im0_annotated, cv2.COLOR_RGB2BGR))
                    except Exception:
                        cv2.imshow("YOLOv5-OBB Detection", im0_annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        raise KeyboardInterrupt
                    elif key == ord('b'):
                        if len(x_hist) > 0:
                            x_baseline = float(np.median(x_hist))
                            LOGGER.info(f"[基线] 已设置 X 基线 = {x_baseline:.3f} m（按 r 清除）")
                    elif key == ord('r'):
                        x_baseline = None
                        LOGGER.info("[基线] 已清除 X 基线")

                if video_writer is not None and not nosave:
                    if video_writer.isOpened():
                        try:
                            video_writer.write(cv2.cvtColor(im0_annotated, cv2.COLOR_RGB2BGR))
                        except Exception:
                            video_writer.write(im0_annotated)

                processed_frames_count += 1
                LOGGER.info(
                    f"Frame {processed_frames_count}: pre:{(t2 - t1):.3f}s, infer:{(t3 - t2):.3f}s, nms:{(t4 - t3):.3f}s")

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
            if txt_file is not None:
                txt_file.close()
            cv2.destroyAllWindows()


# ---------- argparse ----------
def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=str(ROOT / 'runs/best.pt'), help='model path(s)')
    parser.add_argument('--source', type=str, default='1',
                        help='file/dir/URL/glob, 0 for webcam, "1" for Orbbec camera')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.60, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='0', help='cuda device')
    parser.add_argument('--view-img', action='store_true', default=True, help='show results')
    parser.add_argument('--save-txt', action='store_true', default=True, help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', default=False, help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class')
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
    parser.add_argument('--half', action='store_true', dest='half', default=False,
                        help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--mode', type=str, default='SW', help='Orbbec align mode: HW / SW / NONE')
    parser.add_argument('--enable_sync', type=bool, default=True, help='enable frame sync')

    # 额外
    parser.add_argument('--center-mode', type=str, default='poly', choices=['poly', 'bbox', 'mass'],
                        help='像素中心计算方式')
    parser.add_argument('--depth-percentiles', nargs=2, type=float, default=[25, 75], help='ROI 深度百分位（兜底像素法）')
    parser.add_argument('--pp-delta', nargs=2, type=float, default=[0.0, 0.0], help='主点微调 du dv（像素）')
    parser.add_argument('--save-csv', action='store_true', default=True, help='保存 3D 结果到 CSV')

    # 点云拟合参数
    parser.add_argument('--pc-backend', type=str, default='open3d', choices=['auto', 'open3d', 'numpy'],
                        help='点云处理后端')
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
    parser.add_argument('--pc-gate-z-max-mm', type=float, default=1800, help='Z<=此值(mm)')
    parser.add_argument('--pc-gate-y-min-mm', type=float, default=80, help='Y>=此值(mm)')
    parser.add_argument('--pc-gate-y-max-mm', type=float, default=400, help='Y<=此值(mm)')

    # 多相机参数
    parser.add_argument('--multi-cam', action='store_true', default=True, help='enable multi-Orbbec cameras')
    parser.add_argument('--zones-rows', type=int, default=3, help='zone grid rows')
    parser.add_argument('--zones-cols', type=int, default=3, help='zone grid cols')

    # UDP通信相关的参数  IP 端口 周期 串口 超时时间 目标名称
    parser.add_argument('--udp-ip', type=str, default='192.168.171.136', help='UDP receiver IP')
    parser.add_argument('--udp-port', type=int, default=8080, help='UDP receiver port')
    parser.add_argument('--udp-period', type=float, default=10.0, help='aggregate period seconds')
    parser.add_argument('--udp-chunk', type=int, default=1024, help='UDP chunk size bytes')
    parser.add_argument('--udp-timeout', type=float, default=2.0, help='UDP timeout seconds')
    parser.add_argument('--target-name', type=str, default='miehuoqi', help='target class name to trigger sending')

    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1
    print_args(FILE.stem, opt)
    return opt


def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    if opt.multi_cam:
        run_multicam(**vars(opt))
    else:
        run(**vars(opt))


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)


'''
我现在再次跟你描述一下我们项目的实际需要然后你看我现在这版代码怎么改合适一点 我们的项目是一个搭载了stewart平台的可移动平台
现在车的四周都各有一个奥比中光Gemini335L相机  用于识别榴弹炮 根据stewart平台的一个可抓取的范围  我们在车的四周划定了相对应的区域 我们相机通过目标检测识别到榴弹炮后会生成点云获取中心点的相机坐标点  之后通过这样的TRANSFORM_MATRIX = np.array([
    [0.0, 1.0, 0.0, -100],
    [1.0, 0.0, 0.0, -31.55],
    [0.0, 0.0, 1.0, 1030],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float32)  矩阵转化为世界坐标系 目前的这个矩阵只是适用于相机在车辆正前方的相机坐标系转化  接下来还得再次新增一个这样的矩阵处理两边的 把这个坐标系写入文本文件后  我们需要通过UDP将这个文本文件发送至工控机上  之后还要加入语音识别 将榴弹炮从一区搬到二区这样的文本文件也通过UDP发给工控机

我们现在成熟或者说已经完善的代码部分也就是第一版代码是目标检测到榴弹炮  我们生成对应的点云在车辆正前方获取到了较为准确的世界坐标  我们的平台可以完成抓取 但是第二版的代码 侧边的区域还需要再加一个 TRANSFORM_MATRIX这样的矩阵 而且我们现在要先加入两个相机  实现将榴弹炮从正前方的一区搬运到侧边的二区
那么第二版的代码需要完成的功能就是先判断两个相机所拍摄的区域有没有榴弹炮如果有，文本文件中返回以上代码中提到的格式相机的一个世界坐标系 如果没有则是返回NONE  两个相机都要完成  文本文件生成完后通过UDP发送到工控机上就行了

但是现在的第二版代码相比较于之前的第一版代码有一些很显著的问题 首先是因为要持续发文本文件给工控机导致我们的线程可能会阻塞吧  imshow显示的界面非常的卡  也无法生成点云  导致UDP发送的文件内容一直为NONE没有任何意义
接下来我希望你通过我的描述和代码告诉我我的代码该如何更改和优化 才能解决我的实际需求
'''


