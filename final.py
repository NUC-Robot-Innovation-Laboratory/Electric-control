# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
YOLOv5 + Orbbec Depth 支持版 + 世界坐标转换
基于 Ultralytics YOLOv5-OBB
使用方式：
1. Orbbec 摄像头（支持深度）：
   python final.py --weights runs/train/exp16/weights/best.pt --conf-thres 0.85 --source 1
   python final.py --weights best.pt --conf-thres 0.85 --source 1
   python final.py --weights runs/best.pt --conf-thres 0.85 --source 1

2. 常规图片/视频/摄像头（无深度）：
   python detect.py --weights yolov5s.pt --source 0
   python detect.py --weights yolov5s.pt --source path/to/image_or_video
"""

import argparse
import os
import sys
import time
from pathlib import Path
import numpy as np
import cv2
import torch
import torch.backends.cudnn as cudnn
from utils_or import frame_to_bgr_image

# 尝试导入 Orbbec SDK
try:
    import pyorbbecsdk as ob

    has_orbbec = True
except ImportError:
    has_orbbec = False
    print('[警告] 未检测到 pyorbbecsdk，Orbbec 深度功能将被禁用。')

# YOLOv5 相关导入
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 根目录
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression_obb, print_args, scale_coords, scale_polys,
                           strip_optimizer)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, time_sync
from utils.rboxs_utils import rbox2poly

# 修正：将 letterbox 从 utils.augmentations 导入
try:
    from utils.augmentations import letterbox
except ImportError:
    from utils.general import letterbox

    LOGGER.warning("letterbox function imported from utils.general.")

# ----- Orbbec 全局设置 -----
DEPTH_SCALE = 0.001  # 毫米转米
DEPTH_WIDTH, DEPTH_HEIGHT = 640, 480  # 摄像头分辨率
DEPTH_FPS = 30

# # 添加相机内参和畸变系数
CAMERA_MATRIX = np.array([
    [468.13320439, 0.0, 649.32089063],
    [0.0, 474.38540911, 367.82500508],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

DIST_COEFFS = np.array([-0.12393594, 0.0467302, 0.00263421, -0.00696342, -0.01053551], dtype=np.float32)


# ===== 新增：世界坐标转换矩阵 =====
# 相机坐标系到世界坐标系的变换矩阵 (4x4)  
# 原标定旋转
original_R = np.array([
    [9.30724545e-01, -2.57750005e-01, -2.59454727e-01],
    [6.86573375e-02, 8.19962375e-01, -5.68285028e-01],
    [3.59218583e-01, 5.11103353e-01, 7.80855538e-01]
], dtype=np.float32)

# 调整矩阵：交换X/Y (相机Y->世界X, 相机X->世界Y)，并翻转符号如果方向反
# 假设"向右"匹配"向右" (无翻转Y)；"向下"正 (无翻转X)；如果反，改diag为[-1,1,1]或[1,-1,1]
axis_adjust = np.array([
    [0, 1, 0],  # 相机Y -> 世界X
    [1, 0, 0],  # 相机X -> 世界Y
    [0, 0, 1]   # 相机Z -> 世界Z
], dtype=np.float32)

# 翻转矩阵 (如果方向反，调整符号；这里假设正向)
flip_matrix = np.diag([1, 1, 1])  # 无翻转；如果世界Y反，设[1,-1,1]

# 新旋转R = flip @ axis_adjust @ original_R
new_R = flip_matrix @ axis_adjust @ original_R

# 平移T (mm单位，您的值)
T_mm = [68, 1250, 1720]

TRANSFORM_MATRIX = np.array([
    [new_R[0,0], new_R[0,1], new_R[0,2], T_mm[0]],
    [new_R[1,0], new_R[1,1], new_R[1,2], T_mm[1]],
    [new_R[2,0], new_R[2,1], new_R[2,2], T_mm[2]],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float32)

# 将平移转为米
TRANSFORM_MATRIX[:3, 3] /= 1000.0



def frame_to_bgr_image(frame):
    if frame is None:
        return None
    try:
        data = np.frombuffer(frame.get_data(), dtype=np.uint8)
        img = data.reshape((frame.get_height(), frame.get_width(), 3))
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    except Exception as e:
        print(f"Failed to convert frame: {e}")
        return None



def camera_to_world(camera_coords):
    if camera_coords is None:
        return None

    x_cam, y_cam, z_cam = camera_coords

    # 根据实际定义调整映射和符号
    x_world = y_cam  # 交换X和Y
    y_world = x_cam
    z_world = z_cam

    camera_coords_mapped = np.array([x_world, y_world, z_world])

    point_homogeneous = np.append(camera_coords_mapped, 1.0)
    world_homogeneous = TRANSFORM_MATRIX @ point_homogeneous
    world_coords = world_homogeneous[:3] / world_homogeneous[3]

    return world_coords

# 添加获取相机ID的函数
def get_camera_id(pipeline):
    """获取 Orbbec 相机设备信息"""
    try:
        device = pipeline.get_device()
        device_info = device.get_device_info()
        return {
            'name': device_info.get_name(),
            'serial_number': device_info.get_serial_number(),
            'firmware_version': device_info.get_firmware_version()
        }
    except Exception as e:
        LOGGER.warning(f"获取相机ID失败: {e}")
        return None


# 添加像素坐标到相机坐标的转换函数
def pixel_to_camera(pixel_coords, depth_value, camera_matrix, dist_coeffs):
    """
    将像素坐标转换为相机坐标系下的3D坐标
    Args:
        pixel_coords: [x, y] 像素坐标
        depth_value: 深度值（米）
        camera_matrix: 3x3 内参矩阵
        dist_coeffs: 畸变系数
    Returns:
        camera_coords: [X, Y, Z] 相机坐标系下的3D坐标，如果输入无效则返回None
    """
    if depth_value is None or depth_value <= 0:
        return None

    x, y = pixel_coords

    # 验证内参矩阵和畸变系数
    if camera_matrix is None or dist_coeffs is None:
        LOGGER.warning("相机内参或畸变系数未提供，无法转换至相机坐标。")
        return None

    # 像素坐标转为归一化坐标 (去除畸变)
    points_distorted = np.array([[float(x), float(y)]], dtype=np.float32)
    try:
        undistorted_points = cv2.undistortPoints(points_distorted, camera_matrix, dist_coeffs, P=camera_matrix)
        x_undistorted, y_undistorted = undistorted_points[0, 0]
    except Exception as e:
        LOGGER.warning(f"去畸变失败: {e}. 使用原始像素坐标.")
        x_undistorted, y_undistorted = x, y  # 降级处理

    # 获取内参
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # 计算相机坐标
    Z = depth_value
    X = (x_undistorted - cx) * Z / fx
    Y = (y_undistorted - cy) * Z / fy

    return np.array([X, Y, Z])


def draw_axes_at_point(image, point3D, camera_matrix, dist_coeffs, axis_length=0.05):
    """
    在相机坐标系下指定三维点处绘制局部坐标轴
    Args:
        image: 输入图像 (OpenCV BGR format)
        point3D: 绘制坐标轴的原点，相机坐标系下的3D点 [X, Y, Z]
        camera_matrix: 3x3 内参矩阵
        dist_coeffs: 畸变系数
        axis_length: 坐标轴箭头的长度（单位：米）
    Returns:
        绘制了坐标轴的图像
    """
    if point3D is None or camera_matrix is None or dist_coeffs is None:
        return image  # 如果缺少必要信息，则不绘制

    # 构造坐标轴端点（相机坐标系下）
    axis = np.float32([
        [0, 0, 0],  # 原点
        [axis_length, 0, 0],  # X轴正方向
        [0, axis_length, 0],  # Y轴正方向
        [0, 0, axis_length]  # Z轴正方向
    ]).reshape(-1, 3)

    rvec = np.zeros((3, 1), dtype=np.float32)
    tvec = np.array(point3D, dtype=np.float32).reshape(3, 1)

    try:
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
        imgpts = imgpts.astype(int)

        p0 = tuple(imgpts[0].ravel())
        pX = tuple(imgpts[1].ravel())
        pY = tuple(imgpts[2].ravel())
        pZ = tuple(imgpts[3].ravel())

        cv2.line(image, p0, pX, (0, 0, 255), 2)  # X - 红
        cv2.line(image, p0, pY, (0, 255, 0), 2)  # Y - 绿
        cv2.line(image, p0, pZ, (255, 0, 0), 2)  # Z - 蓝
        cv2.circle(image, p0, 3, (0, 0, 0), -1)  # 原点黑点
    except Exception as e:
        LOGGER.warning(f"绘制坐标轴失败: {e}")

    return image


@torch.no_grad()
def run(weights=ROOT / 'best.pt',
        source=ROOT / 'data/images',
        imgsz=(640, 640),
        conf_thres=0.45,  # 匹配第一个代码的默认值
        iou_thres=0.45,  # 匹配第一个代码的默认值
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
        line_thickness=3,  # 匹配第一个代码的默认值
        hide_labels=False,
        hide_conf=False,
        half=False,
        dnn=False):
    source = str(source)

    # ----------------- 统一加载模型 -----------------
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)

    half &= (pt or jit or engine) and device.type != 'cpu'
    if pt or jit:
        model.model.half() if half else model.model.float()

    use_orbbec = str(source).strip() == '1'
    # ------------- 分支：使用 Orbbec 摄像头（支持深度） -------------
    if use_orbbec and has_orbbec:
        LOGGER.info("使用 Orbbec 摄像头模式")
        pipeline = ob.Pipeline()
        config = ob.Config()

        try:
            camera_id = get_camera_id(pipeline)
            if camera_id:
                LOGGER.info(f"相机信息: {camera_id}")

            color_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
            depth_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)

            color_profile = color_profiles.get_video_stream_profile(DEPTH_WIDTH, DEPTH_HEIGHT, ob.OBFormat.RGB,
                                                                    DEPTH_FPS)
            depth_profile = depth_profiles.get_video_stream_profile(DEPTH_WIDTH, DEPTH_HEIGHT, ob.OBFormat.Y16,
                                                                    DEPTH_FPS)

            if color_profile is None:
                raise Exception(f"未找到 {DEPTH_WIDTH}x{DEPTH_HEIGHT}@{DEPTH_FPS} RGB 彩色流，请检查摄像头支持的分辨率。")
            if depth_profile is None:
                raise Exception(f"未找到 {DEPTH_WIDTH}x{DEPTH_HEIGHT}@{DEPTH_FPS} Y16 深度流，请检查摄像头支持的分辨率。")

            config.enable_stream(color_profile)
            config.enable_stream(depth_profile)



            pipeline.start(config)

            ob_device = pipeline.get_device()
            try:
                ob_device.set_bool_property(ob.OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, True)
                LOGGER.info("自动曝光已成功开启。")
            except Exception as e:
                LOGGER.warning(f"设置自动曝光失败（可能不支持或权限不足）: {e}")
                LOGGER.info("Orbbec 自动曝光已启用")

            LOGGER.info("Orbbec 摄像头初始化完成 ")
        except Exception as e:
            LOGGER.error(f"Orbbec 相机初始化失败，回退到标准模式: {e}")
            use_orbbec = False  # 标记为未使用 Orbbec

    # ------------- 非 Orbbec 分支：调用原 YOLOv5 数据加载 ---------
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

    # ----------------- 开始推理 -----------------

    if use_orbbec:
        LOGGER.info("开始 Orbbec 摄像头实时检测...")

        # 修复：提前创建保存目录
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
        save_dir.mkdir(parents=True, exist_ok=True)

        # 视频保存路径
        video_path = os.path.join(str(save_dir), 'orbbec_detected_video.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        try:
            video_writer = cv2.VideoWriter(video_path, fourcc, DEPTH_FPS, (DEPTH_WIDTH, DEPTH_HEIGHT))
            LOGGER.info(f"检测视频将保存到: {video_path}")
        except Exception as e:
            LOGGER.warning(f"无法创建视频写入器: {e}")
            video_writer = None

        processed_frames_count = 0
        dt = [0.0, 0.0, 0.0]

        try:
            while True:
                t_total_start = time_sync()

                frameset = pipeline.wait_for_frames(1000)
                if frameset is None:
                    LOGGER.warning("未能获取到 Orbbec 帧集")
                    time.sleep(0.01)
                    continue

                color_frame = frameset.get_color_frame()
                depth_frame = frameset.get_depth_frame()

                if (color_frame is None) or (depth_frame is None):
                    LOGGER.warning("未能获取到彩色帧或深度帧")
                    continue

                # --- 获取彩色图像数据 ---
                t1 = time_sync()
                color_data = color_frame.get_data()
                if color_data is None or color_data.size == 0:
                    LOGGER.warning("彩色帧数据为空")
                    continue

                try:
                    im0 = color_data.reshape((DEPTH_HEIGHT, DEPTH_WIDTH, 3))  # Orbbec output is RGB HWC
                except Exception as e:
                    LOGGER.warning(
                        f"彩色数据重塑异常: {e}. 帧数据大小: {color_data.size}, 期望形状: ({DEPTH_HEIGHT}, {DEPTH_WIDTH}, 3)")
                    continue

                # --- 图像预处理 (与原版 detect.py 保持一致) ---
                letterboxed_img = letterbox(im0, imgsz, stride=stride, auto=pt)[0]  # Output is HWC, RGB
                im_tensor = torch.from_numpy(letterboxed_img).permute(2, 0, 1).unsqueeze(0).to(device)
                im_tensor = im_tensor.half() if half else im_tensor.float()
                im_tensor /= 255.0
                t2 = time_sync()
                dt[0] += (t2 - t1)

                # --- 模型推理 ---
                pred = model(im_tensor, augment=augment, visualize=visualize)
                t3 = time_sync()
                dt[1] += (t3 - t2)

                # --- NMS ---
                pred = non_max_suppression_obb(pred, conf_thres=conf_thres, iou_thres=iou_thres,
                                               classes=classes, agnostic=agnostic_nms,
                                               multi_label=True, max_det=max_det)
                t4 = time_sync()
                dt[2] += (t4 - t3)

                # --- 处理深度数据 ---
                depth_data = depth_frame.get_data()
                depth_meters = None
                if depth_data is not None and depth_data.size > 0:
                    try:
                        depth_raw = np.frombuffer(depth_data.tobytes(), dtype=np.uint16)
                        depth_raw = depth_raw.reshape((DEPTH_HEIGHT, DEPTH_WIDTH))
                        depth_meters = depth_raw.astype(np.float32) * DEPTH_SCALE
                    except Exception as e:
                        LOGGER.warning(f"深度数据处理异常: {e}")

                # --- 处理检测结果 ---
                for i_det, det in enumerate(pred):
                    im0_annotated = im0.copy()
                    annotator = Annotator(im0_annotated, line_width=line_thickness, example=str(names))

                    if len(det):
                        pred_poly = rbox2poly(det[:, :5])
                        pred_poly = scale_polys(im_tensor.shape[2:], pred_poly, im0.shape)
                        det_numpy = torch.cat((pred_poly, det[:, -2:]),
                                              dim=1).cpu().numpy()  # Convert to NumPy array once

                        # 修正：使用 np.unique 处理 NumPy 数组
                        for c in np.unique(det_numpy[:, -1]):
                            n = (det_numpy[:, -1] == c).sum()
                            s = f"{n} {names[int(c)]}{'s' * (n > 1)}, "
                            LOGGER.info(f"Detected: {s.strip()}")  # Log detections per class


                        for *poly, conf, cls in reversed(det_numpy): # iterate over numpy array
                            # Ensure poly is a list for Annotator.poly_label
                            poly_list = list(poly)
                            label_text = f'{names[int(cls)]} {conf:.2f}'
                            current_camera_coords = None # Initialize for each detection
                            current_world_coords = None  # 新增：存储世界坐标

                            if depth_meters is not None:
                                x_coords = poly_list[::2]
                                y_coords = poly_list[1::2]
                                x1, y1 = int(min(x_coords)), int(min(y_coords))
                                x2, y2 = int(max(x_coords)), int(max(y_coords))

                                x1_clip, y1_clip = np.clip(x1, 0, DEPTH_WIDTH - 1), np.clip(y1, 0, DEPTH_HEIGHT - 1)
                                x2_clip, y2_clip = np.clip(x2, 0, DEPTH_WIDTH - 1), np.clip(y2, 0, DEPTH_HEIGHT - 1)

                                print("\n---目标检测详情---")
                                print(f"类别: {names[int(cls)]}")
                                print(f"置信度: {conf:.2f}")
                                print(f"边界框坐标: ({x1}, {y1}, {x2}, {y2})")

                                if x1_clip < x2_clip and y1_clip < y2_clip:
                                    roi_depth = depth_meters[y1_clip:y2_clip, x1_clip:x2_clip]
                                    print(f"ROI深度信息:")
                                    print(f"  形状: {roi_depth.shape}")
                                    print(f"  最小值: {roi_depth.min():.3f}m")
                                    print(f"  最大值: {roi_depth.max():.3f}m")
                                    print(f"  平均值: {roi_depth.mean():.3f}m") 

                                    valid_depths = roi_depth[(roi_depth > 0) & (roi_depth < 10)]
                                    if valid_depths.size > 0:
                                        depth_val_m = np.median(valid_depths)
                                        print(f"深度处理:")
                                        print(f"  有效深度点数: {valid_depths.size}")
                                        print(f"  深度中位数: {depth_val_m:.2f}m")
                                        print(f"  深度标准差: {valid_depths.std():.3f}m") 
                                        
                                        # 修复：在调用 pixel_to_camera 之前定义 pixel_coords
                                        center_x_px = (x1 + x2) / 2
                                        center_y_px = (y1 + y2) / 2
                                        pixel_coords = [center_x_px, center_y_px]

                                        # 计算相机坐标系坐标
                                        current_camera_coords = pixel_to_camera(pixel_coords, depth_val_m, CAMERA_MATRIX, DIST_COEFFS)
                                        if current_camera_coords is not None:
                                            print(f"相机坐标系:")
                                            print(f"  X: {current_camera_coords[0]:.3f}m")
                                            print(f"  Y: {current_camera_coords[1]:.3f}m")
                                            print(f"  Z: {current_camera_coords[2]:.3f}m") 
                                            
                                            # ===== 转换为世界坐标系 =====
                                            try:
                                                # 转换为齐次坐标 (添加w=1)
                                                point_homogeneous = np.append(current_camera_coords, 1.0)
                                                
                                                # 应用变换矩阵
                                                world_homogeneous = TRANSFORM_MATRIX @ point_homogeneous
                                                
                                                # 转换回3D坐标 (除以w分量)
                                                world_coords = world_homogeneous[:3] / world_homogeneous[3]
                                                current_world_coords = world_coords
                                                
                                                print(f"世界坐标系:")
                                                print(f"  X: {world_coords[0]:.3f}m")
                                                print(f"  Y: {world_coords[1]:.3f}m")
                                                print(f"  Z: {world_coords[2]:.3f}m")
                                                
                                                # 将世界坐标添加到标签
                                                label_text += f' | World: X:{world_coords[0]:.2f} Y:{world_coords[1]:.2f} Z:{world_coords[2]:.2f}'
                                            except Exception as e:
                                                LOGGER.error(f"世界坐标转换失败: {e}")
                                                current_world_coords = None
                                            # ===== 结束世界坐标转换 =====
                                            
                                            label_text += f' | Cam: X:{current_camera_coords[0]:.2f} Y:{current_camera_coords[1]:.2f} Z:{current_camera_coords[2]:.2f}'
                                            im0_annotated = draw_axes_at_point(im0_annotated, current_camera_coords, CAMERA_MATRIX, DIST_COEFFS, axis_length=0.05)
                                            # 新增：打印相机坐标到控制台
                                            LOGGER.info(f"Object {names[int(cls)]} ({conf:.2f}): Pixel({center_x_px:.0f},{center_y_px:.0f}), Depth: {depth_val_m:.2f}m, Camera_Coords: ({current_camera_coords[0]:.3f}, {current_camera_coords[1]:.3f}, {current_camera_coords[2]:.3f})m")
                                            
                                            # 新增：如果世界坐标转换成功，也打印到日志
                                            if current_world_coords is not None:
                                                LOGGER.info(f"World Coords: X={current_world_coords[0]:.3f}m, Y={current_world_coords[1]:.3f}m, Z={current_world_coords[2]:.3f}m")
                                        else:
                                            label_text += " | XYZ:N/A"
                                    else:
                                        label_text += " | Depth:N/A"
                                else:
                                     label_text += " | ROI:Invalid"
                            else:
                                label_text += " | Depth:N/A"  # 如果没有深度数据

                            annotator.poly_label(poly_list, label_text, color=colors(int(cls), True)) # Pass poly_list
                    im0_annotated = annotator.result() # Get final annotated image (still RGB)
                    # --- 显示检测结果 ---
                    if view_img:
                        cv2.imshow("YOLOv5-OBB Detection", cv2.cvtColor(im0_annotated,
                                                                        cv2.COLOR_RGB2BGR))  # Convert RGB to BGR for OpenCV display
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                    # --- 写入视频帧 ---
                    if video_writer is not None and not nosave:
                        if video_writer.isOpened():
                            video_writer.write(cv2.cvtColor(im0_annotated, cv2.COLOR_RGB2BGR))

                    processed_frames_count += 1
                    LOGGER.info(f"Frame {processed_frames_count}: Infer:{t3 - t2:.3f}s, NMS:{t4 - t3:.3f}s")

        except Exception as e:
            LOGGER.error(f"Orbbec 相机处理循环出错: {e}")
        finally:
            LOGGER.info("停止 Orbbec 相机...")
            pipeline.stop()
            if video_writer is not None:
                video_writer.release()
                LOGGER.info(f"检测视频已保存到 {video_path}")
            cv2.destroyAllWindows()

    else:
        # ---------------- 非 Orbbec 分支：使用原 YOLOv5 数据加载方式 ----------------
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

            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(im, augment=augment, visualize=visualize)
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
                    det_numpy = torch.cat((pred_poly, det[:, -2:]), dim=1).cpu().numpy()  # Convert to NumPy array once

                    # 修正：使用 np.unique 处理 NumPy 数组
                    for c in np.unique(det_numpy[:, -1]):
                        n = (det_numpy[:, -1] == c).sum()
                        s_log_line += f"{n} {names[int(c)]}{'s' * (n > 1)}, "

                    for *poly, conf, cls in reversed(det_numpy):  # iterate over numpy array
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


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'best.pt',
                        help='model path(s)')
    parser.add_argument('--source', type=str, default='1',
                        help='file/dir/URL/glob, 0 for webcam, "1" for Orbbec camera')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.45, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', default=True, help='show results')
    parser.add_argument('--save-txt', action='store_true', default=True, help='save results to *.txt')  # 默认保存txt
    parser.add_argument('--save-conf', action='store_true', default=True,
                        help='save confidences in --save-txt labels')  # 默认保存置信度
    parser.add_argument('--save-crop', action='store_true', default=True, help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', default=False, help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
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