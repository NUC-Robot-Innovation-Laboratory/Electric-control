# ******************************************************************************
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
YOLOv5 + Orbbec Depth 支持版 + 世界坐标转换
基于 Ultralytics YOLOv5-OBB
使用方式：
1. Orbbec 摄像头（支持深度）：
   python duiqi.py --weights runs/best.pt --conf-thres 0.85 --source 1 --mode HW

2. 常规图片/视频/摄像头（无深度）：
   python demo.py --weights yolov5s.pt --source 0
   python demo.py --weights yolov5s.pt --source path/to/image_or_video
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

# 尝试导入 Orbbec SDK
try:
    import pyorbbecsdk as ob
    has_orbbec = True
except Exception:
    ob = None
    has_orbbec = False
    print('[警告] 未检测到 pyorbbecsdk，Orbbec 深度功能将被禁用。')

# YOLOv5 相关导入（保持与你工程一致）
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 根目录
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression_obb, print_args, scale_coords, scale_polys,
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
DEPTH_SCALE = 0.001  # mm -> m（默认）
DEPTH_WIDTH, DEPTH_HEIGHT = 640, 480
DEPTH_FPS = 30

# 默认相机内参（如你已有真实标定，请替换）
CAMERA_MATRIX = np.array([
    [468.13320439, 0.0, 649.32089063],
    [0.0, 474.38540911, 367.82500508],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

DIST_COEFFS = np.array([-0.12393594, 0.0467302, 0.00263421, -0.00696342, -0.01053551], dtype=np.float32)

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
    """
    通用的 frame -> BGR numpy image 转换（适用于 color frame, RGB order）
    """
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

def pixel_to_camera(pixel_coords, depth_value, camera_matrix, dist_coeffs):
    if depth_value is None or depth_value <= 0:
        return None
    x, y = pixel_coords
    if camera_matrix is None:
        LOGGER.warning("相机内参缺失")
        return None
    pts = np.array([[[x, y]]], dtype=np.float32)
    try:
        und = cv2.undistortPoints(pts, camera_matrix, dist_coeffs, P=None)
        xn, yn = und[0, 0]
        X = xn * depth_value
        Y = yn * depth_value
        Z = depth_value
        return np.array([X, Y, Z], dtype=np.float32)
    except Exception as e:
        LOGGER.warning(f"undistortPoints failed: {e}. Fallback to raw formula.")
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        X = (x - cx) * depth_value / fx
        Y = (y - cy) * depth_value / fy
        Z = depth_value
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

# ---------- Orbbec 初始化（仅用 config.set_align_mode） ----------
def init_orbbec_pipeline(mode='SW', enable_sync=True, width=DEPTH_WIDTH, height=DEPTH_HEIGHT, fps=DEPTH_FPS):
    """
    初始化 Orbbec pipeline，仅使用 config.set_align_mode()，不使用 AlignFilter。
    返回 pipeline, info, used_mode
    """
    if not has_orbbec:
        raise RuntimeError("pyorbbecsdk 未安装或不可用。")

    pipeline = ob.Pipeline()
    config = ob.Config()
    info = {}

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

    # 获取 profile 列表
    try:
        color_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        depth_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
    except Exception as e:
        color_profiles = None
        depth_profiles = None
        LOGGER.warning(f"get_stream_profile_list failed: {e}")

    # 选择 profile
    color_profile = None
    depth_profile = None
    try:
        if color_profiles is not None:
            color_profile = color_profiles.get_video_stream_profile(width, height, ob.OBFormat.RGB, fps)
        if depth_profiles is not None:
            depth_profile = depth_profiles.get_video_stream_profile(width, height, ob.OBFormat.Y16, fps)
    except Exception as e:
        LOGGER.warning(f"selecting video profile failed: {e}")

    if color_profile is None:
        color_profile = color_profiles.get_default_video_stream_profile()
    if depth_profile is None:
        depth_profile = depth_profiles.get_default_video_stream_profile()

    if color_profile is None or depth_profile is None:
        LOGGER.error("未能找到 color 或 depth profile，初始化失败")
        raise RuntimeError("No color/depth profile available")

    config.enable_stream(color_profile)
    config.enable_stream(depth_profile)

    # 设置对齐模式
    mode_upper = (mode or 'SW').upper()
    try:
        if mode_upper == 'HW':
            if hasattr(pipeline, 'get_d2c_depth_profile_list'):
                hw_list = pipeline.get_d2c_depth_profile_list(color_profile, ob.OBAlignMode.HW_MODE)
                if hw_list and len(hw_list) > 0:
                    hw_profile = hw_list[0]
                    config = ob.Config()
                    config.enable_stream(hw_profile)
                    config.enable_stream(color_profile)
                    config.set_align_mode(ob.OBAlignMode.HW_MODE)
                    LOGGER.info("启用硬件 D2C 对齐（通过 get_d2c_depth_profile_list）")
                else:
                    LOGGER.warning("无可用 HW 对齐 profile，降级到 SW")
                    mode_upper = 'SW'
            else:
                config.set_align_mode(ob.OBAlignMode.HW_MODE)
                LOGGER.info("尝试设置 HW 对齐模式")
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

    # 启用帧同步
    if enable_sync:
        try:
            pipeline.enable_frame_sync()
        except Exception as e:
            LOGGER.warning(f"frame sync 失败: {e}")

    # 启动 pipeline
    try:
        pipeline.start(config)
        info['started'] = True
    except Exception as e:
        LOGGER.error(f"Pipeline start 失败: {e}")
        raise

    return pipeline, info, mode_upper

# ---------- 主运行函数 ----------
@torch.no_grad()
def run(weights=ROOT / 'runs/train/exp16/weights/best.pt',
        source='1',
        imgsz=(640, 640),
        conf_thres=0.25,
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
        enable_sync=True):
    source = str(source)

    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)

    half &= (pt or jit or engine) and device.type != 'cpu'
    if pt or jit:
        model.model.half() if half else model.model.float()

    use_orbbec = (source.strip() == '1') and has_orbbec

    # 如果要使用 Orbbec
    pipeline = None
    info = {}
    if use_orbbec:
        LOGGER.info("准备使用 Orbbec 摄像头")
        try:
            pipeline, info, used_mode = init_orbbec_pipeline(mode=mode, enable_sync=enable_sync,
                                                            width=DEPTH_WIDTH, height=DEPTH_HEIGHT,
                                                            fps=DEPTH_FPS)
            LOGGER.info(f"Orbbec 初始化成功: mode={used_mode}, info={info}")
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
        LOGGER.info("开始 Orbbec 摄像头实时检测（仅用 config.set_align_mode）")
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
        save_dir.mkdir(parents=True, exist_ok=True)
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
                t_total_start = time_sync()
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
                    im0 = cv2.cvtColor(im0, cv2.COLOR_BGR2RGB)
                except Exception:
                    pass

                # --- depth ---
                try:
                    depth_height = depth_frame.get_height()
                    depth_width = depth_frame.get_width()
                    depth_format = depth_frame.get_format()
                    data_bytes = depth_frame.get_data()

                    print(f"Depth frame info:")
                    print(f"  Width: {depth_width}, Height: {depth_height}")
                    print(f"  Format: {depth_format}")
                    print(f"  Data size: {len(data_bytes)}")
                    print(f"  Expected size (H*W*2): {depth_height * depth_width * 2}")

                    # ✅ 修复：将 numpy array 转为 bytes 再调用 .hex()
                    try:
                        if len(data_bytes) >= 10:
                            first_10_bytes = bytes(data_bytes[:10])
                            print(f"First 10 bytes (hex): {first_10_bytes.hex()}")
                            # 打印前10个 uint16 值
                            print(f"First 10 values (as uint16): {np.frombuffer(first_10_bytes, dtype=np.uint16)}")
                    except Exception as e:
                        LOGGER.warning(f"打印前10字节失败: {e}")

                    # 解析
                    if depth_format == ob.OBFormat.Y16:
                        depth_raw = np.frombuffer(data_bytes, dtype=np.uint16)
                        if depth_raw.size != depth_height * depth_width:
                            LOGGER.warning("Y16 数据大小不匹配")
                            depth_meters = None
                        else:
                            depth_raw = depth_raw.reshape((depth_height, depth_width))
                            valid_mask = (depth_raw > 0) & (depth_raw < 65535)
                            if not valid_mask.any():
                                LOGGER.warning("Y16 深度图无有效点")
                                depth_meters = None
                            else:
                                depth_meters = depth_raw.astype(np.float32) * DEPTH_SCALE
                                print(f"Y16 depth range: min={depth_raw.min():.2f}, max={depth_raw.max():.2f}")
                    else:
                        LOGGER.warning(f"未知深度格式: {depth_format}")
                        depth_meters = None

                    # 重采样到目标尺寸
                    if depth_meters is not None and depth_meters.shape != (DEPTH_HEIGHT, DEPTH_WIDTH):
                        depth_meters = cv2.resize(depth_meters, (DEPTH_WIDTH, DEPTH_HEIGHT), interpolation=cv2.INTER_LINEAR)

                except Exception as e:
                    LOGGER.warning(f"深度解码失败: {e}")
                    depth_meters = None

                # --- image preprocessing for model ---
                letterboxed_img = letterbox(im0, imgsz, stride=stride, auto=pt)[0]
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
                for i_det, det in enumerate(pred):
                    im0_annotated = im0.copy()
                    annotator = Annotator(im0_annotated, line_width=line_thickness, example=str(names))

                    if len(det):
                        pred_poly = rbox2poly(det[:, :5])
                        pred_poly = scale_polys(im_tensor.shape[2:], pred_poly, im0.shape)
                        det_numpy = torch.cat((pred_poly, det[:, -2:]), dim=1).cpu().numpy()

                        for c in np.unique(det_numpy[:, -1]):
                            n = (det_numpy[:, -1] == c).sum()
                            s = f"{n} {names[int(c)]}{'s' * (n > 1)}, "
                            LOGGER.info(f"Detected: {s.strip()}")

                        for *poly, conf, cls in reversed(det_numpy):
                            poly_list = list(poly)
                            label_text = f'{names[int(cls)]} {conf:.2f}'
                            current_camera_coords = None
                            current_world_coords = None

                            if depth_meters is not None:
                                x_coords = poly_list[::2]
                                y_coords = poly_list[1::2]
                                x1, y1 = int(min(x_coords)), int(min(y_coords))
                                x2, y2 = int(max(x_coords)), int(max(y_coords))

                                x1c, y1c = np.clip(x1, 0, depth_meters.shape[1] - 1), np.clip(y1, 0, depth_meters.shape[0] - 1)
                                x2c, y2c = np.clip(x2, 0, depth_meters.shape[1] - 1), np.clip(y2, 0, depth_meters.shape[0] - 1)

                                print("\n---目标检测详情---")
                                print(f"类别: {names[int(cls)]}")
                                print(f"置信度: {conf:.2f}")
                                print(f"边界框坐标: ({x1}, {y1}, {x2}, {y2})")

                                if x1c < x2c and y1c < y2c:
                                    roi_depth = depth_meters[y1c:y2c + 1, x1c:x2c + 1]
                                    print(f"ROI深度信息:")
                                    print(f"  形状: {roi_depth.shape}")
                                    print(f"  最小值: {roi_depth.min():.3f}m")
                                    print(f"  最大值: {roi_depth.max():.3f}m")
                                    print(f"  平均值: {roi_depth.mean():.3f}m")
                                    valid_depths = roi_depth[(roi_depth > 0) & (roi_depth < 10)]

                                    if valid_depths.size > 0:
                                        depth_val_m = float(np.median(valid_depths))
                                        print(f"深度处理:")
                                        print(f"  有效深度点数: {valid_depths.size}")
                                        print(f"  深度中位数: {depth_val_m:.2f}m")
                                        print(f"  深度标准差: {valid_depths.std():.3f}m")

                                        center_x_px = (x1 + x2) / 2.0
                                        center_y_px = (y1 + y2) / 2.0
                                        pixel_coords = [center_x_px, center_y_px]

                                        current_camera_coords = pixel_to_camera(pixel_coords, depth_val_m, CAMERA_MATRIX, DIST_COEFFS)
                                        if current_camera_coords is not None:
                                            print(f"相机坐标系:")
                                            print(f"  X: {current_camera_coords[0]:.3f}m")
                                            print(f"  Y: {current_camera_coords[1]:.3f}m")
                                            print(f"  Z: {current_camera_coords[2]:.3f}m")

                                            try:
                                                point_homogeneous = np.append(current_camera_coords, 1.0)
                                                world_homogeneous = TRANSFORM_MATRIX @ point_homogeneous
                                                world_coords = world_homogeneous[:3] / world_homogeneous[3]
                                                current_world_coords = world_coords

                                                print(f"世界坐标系:")
                                                print(f"  X: {world_coords[0]:.3f}m")
                                                print(f"  Y: {world_coords[1]:.3f}m")
                                                print(f"  Z: {world_coords[2]:.3f}m")

                                                label_text += f' | World: X:{world_coords[0]:.2f} Y:{world_coords[1]:.2f} Z:{world_coords[2]:.2f}'
                                            except Exception as e:
                                                LOGGER.error(f"世界坐标转换失败: {e}")
                                                current_world_coords = None

                                            label_text += f' | Cam: X:{current_camera_coords[0]:.2f} Y:{current_camera_coords[1]:.2f} Z:{current_camera_coords[2]:.2f}'
                                            im0_annotated = draw_axes_at_point(im0_annotated, current_camera_coords, CAMERA_MATRIX, DIST_COEFFS, axis_length=0.05)

                                            LOGGER.info(f"Object {names[int(cls)]} ({conf:.2f}): Pixel({center_x_px:.0f},{center_y_px:.0f}), Depth: {depth_val_m:.2f}m, Camera_Coords: ({current_camera_coords[0]:.3f}, {current_camera_coords[1]:.3f}, {current_camera_coords[2]:.3f})m")

                                            if current_world_coords is not None:
                                                LOGGER.info(f"World Coords: X={current_world_coords[0]:.3f}m, Y={current_world_coords[1]:.3f}m, Z={current_world_coords[2]:.3f}m")
                                    else:
                                        label_text += " | Cam:N/A"
                                else:
                                    label_text += " | ROI:Invalid"
                            else:
                                label_text += " | Depth:N/A"

                            annotator.poly_label(poly_list, label_text, color=colors(int(cls), True))

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
                    LOGGER.info(f"Frame {processed_frames_count}: Infer:{t3 - t2:.3f}s, NMS:{t4 - t3:.3f}s")

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
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'runs/train/exp16/weights/best.pt',
                        help='model path(s)')
    parser.add_argument('--source', type=str, default='1',
                        help='file/dir/URL/glob, 0 for webcam, "1" for Orbbec camera')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', default=True, help='show results')
    parser.add_argument('--save-txt', action='store_true', default=True, help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', default=True, help='save confidences in --save-txt labels')
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
    parser.add_argument('--mode', type=str, default='SW', help='Orbbec align mode: HW / SW / NONE')
    parser.add_argument('--enable_sync', type=bool, default=True, help='enable frame sync between color and depth')

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