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