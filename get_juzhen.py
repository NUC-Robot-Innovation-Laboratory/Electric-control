# import numpy as np
# from scipy.optimize import least_squares
# import math
#
# def euler_to_rotation_matrix(roll, pitch, yaw):
#     """将欧拉角转换为旋转矩阵 (ZYX顺序)"""
#     R_x = np.array([[1, 0, 0],
#                     [0, np.cos(roll), -np.sin(roll)],
#                     [0, np.sin(roll), np.cos(roll)]])
#
#     R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
#                     [0, 1, 0],
#                     [-np.sin(pitch), 0, np.cos(pitch)]])
#
#     R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
#                     [np.sin(yaw), np.cos(yaw), 0],
#                     [0, 0, 1]])
#
#     return R_z @ R_y @ R_x
#
#
# def rotation_matrix_to_euler(R):
#     """将旋转矩阵转换为欧拉角 (ZYX顺序)"""
#     sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
#
#     singular = sy < 1e-6
#
#     if not singular:
#         roll = np.arctan2(R[2, 1], R[2, 2])
#         pitch = np.arctan2(-R[2, 0], sy)
#         yaw = np.arctan2(R[1, 0], R[0, 0])
#     else:
#         roll = np.arctan2(-R[1, 2], R[1, 1])
#         pitch = np.arctan2(-R[2, 0], sy)
#         yaw = 0
#
#     return np.array([roll, pitch, yaw])
#
#
# def estimate_camera_orientation(camera_points, world_points):
#     """
#     估计相机的姿态（欧拉角和平移向量）
#     参数:
#         camera_points: 相机坐标系中的点 (Nx3)
#         world_points: 世界坐标系中的点 (Nx3)
#     返回:
#         roll, pitch, yaw: 欧拉角（弧度）
#         t: 平移向量 (3x1)
#     """
#     # 初始估计（使用您提供的手动矩阵）
#     initial_angles = np.array([0, 0, 0])  # 初始欧拉角
#     initial_t = np.array([-100, -675, -90])  # 初始平移向量
#
#     # 定义误差函数
#     def error_func(params):
#         roll, pitch, yaw, tx, ty, tz = params
#         R = euler_to_rotation_matrix(roll, pitch, yaw)
#         t = np.array([tx, ty, tz])
#
#         errors = []
#         for i in range(len(camera_points)):
#             # 将相机坐标系中的点转换到世界坐标系
#             world_point_est = R @ camera_points[i] + t
#             errors.extend(world_point_est - world_points[i])
#
#         return np.array(errors)
#
#     # 初始参数
#     initial_params = np.concatenate([initial_angles, initial_t])
#
#     # 使用最小二乘法优化参数
#     result = least_squares(error_func, initial_params, verbose=0)
#
#     # 提取优化后的参数
#     optimized_params = result.x
#     roll, pitch, yaw = optimized_params[:3]
#     t = optimized_params[3:6]
#
#     return roll, pitch, yaw, t
#
#
# def correct_camera_orientation(camera_points, roll, pitch, yaw):
#     """
#     校正相机坐标系到水平状态
#     参数:
#         camera_points: 相机坐标系中的点 (Nx3)
#         roll, pitch, yaw: 欧拉角（弧度）
#     返回:
#         corrected_points: 校正后的点 (Nx3)
#     """
#     # 计算校正旋转矩阵（将相机坐标系旋转到水平）
#     R_correction = euler_to_rotation_matrix(roll, pitch, yaw)
#
#     # 应用校正
#     corrected_points = []
#     for point in camera_points:
#         corrected_points.append(R_correction @ point)
#
#     return np.array(corrected_points)
#
#
# # 使用您提供的数据
# camera_points = np.array([
#     [-620, 246, 1277],
#     [507, 246, 1303],
#     [320, 249, 1172],
#     [-717, 237, 1429]
# ])
#
# world_points = np.array([
#     [450, -1932, -699],
#     [450, -1978, 427],
#     [450, -1847, 240],
#     [450, -2064, -817]
# ])
#
# # 估计相机姿态
# yaw=math.pi/2
# pitch=-math.pi/2+0.001
# # roll=0-0.05
# roll=0-0.05
# a=np.array([[0.0, -1.0, 0.0, -100],
# [0.0, 0.0, -1.0, -675],
# [1.0, 0.0, 0.0, -90],
# [0.0, 0.0, 0.0, 1.0]])
# t=a[:3,3]
# print(f"估计的欧拉角 (弧度): roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")
# print(f"估计的平移向量: {t}")
#
# # 将弧度转换为角度
# # roll, pitch, yaw=0,0,0
# roll_deg = np.degrees(roll)
# pitch_deg = np.degrees(pitch)
# yaw_deg = np.degrees(yaw)
# print(f"估计的欧拉角 (度): roll={roll_deg:.2f}, pitch={pitch_deg:.2f}, yaw={yaw_deg:.2f}")
#
# # 校正相机坐标系到水平状态
# corrected_camera_points = correct_camera_orientation(camera_points, roll, pitch, yaw)
#
# print("\n校正后的相机坐标系点:")
# for i, point in enumerate(corrected_camera_points):
#     print(f"原始点 {camera_points[i]} -> 校正点 {point}")
#
# # 计算从校正后的相机坐标系到世界坐标系的变换
# # 由于相机坐标系已经校正到水平，我们可以使用更简单的变换
# R_corrected = np.eye(3)  # 校正后的相机坐标系与世界坐标系对齐
# t_corrected = t  # 平移向量保持不变
#
# print("\n从校正后的相机坐标系到世界坐标系的变换:")
# print("旋转矩阵 R (单位矩阵，因为坐标系已对齐):")
# print(R_corrected)
# print("平移向量 t:")
# print(t_corrected)
#
# # 验证变换结果
# print("\n验证校正后的相机点 -> 世界点的变换:")
# sum_e=0
# for i in range(4):
#     world_point_est = R_corrected @ corrected_camera_points[i] + t_corrected
#     error = np.linalg.norm((world_point_est - world_points[i])[1:])
#     print(
#         f"校正相机点 {corrected_camera_points[i]} -> 世界点 {world_point_est} (原始世界点: {world_points[i]}, 误差: {error:.2f})")
#     sum_e+=error
# print(sum_e)
#
#
#
#
#
# # 示例数据（请替换为实际测量的坐标）
# camera_points= np.array([  # 世界坐标系中的点
#     [ -620,246, 1277],
#     [507,246,1303],
#     [320,249,1172],
#     [-717,237,1429]
# ])
#
# world_points= np.array([  # 相机坐标系中的点
#     [450,-1932,-699],
#     [450,-1978,-427],
#     [450,-1847,240],
#     [450,-2064,-817]
# ])
#
# # | 第一组 | -620 | 246  | 1277 | 450  | -1932 | -699 |
# # | 第二组 | 507  | 246  | 1303 | 450  | -1978 | -427 |
# # | 第三组 | 320  | 249  | 1172 | 349  | -1847 | 240  |
# # | 第四组 | -717 | 237  | 1429 | 450  | -2064 | -817 |
# # 计算变换矩阵
# # 计算从世界坐标系到相机坐标系的变换
#
# a=np.array([[0.0, -1.0, 0.0, -100],
# [0.0, 0.0, -1.0, -675],
# [1.0, 0.0, 0.0, -90],
# [0.0, 0.0, 0.0, 1.0]])
#

# import numpy as np
# from scipy.optimize import least_squares
#
#
# def solve_rigid_transform_improved(src_points, dst_points, initial_R=None, initial_t=None):
#     """
#     使用所有点计算从源点集到目标点集的刚体变换（旋转和平移）
#     参数:
#         src_points: 源坐标系中的点 (Nx3矩阵，每行一个点)
#         dst_points: 目标坐标系中的点 (Nx3矩阵，每行一个点)
#         initial_R: 初始旋转矩阵估计 (可选)
#         initial_t: 初始平移向量估计 (可选)
#     返回:
#         R: 旋转矩阵 (3x3)
#         t: 平移向量 (3x1)
#     """
#     # 转换为numpy数组并确保是浮点类型
#     src_points = np.array(src_points, dtype=np.float64)
#     dst_points = np.array(dst_points, dtype=np.float64)
#
#     n_points = src_points.shape[0]
#
#     # 如果没有提供初始估计，使用SVD方法计算初始估计
#     if initial_R is None or initial_t is None:
#         R, t = solve_rigid_transform(src_points, dst_points)
#     else:
#         R, t = initial_R, initial_t
#
#     # 将旋转矩阵转换为轴角表示（更适合优化）
#     def rotation_matrix_to_axis_angle(R):
#         theta = np.arccos((np.trace(R) - 1) / 2)
#         if theta < 1e-10:  # 处理小角度情况
#             return np.zeros(3)
#         axis = np.array([R[2, 1] - R[1, 2],
#                          R[0, 2] - R[2, 0],
#                          R[1, 0] - R[0, 1]]) / (2 * np.sin(theta))
#         return axis * theta
#
#     # 将轴角转换回旋转矩阵
#     def axis_angle_to_rotation_matrix(axis_angle):
#         theta = np.linalg.norm(axis_angle)
#         if theta < 1e-10:  # 处理小角度情况
#             return np.eye(3)
#         axis = axis_angle / theta
#         cos_theta = np.cos(theta)
#         sin_theta = np.sin(theta)
#
#         # 使用罗德里格斯公式
#         K = np.array([[0, -axis[2], axis[1]],
#                       [axis[2], 0, -axis[0]],
#                       [-axis[1], axis[0], 0]])
#
#         return np.eye(3) + sin_theta * K + (1 - cos_theta) * (K @ K)
#
#     # 初始参数：旋转（轴角表示，3个参数）和平移（3个参数）
#     initial_params = np.concatenate([rotation_matrix_to_axis_angle(R), t])
#
#     # 定义误差函数
#     def error_func(params):
#         axis_angle = params[:3]
#         t_est = params[3:6]
#         R_est = axis_angle_to_rotation_matrix(axis_angle)
#
#         errors = []
#         for i in range(n_points):
#             transformed_point = R_est @ src_points[i] + t_est
#             errors.extend(transformed_point - dst_points[i])
#
#         return np.array(errors)
#
#     # 使用最小二乘法优化参数
#     result = least_squares(error_func, initial_params, verbose=0)
#
#     # 提取优化后的参数
#     optimized_params = result.x
#     axis_angle_opt = optimized_params[:3]
#     t_opt = optimized_params[3:6]
#     R_opt = axis_angle_to_rotation_matrix(axis_angle_opt)
#
#     return R_opt, t_opt
#
#
# # 使用您提供的数据
# camera_points = np.array([
#     [-620, 246, 1277],
#     [507, 246, 1303],
#     [320, 249, 1172],
#     [-717, 237, 1429]
# ])
#
# world_points = np.array([
#     [450, -1932, -699],
#     [450, -1978, 427],
#     [450, -1847, 240],  # 注意：第三组的世界点x坐标是349，不是450
#     [450, -2064, -817]
# ])
#
# # 使用您提供的手动矩阵作为初始估计
# manual_R = np.array([
#     [0.0, -1.0, 0.0],
#     [0.0, 0.0, -1.0],
#     [1.0, 0.0, 0.0]
# ])
# manual_t = np.array([-100, -675, -90])
#
# # 使用改进的方法计算变换矩阵
# R_world_to_cam, t_world_to_cam = solve_rigid_transform_improved(
#     world_points, camera_points, manual_R, manual_t
# )
#
# print("优化后的世界坐标系 -> 相机坐标系的变换:")
# print("旋转矩阵 R:")
# print(R_world_to_cam)
# print("平移向量 t:")
# print(t_world_to_cam)
#
# # 计算逆变换（从相机坐标系到世界坐标系）
# R_cam_to_world = R_world_to_cam.T
# t_cam_to_world = -R_cam_to_world @ t_world_to_cam
#
# print("\n优化后的相机坐标系 -> 世界坐标系的变换:")
# print("旋转矩阵 R_inv:")
# print(R_cam_to_world)
# print("平移向量 t_inv:")
# print(t_cam_to_world)
#
# # 验证变换结果
# print("\n验证相机点 -> 世界点的变换:")
# total_er=0
# for i in range(4):
#     # 从相机坐标转换到世界坐标
#     world_point = R_cam_to_world @ camera_points[i] + t_cam_to_world
#     error = np.linalg.norm((world_point - world_points[i])[1:])
#     print(f"相机点 {camera_points[i]} -> 世界点 {world_point} (原始世界点: {world_points[i]}, 误差: {error:.2f})")
#     total_er+=error
# print(total_er)
#
# print("\n验证世界点 -> 相机点的变换:")
# for i in range(4):
#     # 从世界坐标转换到相机坐标
#     camera_point = R_world_to_cam @ world_points[i] + t_world_to_cam
#     error = np.linalg.norm(camera_point - camera_points[i])
#     print(f"世界点 {world_points[i]} -> 相机点 {camera_point} (原始相机点: {camera_points[i]}, 误差: {error:.2f})")



# 右侧相机
# import numpy as np
# from scipy.optimize import least_squares
#
#
# def solve_rigid_transform_improved(src_points, dst_points, initial_R=None, initial_t=None):
#     """
#     使用所有点计算从源点集到目标点集的刚体变换（旋转和平移）
#     参数:
#         src_points: 源坐标系中的点 (Nx3矩阵，每行一个点)
#         dst_points: 目标坐标系中的点 (Nx3矩阵，每行一个点)
#         initial_R: 初始旋转矩阵估计 (可选)
#         initial_t: 初始平移向量估计 (可选)
#     返回:
#         R: 旋转矩阵 (3x3)
#         t: 平移向量 (3x1)
#     """
#     # 转换为numpy数组并确保是浮点类型
#     src_points = np.array(src_points, dtype=np.float64)
#     dst_points = np.array(dst_points, dtype=np.float64)
#
#     n_points = src_points.shape[0]
#
#     # 如果没有提供初始估计，使用SVD方法计算初始估计
#     if initial_R is None or initial_t is None:
#         R, t = solve_rigid_transform(src_points, dst_points)
#     else:
#         R, t = initial_R, initial_t
#
#     # 将旋转矩阵转换为轴角表示（更适合优化）
#     def rotation_matrix_to_axis_angle(R):
#         theta = np.arccos((np.trace(R) - 1) / 2)
#         if theta < 1e-10:  # 处理小角度情况
#             return np.zeros(3)
#         axis = np.array([R[2, 1] - R[1, 2],
#                          R[0, 2] - R[2, 0],
#                          R[1, 0] - R[0, 1]]) / (2 * np.sin(theta))
#         return axis * theta
#
#     # 将轴角转换回旋转矩阵
#     def axis_angle_to_rotation_matrix(axis_angle):
#         theta = np.linalg.norm(axis_angle)
#         if theta < 1e-10:  # 处理小角度情况
#             return np.eye(3)
#         axis = axis_angle / theta
#         cos_theta = np.cos(theta)
#         sin_theta = np.sin(theta)
#
#         # 使用罗德里格斯公式
#         K = np.array([[0, -axis[2], axis[1]],
#                       [axis[2], 0, -axis[0]],
#                       [-axis[1], axis[0], 0]])
#
#         return np.eye(3) + sin_theta * K + (1 - cos_theta) * (K @ K)
#
#     # 初始参数：旋转（轴角表示，3个参数）和平移（3个参数）
#     initial_params = np.concatenate([rotation_matrix_to_axis_angle(R), t])
#
#     # 定义误差函数
#     def error_func(params):
#         axis_angle = params[:3]
#         t_est = params[3:6]
#         R_est = axis_angle_to_rotation_matrix(axis_angle)
#
#         errors = []
#         for i in range(n_points):
#             transformed_point = R_est @ src_points[i] + t_est
#             errors.extend(transformed_point - dst_points[i])
#
#         return np.array(errors)
#
#     # 使用最小二乘法优化参数
#     result = least_squares(error_func, initial_params, verbose=0)
#
#     # 提取优化后的参数
#     optimized_params = result.x
#     axis_angle_opt = optimized_params[:3]
#     t_opt = optimized_params[3:6]
#     R_opt = axis_angle_to_rotation_matrix(axis_angle_opt)
#
#     return R_opt, t_opt
#
#
# # 使用您提供的数据
# camera_points = np.array([
#     [-629, 238, 1214],
#     [32, 282, 1427],
#     [709, 246, 1073],
#     [647, 269, 1415],
#     [57, 261, 1232],
#     [-614, 257, 1244],
#     [-655, 252, 1379],
#     [-58, 258, 1249]
# ])
#
# # -629.2433739	238.1799817085266	1213.8837575912476
# # 32.44228661060333	282.13897347450256	1426.575779914856
# # 708.7962031364441	245.7047700881958	1073.33505153656
# # 647.3563313484192	269.013375043869	1414.9551391601562
# # 57.54637345671654	261.46918535232544	1232.0201396942139
# # -614.5479679	257.0344805717468	1243.565320968628
# # -654.6847224	251.5895664691925	1378.697395324707
# # -57.53321573	258.1462264060974	1249.2554187774658
#
#
#
#
# world_points = np.array([
#     [450, 2000, 600],
#     [450, 2100, -100],
#     [450, 1800, -800],
#     [450, 2050, 700],
#     [450, 1900, -120],
#     [450, 1950, 550],
#     [450, 2090, 580],
#     [450, 1950, 0]
# ])
#
# # 使用您提供的手动矩阵作为初始估计
# manual_R = np.array([
#     [0.00142069, -0.99914183, -0.04139544],
#     [-0.02943357, 0.04133576, 0.99871168],
#     [-0.99956573, 0.00263728, -0.02934959]
# ])
#
# manual_t = np.array([748, 675, 67])
#
#
#
#
# # 使用改进的方法计算变换矩阵
# R_world_to_cam, t_world_to_cam = solve_rigid_transform_improved(
#     world_points, camera_points, manual_R, manual_t
# )
#
# print("优化后的世界坐标系 -> 相机坐标系的变换:")
# print("旋转矩阵 R:")
# print(R_world_to_cam)
# print("平移向量 t:")
# print(t_world_to_cam)
#
# # 计算逆变换（从相机坐标系到世界坐标系）
# R_cam_to_world = R_world_to_cam.T
# t_cam_to_world = -R_cam_to_world @ t_world_to_cam
#
# print("\n优化后的相机坐标系 -> 世界坐标系的变换:")
# print("旋转矩阵 R_inv:")
# print(R_cam_to_world)
# print("平移向量 t_inv:")
# print(t_cam_to_world)
#
# # 验证变换结果
# print("\n验证相机点 -> 世界点的变换:")
# total_er=0
# for i in range(4):
#     # 从相机坐标转换到世界坐标
#     world_point = R_cam_to_world @ camera_points[i] + t_cam_to_world
#     error = np.linalg.norm((world_point - world_points[i])[1:])
#     print(f"相机点 {camera_points[i]} -> 世界点 {world_point} (原始世界点: {world_points[i]}, 误差: {error:.2f})")
#     total_er+=error
# print(total_er)
#
# print("\n验证世界点 -> 相机点的变换:")
# for i in range(4):
#     # 从世界坐标转换到相机坐标
#     camera_point = R_world_to_cam @ world_points[i] + t_world_to_cam
#     error = np.linalg.norm(camera_point - camera_points[i])
#     print(f"世界点 {world_points[i]} -> 相机点 {camera_point} (原始相机点: {camera_points[i]}, 误差: {error:.2f})")
#


import numpy as np
from scipy.optimize import least_squares


def solve_rigid_transform(src_points, dst_points):
    """
    使用 SVD 求解刚体变换 R, t 使得: dst = R * src + t
    """
    src_points = np.array(src_points, dtype=np.float64)
    dst_points = np.array(dst_points, dtype=np.float64)

    centroid_src = np.mean(src_points, axis=0)
    centroid_dst = np.mean(dst_points, axis=0)

    src_centered = src_points - centroid_src
    dst_centered = dst_points - centroid_dst

    H = src_centered.T @ dst_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 防止出现反射矩阵
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_dst - R @ centroid_src
    return R, t


def solve_rigid_transform_improved(src_points, dst_points, initial_R=None, initial_t=None):
    """
    改进的刚体变换求解（加入非线性最小二乘优化）
    """
    src_points = np.array(src_points, dtype=np.float64)
    dst_points = np.array(dst_points, dtype=np.float64)
    n_points = src_points.shape[0]

    # 初始估计（若无提供）
    if initial_R is None or initial_t is None:
        R, t = solve_rigid_transform(src_points, dst_points)
    else:
        R, t = initial_R, initial_t

    # rotation <-> axis-angle
    def rotation_matrix_to_axis_angle(R):
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
        if theta < 1e-10:
            return np.zeros(3)
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2 * np.sin(theta))
        return axis * theta

    def axis_angle_to_rotation_matrix(axis_angle):
        theta = np.linalg.norm(axis_angle)
        if theta < 1e-10:
            return np.eye(3)
        axis = axis_angle / theta
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

    initial_params = np.concatenate([rotation_matrix_to_axis_angle(R), t])

    def error_func(params):
        axis_angle = params[:3]
        t_est = params[3:6]
        R_est = axis_angle_to_rotation_matrix(axis_angle)
        transformed = (R_est @ src_points.T).T + t_est
        return (transformed - dst_points).ravel()

    result = least_squares(error_func, initial_params, verbose=0)
    axis_angle_opt = result.x[:3]
    t_opt = result.x[3:6]
    R_opt = axis_angle_to_rotation_matrix(axis_angle_opt)

    return R_opt, t_opt


# ========== 数据部分 ==========
camera_points = np.array([
    [32, 282, 1427],
    [709, 246, 1073],
    [57, 261, 1232],
    [-614, 257, 1244],
    [-655, 252, 1379],
    [-58, 258, 1249]
])

world_points = np.array([
    [450, 2100, -100],
    [450, 1800, -800],
    [450, 1900, -120],
    [450, 1950, 550],
    [450, 2090, 580],
    [450, 1950, 0]
])

# 手动初始矩阵
manual_R = np.array([
    [0.00142069, -0.99914183, -0.04139544],
    [-0.02943357, 0.04133576, 0.99871168],
    [-0.99956573, 0.00263728, -0.02934959]
])
manual_t = np.array([748, 675, 67])

# 求解优化后的变换矩阵
R_world_to_cam, t_world_to_cam = solve_rigid_transform_improved(
    world_points, camera_points, manual_R, manual_t
)

# 计算逆变换
R_cam_to_world = R_world_to_cam.T
t_cam_to_world = -R_cam_to_world @ t_world_to_cam

print("优化后的世界坐标系 -> 相机坐标系的变换:")
print("旋转矩阵 R:")
print(R_world_to_cam)
print("平移向量 t:")
print(t_world_to_cam)

print("\n优化后的相机坐标系 -> 世界坐标系的变换:")
print("旋转矩阵 R_inv:")
print(R_cam_to_world)
print("平移向量 t_inv:")
print(t_cam_to_world)

# ========== 验证部分 ==========
print("\n验证相机点 -> 世界点的变换:")
total_error_cam2world = 0
for i in range(len(camera_points)):
    world_pred = R_cam_to_world @ camera_points[i] + t_cam_to_world
    error = np.linalg.norm(world_pred - world_points[i])
    total_error_cam2world += error
    print(f"{i+1}. 相机点 {camera_points[i]} -> 世界点 {world_pred} "
          f"(原始: {world_points[i]}, 误差: {error:.2f})")

print(f"平均误差: {total_error_cam2world / len(camera_points):.3f}\n")

print("验证世界点 -> 相机点的变换:")
total_error_world2cam = 0
for i in range(len(world_points)):
    cam_pred = R_world_to_cam @ world_points[i] + t_world_to_cam
    error = np.linalg.norm(cam_pred - camera_points[i])
    total_error_world2cam += error
    print(f"{i+1}. 世界点 {world_points[i]} -> 相机点 {cam_pred} "
          f"(原始: {camera_points[i]}, 误差: {error:.2f})")

print(f"平均误差: {total_error_world2cam / len(world_points):.3f}")
