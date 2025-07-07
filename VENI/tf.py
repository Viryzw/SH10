import numpy as np
import cv2, math

class CoordinateTransformer:
    def __init__(self, camera_matrix, dist_coeffs, R_cam_to_body):
        """
        camera_matrix: 3x3 np.array, 相机内参矩阵
        dist_coeffs: 1D np.array, 畸变系数
        R_cam_to_body: 3x3 np.array， 相机坐标系到机体坐标系的旋转矩阵，默认单位矩阵
        roll, pitch, yaw: 机体姿态角（弧度），用于世界坐标系变换
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.R_cam_to_body = R_cam_to_body
        self.relative_coord = []

    def euler_to_rot_matrix(self, roll, pitch, yaw):
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        R_x = np.array([[1, 0, 0],
                        [0, cr, -sr],
                        [0, sr, cr]])

        R_y = np.array([[cp, 0, sp],
                        [0, 1, 0],
                        [-sp, 0, cp]])

        R_z = np.array([[cy, -sy, 0],
                        [sy, cy, 0],
                        [0, 0, 1]])

        return R_z @ R_y @ R_x

    def pixel_to_world(self, pixel_point, x, y, z, roll, pitch, yaw):
        """
        pixel_point: (x, y) 像素坐标
        x, y, z 为gazebo原点系坐标
        
        返回世界坐标系坐标 (3,)
        """
        # 转换为 numpy 数组并reshape成(1,1,2) float32
        pixel_points = np.array([[pixel_point]], dtype=np.float32)
        now_point = np.array([x, y, z], dtype=np.float32)

        # 去畸变，得到归一化相机坐标
        undistorted_points = cv2.undistortPoints(pixel_points, self.camera_matrix, self.dist_coeffs)
        x_norm = undistorted_points[0,0,0]
        y_norm = undistorted_points[0,0,1]

        # 相机坐标系坐标
        point_cam = np.array([x_norm, y_norm, 1])  # shape (3,)

        # 转到机体坐标系
        point_body = self.R_cam_to_body @ point_cam
        
        # 转到世界坐标系
        point_world = self.euler_to_rot_matrix(roll, pitch, yaw) @ point_body
        
        # 深度处理
        z = -1 / point_world[2] * z
        point_world_depth = z * point_world
        
        # 平移处理
        self.relative_coord = point_world_depth
        point_world_final = point_world_depth + now_point

        return point_world_final

def vision_yaw(cx, cy):
    if cx * cy > 0:
        return math.atan2(abs(cx), abs(cy))
    else:
        return math.atan2(-1*abs(cx), abs(cy))