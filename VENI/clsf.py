import math
import numpy as np
from scipy.optimize import least_squares


# 拟合类
class CircleLeastSquaresFilter:
    def __init__(self, iter):
        self.center = None  # (a, b)
        self.radius = None  # R
        self.iter = iter
    
    # 基础残差函数
    def _residuals_basic(self, params, x_data, y_data):
        a, b, R = params
        x_data = np.asarray(x_data).flatten()
        y_data = np.asarray(y_data).flatten()
        distances = np.sqrt((x_data - a)**2 + (y_data - b)**2)
        return (distances - R).flatten()

    # 拟合优化
    def _residuals_x_consistency(self, center_params, x_data, y_data, fixed_radius):
        a, b = center_params
        residuals = []
        for x, y in zip(x_data, y_data):
            dx = x - a
            dy = y - b
            theta = np.arctan2(dy, dx)
            pred_x = a + fixed_radius * np.cos(theta)
            residuals.append(pred_x - x)
        return np.array(residuals).flatten()

    # 先对输入点进行全参数拟合，再优化
    def fit(self, positions, dt=0.1, tol=1e-3):
        x = np.asarray(positions[:, 0]).flatten()
        y = np.asarray(positions[:, 1]).flatten()

        # 先用无约束拟合求解初始参数
        a0 = np.mean(x)
        b0 = np.mean(y)
        R0 = np.mean(np.sqrt((x - a0)**2 + (y - b0)**2))
        initial_params = [a0, b0, R0]

        # 初步拟合
        result = least_squares(self._residuals_basic, initial_params, args=(x, y))
        a, b, R = result.x

        # 迭代优化
        center_params = np.array([a, b])
        for i in range(self.iter):
            res = least_squares(self._residuals_x_consistency, center_params, args=(x, y, R))
            new_center = res.x
            diff = np.linalg.norm(new_center - center_params)
            center_params = new_center
            if diff < tol:
                break

        self.center = center_params
        self.radius = R

    # 对一组点进行拟合预测
    def predict(self, positions):
        a, b = self.center
        R = self.radius
        pred_points = []
        for px, py in positions:
            dx = px - a
            dy = py - b
            theta = np.arctan2(dy, dx)
            pred_x = a + R * np.cos(theta)
            pred_y = b + R * np.sin(theta)
            pred_points.append([pred_x, pred_y])
        return np.array(pred_points)
    
    # 对单点进行拟合预测
    def predict_single(self, x, y):
        a, b = self.center
        R = self.radius
        dx = x - a
        dy = y - b

        theta = np.arctan2(math.sqrt(R**2 + dx**2), dx)
        pred_x = a + R * np.cos(theta)
        pred_y = b + R * np.sin(theta)
        return x, pred_y


# 用于管理点的添加、拟合、预测等操作
class ClsfManager:
    def __init__(self, max_points=None, iter=100, max_fit_point=3):
        self.max_points = max_points
        self.positions = []
        self.filter = CircleLeastSquaresFilter(iter=iter)
        self.fitted = False
        self.max_fit_point = max_fit_point

    # 添加一个点并触发拟合或预测逻辑
    def add_point(self, x, y):
        self.positions.append([x, y])

        # 如果不设 max_points，只做拟合并返回 center 和 radius
        if self.max_points is None:
            if len(self.positions) > self.max_fit_point:
                self._fit()
                return self.filter.center, self.filter.radius
            else:
                return None
            
        # 如果设了 max_points，则控制点数并执行拟合或预测
        if self.max_points is not None:
            if len(self.positions) == self.max_points:
                self._fit()
                return "FITTED"
            elif len(self.positions) > self.max_points:
                return self.predict(x, y)
            else:
                return None

    # 清除所有记录并重置状态
    def clear(self):
        self.positions = []
        self.filter.center = None
        self.filter.radius = None
        self.fitted = False

    # 内部函数：执行拟合流程
    def _fit(self):
        data = np.array(self.positions[:self.max_points] if self.max_points else self.positions)
        self.filter.fit(data)
        self.fitted = True

    # 对一组点进行预测
    def predict_all(self, positions):
        if not self.fitted:
            return None
        return self.filter.predict(positions)

    # 对单点进行预测（返回校正后的位置）
    def predict(self, x, y):
        if not self.fitted:
            return None
        return self.filter.predict_single(x, y)