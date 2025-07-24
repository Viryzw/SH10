# PID 控制类
class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp    # 比例系数
        self.Ki = Ki    # 积分系数
        self.Kd = Kd    # 微分系数
        self.dt = dt    # 采样周期
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, error):
        """
        :param error: 当前误差（期望值 - 实际值）
        :return: PID 控制输出值
        """
        
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output
        
    def clear(self):
        self.previous_error = 0.0
        self.integral = 0.0