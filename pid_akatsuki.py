class ProportionalPID:
    def __init__(self, kp=0.5, ki=0.1, kd=0.2, deadband=0.1, max_output=127):
        """
        初始化PID控制器参数
        
        参数:
        kp - 比例系数
        ki - 积分系数
        kd - 微分系数
        deadband - 死区范围，在目标位置±deadband内不做调整
        max_output - 输出的最大值，对应遥控器的最大偏移量
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadband = deadband
        self.max_output = max_output
        
        # 初始化内部状态
        self.error_sum = 0  # 积分项累加值
        self.last_error = 0  # 上一次的误差值
        self.last_output = 127  # 默认输出值，对应遥控器的中立位置
    
    def update(self, current:float, target=0.0):
        """
        根据当前位置和目标位置更新PID控制器并计算输出
        
        参数:
        current_x - 当前识别到的物体x坐标（左正右负）
        target_x - 目标x坐标，默认为0（车体正前方）
        
        返回:
        对应遥控器的输出值（0-255）
        """
        print("pid controller working")
        current = -current
        # 计算误差
        error = target - current
        
        # 检查是否在死区内
        if abs(error) <= self.deadband:
            self.error_sum = 0  # 清除积分项
            print("\033[031mpid controller working\033[0m")
            return 127  # 在死区内，不做调整
        
        # 计算PID各项
        p_term = self.kp * error
        
        # 积分项计算与限制
        self.error_sum += error
        i_term = self.ki * self.error_sum
        
        # 微分项计算
        d_term = self.kd * (error - self.last_error)
        self.last_error = error
        
        # 计算PID总输出
        pid_output = p_term + i_term + d_term
        
        # 将PID输出映射到遥控器范围（0-255）
        # 注意：输出为负时表示向右，对应遥控器值减小；输出为正时表示向左，对应遥控器值增加
        output = 127 - pid_output  # 基础值减去PID输出
        
        # 限制输出范围
        if output < 0:
            output = 0
        elif output > 255:
            output = 255
            
        self.last_output = output
        return output
    
    def reset(self):
        """重置PID控制器的内部状态"""
        self.error_sum = 0
        self.last_error = 0
        self.last_output = 127