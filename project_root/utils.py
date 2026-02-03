# utils.py
class HysteresisFilter:
    """
    状态机迟滞滤波器：防止姿态识别乱跳
    """
    def __init__(self, threshold=8, fall_threshold=3):
        self.current_state = 0   # 当前稳定状态
        self.pending_state = 0   # 待确认的新状态
        self.counter = 0         # 计数器
        self.threshold = threshold
        self.fall_threshold = fall_threshold

    def update(self, new_pred):
        # 跌倒检测特殊处理：如果是跌倒(3)，使用更短的确认时间
        target_threshold = self.fall_threshold if new_pred == 3 else self.threshold

        if new_pred == self.pending_state:
            # 如果新预测值 == 待确认值，计数器+1
            self.counter += 1
        else:
            # 如果新预测值变了，重置计数器，开始重新累计
            self.pending_state = new_pred
            self.counter = 1

        # 检查是否满足切换条件
        if self.counter >= target_threshold:
            self.current_state = self.pending_state
            # 达到目标后，计数器可以重置或保持，这里保持以防抖动
            self.counter = 0 
            
        return self.current_state