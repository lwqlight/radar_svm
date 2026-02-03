# config.py
import os

# --- 硬件配置 ---
SERIAL_PORT = '/dev/ttyACM0'  # 串口号
BAUD_RATE = 115200

# --- 文件路径配置 ---
DATA_DIR = "./data"
WEIGHTS_DIR = "./weights"
CSV_PATH = os.path.join(DATA_DIR, "radar_training_data.csv")
MODEL_PATH = os.path.join(WEIGHTS_DIR, "radar_svm_model.pkl")
SCALER_PATH = os.path.join(WEIGHTS_DIR, "radar_scaler.pkl")

# --- 采集配置 ---
# 【关键】这里改采集帧数，500帧约等于50秒数据，足够丰富
COLLECT_NUM_FRAMES = 500  
COLLECT_DELAY = 0.05      # 采样间隔 (秒)

# --- 算法/推理配置 ---
# 迟滞滤波阈值：连续多少帧一致才切换状态？
# 越大越稳，越小越灵敏。建议 6-10
FILTER_THRESHOLD = 8      

# 跌倒检测阈值 (紧急事件不需要等待那么久)
FALL_CONFIRM_FRAMES = 3   

# 标签定义
LABEL_MAP = {
    0: "Wait...",
    1: "🟢 站立 (Standing)",
    2: "🟡 坐下 (Sitting)",
    3: "🚨 跌倒 (FALL)"
}