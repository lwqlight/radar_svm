# config.py
import os

# --- 硬件串口配置 ---
# 请根据实际情况修改:
# 蓝牙模块
REMOTE_PORT = '/dev/ttyACM1'  
REMOTE_BAUD = 9600

# 雷达通常是 /dev/ttyACM1
RADAR_PORT = '/dev/ttyACM0'   
RADAR_BAUD = 115200

# --- 身份验证配置 (已修正) ---
# 您的专属遥控器 ID: E7 6B 42 43
# 只有匹配这个 ID 的指令才会被执行，防止被别人干扰
TARGET_REMOTE_ID = [0xE7, 0x6B, 0x42, 0x43]


# --- 数据存储 ---
DATA_DIR = "./data"
CSV_PATH = os.path.join(DATA_DIR, "radar_10dim_dataset.csv")
COLLECT_NUM_FRAMES = 500  # 每次按键录制的帧数
# 0.1 = 10Hz (500帧需50秒) -> 太慢
# 0.05 = 20Hz (500帧需25秒) -> 推荐
# 0.02 = 50Hz (500帧需10秒) -> 极速 (需确保雷达串口不拥堵)
COLLECT_DELAY = 0.05

# --- 标签定义 ---
# 遥控器按键值 (Hex) -> 标签 ID 的映射
KEY_MAPPING = {
    0x25: 0, # 空闲 (Idle)
    0x26: 1, # 站立 (Standing)
    0x27: 2, # 坐下 (Sitting)
    0x28: 3  # 躺卧 (Lying)
}

LABEL_NAMES = {
    0: "⚪ 空闲 (Idle)",
    1: "🟢 站立 (Standing)",
    2: "🟡 坐下 (Sitting)",
    3: "🔵 躺卧 (Lying)"
}