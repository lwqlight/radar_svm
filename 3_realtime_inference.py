import serial
import struct
import threading
import time
import joblib
import pandas as pd
from collections import deque, Counter
import feature_extractor # 导入同一个特征提取器

# --- 配置 ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
MODEL_FILE = './weights/radar_svm_model.pkl'
SCALER_FILE = './weights/radar_scaler.pkl'
HISTORY_LEN = 8  # 滤波窗口，设为 8 会更稳

# 加载模型
print("加载模型...")
try:
    clf = joblib.load(MODEL_FILE)
    scaler = joblib.load(SCALER_FILE)
except:
    print("❌ 请先重新采集数据并训练！")
    exit()

LABEL_MAP = {0: "Wait...", 1: "🟢 站立", 2: "🟡 坐下", 3: "🚨 跌倒"}

# 全局数据
current_target = {'z': 0.0, 'speed': 0.0}
current_points = []
data_lock = threading.Lock()
stop_flag = False

# ... (calc_checksum, send_cmd, parse_data 函数完全复用上面的采集脚本逻辑) ...
# ... 为了节省篇幅，请直接把 parse_data 复制过来 ...
def calc_checksum(data_bytes):
    checksum = 0
    for b in data_bytes: checksum ^= b
    return (~checksum) & 0xFF

def send_cmd(ser, cmd_val):
    header = b'\x01\x00\x01\x00\x04\x02\x01'
    h_cs = calc_checksum(header)
    payload = struct.pack('<I', cmd_val)
    d_cs = calc_checksum(payload)
    return header + bytes([h_cs]) + payload + bytes([d_cs])

def parse_data(ser):
    global current_target, current_points
    buffer = b""
    while not stop_flag:
        if ser.in_waiting: buffer += ser.read(ser.in_waiting)
        while len(buffer) >= 8:
            if buffer[0] != 0x01:
                buffer = buffer[1:]
                continue
            try:
                header = buffer[0:7]
                if calc_checksum(header) != buffer[7]:
                    buffer = buffer[1:]
                    continue
                _, _, data_len, frame_type = struct.unpack('>BHHH', header)
                total_len = 8 + data_len + 1
                if len(buffer) < total_len: break
                
                payload = buffer[8:8+data_len]
                if calc_checksum(payload) == buffer[8+data_len]:
                    if frame_type == 0x0A04 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        if num > 0:
                            _, _, z, dop_idx, _ = struct.unpack('<fffii', payload[4:24])
                            with data_lock:
                                current_target['z'] = z
                                current_target['speed'] = float(dop_idx)
                    elif frame_type == 0x0A08 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        temp_points = []
                        offset = 4
                        for _ in range(num):
                            if offset+20 > len(payload): break
                            _, x, y, z, speed = struct.unpack('<iffff', payload[offset:offset+20])
                            if abs(x) < 4.0 and 0.1 < y < 6.0:
                                temp_points.append((x, y, speed))
                            offset += 20
                        with data_lock:
                            current_points = temp_points
                buffer = buffer[total_len:]
            except:
                buffer = buffer[1:]

def inference_loop():
    history = deque(maxlen=HISTORY_LEN)
    last_status = -1
    print("\n🚀 开始 10维 特征融合识别...\n")
    
    while not stop_flag:
        time.sleep(0.1)
        
        with data_lock:
            # --- 核心修改：使用统一的特征提取器 ---
            raw_feats = feature_extractor.extract_features(current_target, current_points)
        
        try:
            # 转为 DataFrame 以匹配训练时的格式
            input_df = pd.DataFrame([raw_feats], columns=feature_extractor.FEATURE_NAMES)
            
            # 预处理 + 推理
            scaled = scaler.transform(input_df)
            pred = clf.predict(scaled)[0]
            
            # 滤波
            history.append(pred)
            if len(history) >= HISTORY_LEN // 2:
                final_pred = Counter(history).most_common(1)[0][0]
                
                if final_pred != last_status:
                    print(f"[{time.strftime('%H:%M:%S')}] 切换 -> {LABEL_MAP.get(final_pred, str(final_pred))}")
                    last_status = final_pred
                elif final_pred == 3:
                    print(f"\r🚨 跌倒检测! Z={raw_feats[0]:.2f}m Ratio={raw_feats[5]:.2f}", end="")
                    
        except Exception:
            pass

if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.write(send_cmd(ser, 0x14)); time.sleep(0.1)
        ser.write(send_cmd(ser, 0x08)); time.sleep(0.1)
        ser.write(send_cmd(ser, 0x06)); time.sleep(0.1)
        ser.reset_input_buffer()
        
        t = threading.Thread(target=parse_data, args=(ser,), daemon=True)
        t.start()
        inference_loop()
    except KeyboardInterrupt:
        stop_flag = True