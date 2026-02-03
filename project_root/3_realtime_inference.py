import serial
import struct
import threading
import time
import joblib
import pandas as pd
import config            # 导入配置
import feature_extractor # 导入特征提取
from utils import HysteresisFilter # 导入滤波器

# 全局变量
current_target = {'z': 0.0, 'speed': 0.0}
current_points = []
data_lock = threading.Lock()
stop_flag = False

# --- 1. 修正校验算法 (必须是 XOR) ---
def calc_checksum(data):
    checksum = 0
    for b in data:
        checksum ^= b
    return (~checksum) & 0xFF

def send_cmd(ser, cmd_val):
    header = b'\x01\x00\x01\x00\x04\x02\x01'
    # 计算头校验
    h_cs = calc_checksum(header)
    # 负载
    payload = struct.pack('<I', cmd_val)
    # 计算数据校验
    d_cs = calc_checksum(payload)
    return header + bytes([h_cs]) + payload + bytes([d_cs])

# --- 2. 完整的解析逻辑 (不能省略) ---
def parse_data(ser):
    global current_target, current_points
    buffer = b""
    print("DEBUG: 数据接收线程已启动...")
    
    while not stop_flag:
        try:
            if ser.in_waiting: 
                buffer += ser.read(ser.in_waiting)
            
            while len(buffer) >= 8:
                if buffer[0] != 0x01:
                    buffer = buffer[1:]
                    continue
                
                # 校验头
                header = buffer[0:7]
                if calc_checksum(header) != buffer[7]:
                    buffer = buffer[1:]
                    continue
                
                _, _, data_len, frame_type = struct.unpack('>BHHH', header)
                total_len = 8 + data_len + 1
                
                if len(buffer) < total_len: break
                
                payload = buffer[8 : 8+data_len]
                # 校验数据
                if calc_checksum(payload) == buffer[8+data_len]:
                    # 解析目标 (0x0A04)
                    if frame_type == 0x0A04 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        if num > 0 and len(payload) >= 24:
                            _, _, z, dop_idx, _ = struct.unpack('<fffii', payload[4:24])
                            with data_lock:
                                current_target['z'] = z
                                current_target['speed'] = float(dop_idx)
                                
                    # 解析点云 (0x0A08)
                    elif frame_type == 0x0A08 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        temp = []
                        off = 4
                        for _ in range(num):
                            if off + 20 > len(payload): break
                            _, x, y, z, s = struct.unpack('<iffff', payload[off:off+20])
                            if abs(x) < 4.0 and 0.1 < y < 6.0:
                                temp.append((x, y, s))
                            off += 20
                        with data_lock:
                            current_points = temp
                            
                buffer = buffer[total_len:]
        except Exception:
            buffer = buffer[1:]

def inference_loop():
    print(f"正在加载模型: {config.MODEL_PATH} ...")
    try:
        clf = joblib.load(config.MODEL_PATH)
        scaler = joblib.load(config.SCALER_PATH)
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return

    # 初始化滤波器
    hysteresis = HysteresisFilter(
        threshold=config.FILTER_THRESHOLD, 
        fall_threshold=config.FALL_CONFIRM_FRAMES
    )
    
    last_status = -1
    print("\n🚀 开始实时推理 (Ctrl+C 停止)...")
    print("等待数据流稳定...")
    
    while not stop_flag:
        time.sleep(0.1) # 10Hz 推理
        
        # 1. 提取特征
        with data_lock:
            feats = feature_extractor.extract_features(current_target, current_points)
        
        # --- 3. 增加调试监控 ---
        # 如果 Z=0 且 点云数=0，说明数据没进来，打印个提示
        if feats[0] == 0 and feats[8] == 0:
            # print("\r等待有效数据...", end="") # 如果觉得刷屏烦可以注释掉
            pass
        else:
            # 只有当有数据时才进行推理，节省资源
            try:
                # 2. 预处理
                input_df = pd.DataFrame([feats], columns=feature_extractor.FEATURE_NAMES)
                scaled = scaler.transform(input_df)
                
                # 3. 预测
                raw_pred = clf.predict(scaled)[0]
                
                # 4. 滤波
                stable_pred = hysteresis.update(raw_pred)
                
                # 5. 显示
                if stable_pred != last_status:
                    timestamp = time.strftime("%H:%M:%S")
                    status_str = config.LABEL_MAP.get(stable_pred, f"Unknown({stable_pred})")
                    
                    # 打印切换信息
                    print(f"[{timestamp}] 状态切换 -> {status_str}")
                    
                    # 调试：打印一下当前的特征，方便你看模型是根据什么判的
                    print(f"   (特征: Z={feats[0]:.2f}, 宽深比={feats[5]:.2f}, 点数={feats[8]})")
                    
                    last_status = stable_pred
                
                # 跌倒报警
                if stable_pred == 3:
                     print(f"\r! Z={feats[0]:.2f}m", end="")

            except Exception as e:
                print(f"推理错误: {e}")

if __name__ == "__main__":
    try:
        print(f"打开串口 {config.SERIAL_PORT}...")
        ser = serial.Serial(config.SERIAL_PORT, config.BAUD_RATE, timeout=0.1)
        
        # 多发几次初始化，确保唤醒
        for i in range(2):
            for cmd in [0x14, 0x08, 0x06]:
                ser.write(send_cmd(ser, cmd))
                time.sleep(0.1)
        ser.reset_input_buffer()
        
        t = threading.Thread(target=parse_data, args=(ser,), daemon=True)
        t.start()
        
        inference_loop()
        
    except KeyboardInterrupt:
        stop_flag = True
        print("\n程序已停止")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")