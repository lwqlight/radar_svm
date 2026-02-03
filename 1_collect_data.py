import serial
import struct
import threading
import time
import csv
from collections import deque
import feature_extractor  # 导入刚才写好的特征提取器

# --- 配置 ---
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200
CSV_FILENAME = "radar_training_data.csv"

# 全局数据容器
current_target = {'z': 0.0, 'speed': 0.0}
current_points = [] # 存储一帧内的点云
data_lock = threading.Lock()
stop_flag = False
current_label = 0

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
                    
                    # 1. 目标信息 (0x0A04)
                    if frame_type == 0x0A04 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        if num > 0:
                            _, _, z, dop_idx, _ = struct.unpack('<fffii', payload[4:24])
                            with data_lock:
                                current_target['z'] = z
                                current_target['speed'] = float(dop_idx)

                    # 2. 点云信息 (0x0A08)
                    elif frame_type == 0x0A08 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        temp_points = []
                        offset = 4
                        for _ in range(num):
                            if offset+20 > len(payload): break
                            _, x, y, z, speed = struct.unpack('<iffff', payload[offset:offset+20])
                            # 简单清洗
                            if abs(x) < 4.0 and 0.1 < y < 6.0:
                                temp_points.append((x, y, speed))
                            offset += 20
                        
                        with data_lock:
                            current_points = temp_points

                buffer = buffer[total_len:]
            except:
                buffer = buffer[1:]

if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print("初始化雷达...")
        ser.write(send_cmd(ser, 0x14)) # 侧装
        time.sleep(0.1)
        ser.write(send_cmd(ser, 0x08)) # 目标
        time.sleep(0.1)
        ser.write(send_cmd(ser, 0x06)) # 点云
        time.sleep(0.1)
        ser.reset_input_buffer()

        t = threading.Thread(target=parse_data, args=(ser,), daemon=True)
        t.start()
        
        # 准备 CSV (追加模式，防止误删)
        import os
        file_exists = os.path.isfile(CSV_FILENAME)
        
        with open(CSV_FILENAME, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                # 写入包含 10 个特征 + label 的表头
                writer.writerow(feature_extractor.FEATURE_NAMES + ['label'])
            
            print(f"\n✅ 已连接! 数据将存入: {CSV_FILENAME}")
            print("请按标签录制: 0=无人, 1=站立, 2=坐下, 3=跌倒 (按 q 退出)")
            
            while True:
                user_input = input(f"当前标签 [{current_label}] (输入新值录制，回车继续): ")
                if user_input == 'q': break
                if user_input.isdigit(): current_label = int(user_input)
                
                print(f"正在录制标签 {current_label} (20帧)...", end="", flush=True)
                for _ in range(20):
                    with data_lock:
                        # --- 核心修改：调用特征提取器 ---
                        feats = feature_extractor.extract_features(current_target, current_points)
                        row = feats + [current_label]
                    
                    writer.writerow(row)
                    time.sleep(0.05)
                print(" 完成")

    except Exception as e:
        print(e)
    finally:
        stop_flag = True