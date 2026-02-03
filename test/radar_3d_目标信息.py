import serial
import struct
import threading
import time
import math
from collections import deque

# ---------------- 配置区域 ----------------
SERIAL_PORT = '/dev/ttyACM0'  # 请替换为您的串口
BAUD_RATE = 115200
INSTALL_MODE = 0x14  # 侧装
# ----------------------------------------

stop_flag = False

def calc_checksum(data_bytes):
    checksum = 0
    for b in data_bytes:
        checksum ^= b
    return (~checksum) & 0xFF

def send_command_int32(ser, cmd_value):
    header = b'\x01\x00\x01\x00\x04\x02\x01'
    h_cs = calc_checksum(header)
    payload = struct.pack('<I', cmd_value)
    d_cs = calc_checksum(payload)
    return header + bytes([h_cs]) + payload + bytes([d_cs])

def init_radar_config(ser):
    print("\n[初始化] 正在配置雷达...")
    
    # 1. 设置侧装
    ser.write(send_command_int32(ser, INSTALL_MODE))
    time.sleep(0.2)
    
    # 2. 【关键】开启目标信息显示 (0x08)
    # 这会启用 0x0A04 协议回传
    print("  -> 开启目标信息 (Command: 0x08)")
    ser.write(send_command_int32(ser, 0x08))
    time.sleep(0.2)

    # 3. 同时也开启点云 (0x06)，方便对比
    print("  -> 开启点云信息 (Command: 0x06)")
    ser.write(send_command_int32(ser, 0x06))
    time.sleep(0.2)
    
    ser.reset_input_buffer()
    print("[初始化] 完成。正在监听目标数据 (0x0A04)...\n")

def process_target_info(payload):
    """
    [cite_start]解析 0x0A04 目标信息帧 [cite: 163]
    格式: TargetNum(4) + N * [x(4), y(4), z(4), dop_idx(4), cluster_id(4)]
    """
    if len(payload) < 4: return
    
    # 解析目标数量
    num = struct.unpack('<i', payload[0:4])[0]
    
    if num > 0:
        print(f"🎯 发现 {num} 个目标 (0x0A04):")
        
    offset = 4
    for i in range(num):
        if offset + 20 > len(payload): break
        try:
            # 解析 20 字节结构
            x, y, z, dop_idx, cluster_id = struct.unpack('<fffii', payload[offset:offset+20])
            
            # --- 重点观察 dop_idx ---
            # dop_idx 是 int32，如果它不为0，说明有多普勒速度！
            speed_status = "🛑静止"
            if abs(dop_idx) > 0:
                speed_status = f"🚀运动 (Idx={dop_idx})"
            
            print(f"   目标[{i}]: Pos=({x:.2f}, {y:.2f}) | Z={z:.2f} | 速度索引(dop_idx)={dop_idx} [{speed_status}]")
            
        except struct.error:
            pass
        offset += 20

def serial_thread_task():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ 串口已连接")
        init_radar_config(ser)
        
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
                    
                    payload = buffer[8 : 8+data_len]
                    if calc_checksum(payload) == buffer[8+data_len]:
                        
                        # --- 0x0A04: 目标信息 (重点看这个) ---
                        if frame_type == 0x0A04:
                            process_target_info(payload)
                            
                        # --- 0x0A08: 点云信息 (顺便看一眼) ---
                        elif frame_type == 0x0A08:
                            # 这里简单打印一下点云数，证明点云也在传
                            num = struct.unpack('<i', payload[0:4])[0]
                            # print(f"   (点云帧: {num} 点)") 
                            pass
                            
                    buffer = buffer[total_len:]
                except Exception as e:
                    print(f"err: {e}")
                    buffer = buffer[1:]
    except Exception as e:
        print(f"❌ 串口错误: {e}")

if __name__ == "__main__":
    t = threading.Thread(target=serial_thread_task)
    t.daemon = True 
    t.start()
    
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        stop_flag = True
        print("停止")