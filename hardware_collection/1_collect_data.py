import serial
import struct
import threading
import time
import csv
import os
import sys
import config            # 导入配置
import feature_extractor # 导入特征提取

# --- 全局变量 ---
command_queue = [] 
current_target = {'z': 0.0, 'speed': 0.0}
current_points = []
data_lock = threading.Lock()
stop_flag = False
is_busy = False 

# ====================================================================
# 模块 1: 遥控器监听线程 (带身份验证 & 校验和)
# ====================================================================
def remote_listener_thread():
    """
    监听遥控器，执行三层过滤：
    1. 包完整性 (Header/Tail)
    2. 数据校验 (Checksum)
    3. 身份验证 (Remote ID)
    """
    global is_busy
    print(f"🎮 [遥控器] 正在连接 {config.REMOTE_PORT} ...")
    print(f"🔐 [安全] 仅响应 ID: {[hex(x) for x in config.TARGET_REMOTE_ID]}")

    try:
        ser = serial.Serial(config.REMOTE_PORT, config.REMOTE_BAUD, timeout=0.1)
    except Exception as e:
        print(f"❌ [遥控器] 连接失败: {e}")
        return

    buffer = bytearray()
    print("✅ [遥控器] 就绪! (非绑定设备将被忽略)")

    while not stop_flag:
        try:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

            # 完整包长度为 21 字节
            while len(buffer) >= 21:
                # 1. 检查包头 (AA 55)
                if buffer[0] == 0xAA and buffer[1] == 0x55:
                    # 2. 检查包尾 (FE) - 索引 20
                    if buffer[20] == 0xFE:
                        
                        # --- 3. 计算校验和 (Checksum) ---
                        # 协议：从包头(0)开始，到数据内容最后一个字节(18)求和
                        # 校验位是索引 19
                        cal_sum = sum(buffer[0:19]) & 0xFF
                        recv_sum = buffer[19]
                        
                        if cal_sum == recv_sum:
                            # --- 4. 身份验证 (Remote ID) ---
                            # ID 位于索引 7, 8, 9, 10
                            recv_id = list(buffer[7:11])
                            
                            if recv_id == config.TARGET_REMOTE_ID:
                                # --- 5. 提取按键码 ---
                                # 按键码位于索引 13
                                key_val = buffer[13]
                                
                                # 处理逻辑 (防误触 + 映射)
                                if key_val in config.KEY_MAPPING:
                                    if is_busy or len(command_queue) > 0:
                                        print(f"🔒 [忽略] 系统忙，指令已丢弃", end="\r")
                                    else:
                                        label = config.KEY_MAPPING[key_val]
                                        command_queue.append(label)
                                        # 漂亮的十六进制打印 ID
                                        id_str = ' '.join([f'{b:02X}' for b in recv_id])
                                        print(f"\n⚡ [验证通过] ID:{id_str} | 键值:{key_val:02X} -> 动作:{label}")
                            else:
                                # ID 不匹配 (干扰信号)
                                other_id = ' '.join([f'{b:02X}' for b in recv_id])
                                print(f"🛡️ [拦截] 检测到其他遥控器信号 ID: {other_id}", end="\r")
                                
                        else:
                            print(f"⚠️ [校验失败] 计算:{cal_sum:02X} != 接收:{recv_sum:02X}", end="\r")

                        # 移除处理完的帧
                        del buffer[:21]
                    else:
                        # 包尾不对，滑窗
                        del buffer[0]
                else:
                    # 包头不对，滑窗
                    del buffer[0]
            time.sleep(0.01)
        except Exception as e:
            print(f"遥控器线程错: {e}")
            time.sleep(1)

# ====================================================================
# 模块 2: 雷达通信 (保持不变)
# ====================================================================
def calc_checksum(data):
    checksum = 0
    for b in data: checksum ^= b
    return (~checksum) & 0xFF

def send_cmd(ser, cmd_val):
    header = b'\x01\x00\x01\x00\x04\x02\x01'
    h_cs = calc_checksum(header)
    payload = struct.pack('<I', cmd_val)
    d_cs = calc_checksum(payload)
    return header + bytes([h_cs]) + payload + bytes([d_cs])

def radar_listener_thread(ser):
    global current_target, current_points
    buffer = b""
    while not stop_flag:
        try:
            if ser.in_waiting: buffer += ser.read(ser.in_waiting)
            while len(buffer) >= 8:
                if buffer[0] != 0x01:
                    buffer = buffer[1:]
                    continue
                header = buffer[0:7]
                if calc_checksum(header) != buffer[7]:
                    buffer = buffer[1:]
                    continue
                _, _, data_len, frame_type = struct.unpack('>BHHH', header)
                total_len = 8 + data_len + 1
                if len(buffer) < total_len: break
                
                payload = buffer[8:8+data_len]
                if calc_checksum(payload) == buffer[8+data_len]:
                    if frame_type == 0x0A04 and len(payload) >= 24:
                        num = struct.unpack('<i', payload[0:4])[0]
                        if num > 0:
                            _, _, z, dop_idx, _ = struct.unpack('<fffii', payload[4:24])
                            with data_lock:
                                current_target['z'] = z
                                current_target['speed'] = float(dop_idx)
                    elif frame_type == 0x0A08 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        temp = []
                        off = 4
                        for _ in range(num):
                            if off+20 > len(payload): break
                            _, x, y, z, s = struct.unpack('<iffff', payload[off:off+20])
                            if abs(x)<4 and 0.1<y<6: temp.append((x,y,s))
                            off += 20
                        with data_lock: current_points = temp
                buffer = buffer[total_len:]
        except: buffer = buffer[1:]

# ====================================================================
# 模块 3: 主程序
# ====================================================================
if __name__ == "__main__":
    if not os.path.exists(config.DATA_DIR): os.makedirs(config.DATA_DIR)

    t_remote = threading.Thread(target=remote_listener_thread, daemon=True)
    t_remote.start()

    try:
        print(f"📡 [雷达] 正在连接 {config.RADAR_PORT} ...")
        radar_ser = serial.Serial(config.RADAR_PORT, config.RADAR_BAUD, timeout=0.1)
        for _ in range(2):
            for cmd in [0x14, 0x08, 0x06]:
                radar_ser.write(send_cmd(radar_ser, cmd))
                time.sleep(0.05)
        radar_ser.reset_input_buffer()
        
        t_radar = threading.Thread(target=radar_listener_thread, args=(radar_ser,), daemon=True)
        t_radar.start()

        file_exists = os.path.isfile(config.CSV_PATH)
        with open(config.CSV_PATH, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(feature_extractor.FEATURE_NAMES + ['label'])
            
            print("\n" + "="*50)
            print(f"✅ 数据文件: {config.CSV_PATH}")
            print(f"⚙️ 采集模式: 身份验证遥控录制 ({config.COLLECT_NUM_FRAMES}帧/次)")
            print("🎮 等待专属遥控器指令...")
            print("="*50 + "\n")
            
            while True:
                if command_queue:
                    label = command_queue.pop(0)
                    is_busy = True 
                    
                    label_name = config.LABEL_NAMES.get(label, str(label))
                    print(f"\n🎥 [开始] 录制 [{label_name}]...")
                    
                    for i in range(config.COLLECT_NUM_FRAMES):
                        with data_lock:
                            feats = feature_extractor.extract_features(current_target, current_points)
                            if i % 20 == 0:
                                print(f"\r✅ 录制中: Z={feats[0]:.2f}m ({i}/{config.COLLECT_NUM_FRAMES})", end="")
                        
                        writer.writerow(feats + [label])
                        f.flush()
                        time.sleep(config.COLLECT_DELAY)
                    
                    print(f"\n✨ [完成] 录制结束")
                    is_busy = False
                else:
                    time.sleep(0.05)

    except KeyboardInterrupt:
        stop_flag = True
        print("\n👋 程序退出")
    except Exception as e:
        stop_flag = True
        print(f"\n❌ 错误: {e}")