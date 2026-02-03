import serial
import struct
import threading
import time
import csv
import os
import config            # 导入配置
import feature_extractor # 导入特征提取

# 全局变量
current_target = {'z': 0.0, 'speed': 0.0}
current_points = []
data_lock = threading.Lock()
stop_flag = False

# --- 核心修正：通信校验函数 (XOR 算法) ---
def calc_checksum(data):
    """
    修正后的校验算法：使用异或 (XOR)
    """
    checksum = 0
    for b in data:
        checksum ^= b
    return (~checksum) & 0xFF

def send_cmd(ser, cmd_val):
    """
    封装发送指令：自动计算头部和载荷的校验和
    """
    # 协议头: SOF(1) + ID(2) + LEN(2) + TYPE(2)
    # ID=1, LEN=4, TYPE=0x0201 (控制指令)
    header = b'\x01\x00\x01\x00\x04\x02\x01'
    
    # 计算头校验
    h_cs = calc_checksum(header)
    
    # 负载: int32 小端序
    payload = struct.pack('<I', cmd_val)
    
    # 计算数据校验
    d_cs = calc_checksum(payload)
    
    # 拼接完整帧
    frame = header + bytes([h_cs]) + payload + bytes([d_cs])
    return frame

def parse_data(ser):
    """
    完整的数据解析线程：不再省略任何逻辑
    """
    global current_target, current_points
    buffer = b""
    print("DEBUG: 数据接收线程已启动，正在监听数据流...")
    
    while not stop_flag:
        try:
            if ser.in_waiting: 
                buffer += ser.read(ser.in_waiting)
            
            # 寻找帧头 SOF (0x01)
            # 至少需要 8 字节 (7字节头 + 1字节头校验) 才能开始判断
            while len(buffer) >= 8:
                if buffer[0] != 0x01:
                    buffer = buffer[1:] # 不是头，丢弃
                    continue
                
                # 1. 校验头部
                header = buffer[0:7]
                h_cksum_recv = buffer[7]
                
                if calc_checksum(header) != h_cksum_recv:
                    buffer = buffer[1:] # 校验失败，滑动窗口
                    continue
                
                # 解析头部信息
                _, _, data_len, frame_type = struct.unpack('>BHHH', header)
                total_len = 8 + data_len + 1 # 头(7+1) + 数据(N) + 数据校验(1)
                
                if len(buffer) < total_len: 
                    break # 数据没收全，等待下一波串口数据
                
                # 2. 校验数据体
                payload = buffer[8 : 8+data_len]
                d_cksum_recv = buffer[8+data_len]
                
                if calc_checksum(payload) == d_cksum_recv:
                    
                    # --- 解析 0x0A04 (目标信息) ---
                    # 格式: Num(4) + [x, y, z, dop_idx, cluster_id]...
                    if frame_type == 0x0A04 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        if num > 0:
                            # 我们只取第一个主要目标 (Offset=4)
                            # 20字节结构: x(4), y(4), z(4), dop_idx(4), cluster_id(4)
                            if len(payload) >= 24:
                                _, _, z, dop_idx, _ = struct.unpack('<fffii', payload[4:24])
                                with data_lock:
                                    current_target['z'] = z
                                    current_target['speed'] = float(dop_idx)
                                
                    # --- 解析 0x0A08 (点云信息) ---
                    # 格式: Num(4) + [cluster_id, x, y, z, speed]...
                    elif frame_type == 0x0A08 and len(payload) >= 4:
                        num = struct.unpack('<i', payload[0:4])[0]
                        temp_points = []
                        offset = 4
                        for _ in range(num):
                            if offset + 20 > len(payload): break
                            # 解析单个点: cluster(4), x(4), y(4), z(4), speed(4)
                            _, x, y, z, s = struct.unpack('<iffff', payload[offset:offset+20])
                            
                            # 简单清洗: 过滤掉太远或异常的噪点
                            if abs(x) < 4.0 and 0.1 < y < 6.0:
                                temp_points.append((x, y, s))
                            
                            offset += 20
                        
                        with data_lock:
                            current_points = temp_points
                            
                # 处理完一帧，从缓冲区移除该帧数据
                buffer = buffer[total_len:] 
                
        except Exception as e:
            # 捕获解析过程中的意外错误，防止线程退出
            print(f"解析出错: {e}")
            buffer = buffer[1:]

if __name__ == "__main__":
    try:
        # 确保数据目录存在
        if not os.path.exists(config.DATA_DIR): 
            os.makedirs(config.DATA_DIR)
        
        print(f"正在打开串口 {config.SERIAL_PORT}...")
        ser = serial.Serial(config.SERIAL_PORT, config.BAUD_RATE, timeout=0.1)
        
        # --- 初始化雷达 (暴力唤醒模式) ---
        # 很多时候雷达没反应是因为初始化指令发丢了，这里多发几次
        print("正在初始化雷达...")
        
        # 1. 先清空一下可能存在的乱码
        ser.write(b'\x00\x00\x00')
        time.sleep(0.1)
        
        # 2. 循环发送关键指令
        cmds = [
            (0x14, "侧装模式"), 
            (0x08, "开启目标信息(0x0A04)"), 
            (0x06, "开启点云信息(0x0A08)"),
            (0x0C, "设置高灵敏度"), 
            (0x0F, "设置快速触发")
        ]
        
        for _ in range(2): # 发送两轮，确保雷达收到
            for cmd, name in cmds:
                # print(f"  -> 发送: {name}") 
                ser.write(send_cmd(ser, cmd))
                time.sleep(0.1) # 稍微延时，防止指令粘包
        
        ser.reset_input_buffer()
        print("✅ 初始化指令已发送，等待数据回传...")

        # 启动接收线程
        t = threading.Thread(target=parse_data, args=(ser,), daemon=True)
        t.start()
        
        # 准备 CSV 文件
        file_exists = os.path.isfile(config.CSV_PATH)
        with open(config.CSV_PATH, 'a', newline='') as f:
            writer = csv.writer(f)
            # 如果文件不存在，写入表头
            if not file_exists:
                writer.writerow(feature_extractor.FEATURE_NAMES + ['label'])
            
            print(f"\n✅ 数据将追加到: {config.CSV_PATH}")
            print(f"⚙️ 每次采集: {config.COLLECT_NUM_FRAMES} 帧 (约 {config.COLLECT_NUM_FRAMES * config.COLLECT_DELAY} 秒)")
            print("❗ 提示: 采集时请确保雷达前方有人活动，否则 Z轴 可能为 0")
            
            while True:
                u_in = input("\n请输入标签 (0=空闲, 1=站立, 2=坐下, 3=跌倒, q=退出): ")
                if u_in.lower() == 'q': break
                if not u_in.isdigit(): continue
                
                label = int(u_in)
                print(f"🎥 开始录制标签 [{label}] ... 请变换姿态!")
                
                for i in range(config.COLLECT_NUM_FRAMES):
                    with data_lock:
                        # 提取特征 (这里 current_target 和 current_points 应该已经被线程更新了)
                        feats = feature_extractor.extract_features(current_target, current_points)
                        
                        # --- 实时反馈区 ---
                        # 如果 Z轴(feats[0]) 为0 且 点云数(feats[8]) 为0，说明没读到有效数据
                        if i % 20 == 0: # 每20帧打印一次状态，避免刷屏太快
                            if feats[0] == 0 and feats[8] == 0:
                                print(f"\r⚠️ [无数据] 请在雷达前晃动... ({i}/{config.COLLECT_NUM_FRAMES})", end="")
                            else:
                                print(f"\r✅ 录制中: Z={feats[0]:.2f}m | 点云数={feats[8]} ({i}/{config.COLLECT_NUM_FRAMES})", end="")
                    
                    writer.writerow(feats + [label])
                    time.sleep(config.COLLECT_DELAY)
                
                print("\n完成!")

    except Exception as e:
        print(f"\n❌ 发生严重错误: {e}")
    finally:
        stop_flag = True
        print("程序已退出")