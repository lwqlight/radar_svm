import serial
import struct
import threading
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque  # 引入双端队列用于做数据缓冲

# ---------------- 配置区域 ----------------
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200 
# ----------------------------------------

# 全局变量
# 使用 deque 作为缓冲区，maxlen=10 表示保留最近10帧的数据（余辉效果）
# 如果觉得拖尾太长，可以把 maxlen 改小，比如 5
global_point_buffer = deque(maxlen=8) 
data_lock = threading.Lock()
stop_flag = False

def calc_checksum(data_bytes):
    checksum = 0
    for b in data_bytes:
        checksum ^= b
    return (~checksum) & 0xFF

def send_enable_point_cloud(ser):
    """ 发送开启点云指令 """
    header = b'\x01\x00\x01\x00\x04\x02\x01'
    h_cs = calc_checksum(header)
    payload = b'\x06\x00\x00\x00'
    d_cs = calc_checksum(payload)
    frame = header + bytes([h_cs]) + payload + bytes([d_cs])
    print(f"[TX] 发送开启点云指令: {frame.hex().upper()}")
    ser.write(frame)

def serial_thread_task():
    """ 串口读取线程 """
    global global_point_buffer
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ 串口已连接: {SERIAL_PORT}")
        
        time.sleep(1)
        send_enable_point_cloud(ser)
        
        buffer = b""
        while not stop_flag:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

            while len(buffer) >= 8:
                if buffer[0] != 0x01:
                    buffer = buffer[1:]
                    continue
                try:
                    header = buffer[0:7]
                    h_cksum_recv = buffer[7]
                    if calc_checksum(header) != h_cksum_recv:
                        buffer = buffer[1:]
                        continue
                    
                    _, _, data_len, frame_type = struct.unpack('>BHHH', header)
                    total_len = 8 + data_len + 1
                    
                    if len(buffer) < total_len:
                        break 
                    
                    payload = buffer[8 : 8+data_len]
                    d_cksum_recv = buffer[8+data_len]
                    
                    if calc_checksum(payload) == d_cksum_recv:
                        # 0x0A08 点云数据
                        if frame_type == 0x0A08:
                            if len(payload) >= 4:
                                num = struct.unpack('<i', payload[0:4])[0]
                                current_frame_points = []
                                offset = 4
                                for _ in range(num):
                                    if offset + 20 > len(payload): break
                                    _, x, y, z, speed = struct.unpack('<iffff', payload[offset:offset+20])
                                    # 这里可以过滤掉一些明显错误的噪点，例如 z > 3.0 或 z < -3.0
                                    current_frame_points.append((x, y, z, speed))
                                    offset += 20
                                
                                # 将当前帧数据加入缓冲区（自动挤掉最老的一帧）
                                with data_lock:
                                    if current_frame_points: # 只有非空才添加，防止空帧闪烁
                                        global_point_buffer.append(current_frame_points)
                                    
                    buffer = buffer[total_len:]
                    
                except Exception:
                    buffer = buffer[1:]
                    
    except Exception as e:
        print(f"❌ 串口错误: {e}")

def visualization_task():
    """ 
    可视化主循环 (优化版)
    优化策略：不使用 cla() 清空整个画布，而是只移除上一帧的散点对象。
    """
    fig = plt.figure(figsize=(10, 8))
    fig.canvas.manager.set_window_title('HLK-LD6002B Point Cloud (Optimized)')
    
    ax = fig.add_subplot(111, projection='3d')
    
    # --- 初始化坐标系 (只设置一次，固定住背景) ---
    ax.set_title("Real-time Point Cloud (Persistence Mode)")
    ax.set_xlim(-3, 3) 
    ax.set_ylim(0, 6)   
    ax.set_zlim(-2, 2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    # 绘制雷达原点
    ax.scatter([0], [0], [0], c='r', marker='^', s=100, label='Radar')

    # 用于存储上一帧的散点图对象
    scatter_plot = None

    print("📊 启动平滑可视化窗口...")
    
    try:
        while not stop_flag:
            # 1. 获取缓冲区中的所有点 (将最近几帧的数据合并)
            all_points = []
            with data_lock:
                # global_point_buffer 包含多帧，每帧包含多个点
                for frame in global_point_buffer:
                    all_points.extend(frame)
            
            # 2. 如果有旧的散点图，先移除它 (不要清除整个ax，只移除点)
            if scatter_plot:
                scatter_plot.remove()
                scatter_plot = None

            # 3. 绘制新的点
            if all_points:
                xs = [p[0] for p in all_points]
                ys = [p[1] for p in all_points]
                zs = [p[2] for p in all_points]
                # 使用速度或距离来着色
                colors = [p[1] for p in all_points] 
                
                # 绘制新的散点
                scatter_plot = ax.scatter(xs, ys, zs, c=colors, cmap='viridis', s=20, alpha=0.6)
            
            # 4. 极短暂停，刷新画面
            plt.pause(0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        plt.close()

if __name__ == "__main__":
    t = threading.Thread(target=serial_thread_task)
    t.daemon = True 
    t.start()
    
    try:
        visualization_task()
    except KeyboardInterrupt:
        stop_flag = True
        time.sleep(0.5)