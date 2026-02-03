import serial
import struct
import threading
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

# ---------------- 配置区域 ----------------
SERIAL_PORT = '/dev/ttyACM0' # 串口设备路径
BAUD_RATE = 115200 # 波特率，若无数据请尝试 1382400

# 安装方式配置 (根据实际测试安装方式，测试侧装方式偏多)
# 0x13: 顶装 (Top Mount) - 适合吸顶安装，输出 X,Y,Height
# 0x14: 侧装 (Side Mount) - 适合挂墙安装，输出 X,Y,Z
INSTALL_MODE = 0x14
# ----------------------------------------

# 数据缓冲区 (保留最近 5 帧，制造拖尾效果)
global_point_buffer = deque(maxlen=5) 
data_lock = threading.Lock()
stop_flag = False

def calc_checksum(data_bytes):
    checksum = 0
    for b in data_bytes:
        checksum ^= b
    return (~checksum) & 0xFF

def send_command_int32(ser, cmd_value):
    """ 发送带有 int32 参数的控制指令 (0x0201) """
    # Header: SOF(1)+ID(2)+LEN(2)+TYPE(2)
    # [cite_start]TYPE=0x0201 (控制指令) [cite: 1371]
    header = b'\x01\x00\x01\x00\x04\x02\x01'
    h_cs = calc_checksum(header)
    
    # Payload: int32 小端序
    payload = struct.pack('<I', cmd_value)
    d_cs = calc_checksum(payload)
    
    frame = header + bytes([h_cs]) + payload + bytes([d_cs])
    return frame

def init_radar_config(ser):
    """ 初始化雷达配置：开启点云 + 设置安装方式 """
    print("\n[配置] 正在初始化雷达...")
    
    # [cite_start]1. 发送开启点云指令 (Command: 0x06) [cite: 1378]
    cmd_cloud = send_command_int32(ser, 0x06)
    ser.write(cmd_cloud)
    print(f"  -> 发送: 开启点云 (0x06)")
    time.sleep(0.2)
    
    # [cite_start]2. 发送安装方式指令 (Command: 0x13 顶装 / 0x14 侧装) [cite: 1395, 1396]
    # 这一步非常关键！如果不设置，Z轴可能一直是0
    cmd_install = send_command_int32(ser, INSTALL_MODE)
    ser.write(cmd_install)
    mode_str = "顶装 (Top)" if INSTALL_MODE == 0x13 else "侧装 (Side)"
    print(f"  -> 发送: 设置为{mode_str}模式 (0x{INSTALL_MODE:02X})")
    time.sleep(0.2)
    print("[配置] 完成，等待数据...\n")

def serial_thread_task():
    global global_point_buffer
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ 串口已连接: {SERIAL_PORT}")
        
        time.sleep(1)
        init_radar_config(ser)
        
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
                    if calc_checksum(header) != buffer[7]:
                        buffer = buffer[1:]
                        continue
                    
                    _, _, data_len, frame_type = struct.unpack('>BHHH', header)
                    total_len = 8 + data_len + 1
                    
                    if len(buffer) < total_len: break 
                    
                    payload = buffer[8 : 8+data_len]
                    if calc_checksum(payload) == buffer[8+data_len]:
                        
                        # [cite_start]解析点云 (0x0A08) [cite: 1475]
                        if frame_type == 0x0A08 and len(payload) >= 4:
                            num = struct.unpack('<i', payload[0:4])[0]
                            points = []
                            offset = 4
                            
                            # 终端输出头部
                            if num > 0:
                                print(f"\n 收到帧: {num} 个点")
                                
                            for i in range(num):
                                if offset + 20 > len(payload): break
                                # 解析: cluster(4), x(4), y(4), z(4), speed(4)
                                clus, x, y, z, speed = struct.unpack('<iffff', payload[offset:offset+20])
                                points.append((x, y, z, speed))
                                
                                # --- 终端输出点信息 (前5个点) ---
                                if i < 5: 
                                    print(f"   Ref[{i}]: X={x:5.2f}m, Y={y:5.2f}m, Z={z:5.2f}m, V={speed:5.2f}m/s")
                                offset += 20
                                
                            if num > 5: print(f"   ... (剩余 {num-5} 个点隐藏)")

                            with data_lock:
                                if points: global_point_buffer.append(points)
                                    
                    buffer = buffer[total_len:]
                except:
                    buffer = buffer[1:]
    except Exception as e:
        print(f"❌ 串口错误: {e}")

def visualization_task():
    """ 3D 可视化窗口 """
    fig = plt.figure(figsize=(10, 8))
    fig.canvas.manager.set_window_title('HLK-LD6002B 3D Point Cloud')
    ax = fig.add_subplot(111, projection='3d')
    
    # 初始视角设置 (俯视+侧视，增强立体感)
    ax.view_init(elev=25, azim=-45) 

    scatter_plot = None
    print("启动 3D 可视化...")
    
    try:
        while not stop_flag:
            all_points = []
            with data_lock:
                for frame in global_point_buffer:
                    all_points.extend(frame)
            
            if scatter_plot: scatter_plot.remove()
            
            # 刷新坐标轴 (保持范围固定，防止画面抖动)
            ax.set_xlim(-3, 3) 
            ax.set_ylim(0, 6)   
            ax.set_zlim(-2, 2) # Z轴范围，如果是顶装，Z通常为负数(向下)或高度
            ax.set_xlabel('X (Lateral)')
            ax.set_ylabel('Y (Distance)')
            ax.set_zlabel('Z (Height)')
            
            # 绘制雷达原点
            ax.scatter([0], [0], [0], c='red', marker='^', s=100)

            if all_points:
                xs = [p[0] for p in all_points]
                ys = [p[1] for p in all_points]
                zs = [p[2] for p in all_points]
                # 颜色根据 Z 轴高度变化，增强立体感
                scatter_plot = ax.scatter(xs, ys, zs, c=zs, cmap='plasma', s=20, alpha=0.8)
            
            plt.pause(0.05)
            
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