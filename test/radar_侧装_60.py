import serial
import struct
import threading
import time
import math  # 引入 math 库处理 nan
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

# ---------------- 配置区域 ----------------
SERIAL_PORT = '/dev/ttyACM0'  # Linux/Mac 路径，Windows 请改为 'COM3' 等
BAUD_RATE = 115200            # 协议默认波特率

# 安装方式配置
# 0x13: 顶装 (Top Mount)
# 0x14: 侧装 (Side Mount) - 您当前的使用场景
INSTALL_MODE = 0x14
# ----------------------------------------

# 全局缓冲区
global_point_buffer = deque(maxlen=10) 
data_lock = threading.Lock()
stop_flag = False

def calc_checksum(data_bytes):
    """ 计算校验和 """
    checksum = 0
    for b in data_bytes:
        checksum ^= b
    return (~checksum) & 0xFF

def send_command_int32(ser, cmd_value):
    """ 发送带有 int32 参数的控制指令 """
    header = b'\x01\x00\x01\x00\x04\x02\x01' # SOF, ID, LEN=4, TYPE=0x0201
    h_cs = calc_checksum(header)
    payload = struct.pack('<I', cmd_value) # int32 小端序
    d_cs = calc_checksum(payload)
    frame = header + bytes([h_cs]) + payload + bytes([d_cs])
    return frame

def init_radar_config(ser):
    """ 初始化雷达配置 """
    print("\n[初始化] 正在配置雷达...")
    
    # 1. 设置安装模式 (侧装)
    cmd_install = send_command_int32(ser, INSTALL_MODE)
    ser.write(cmd_install)
    mode_str = "侧装 (Side)" if INSTALL_MODE == 0x14 else "顶装 (Top)"
    print(f"  -> [TX] 发送模式设置: {mode_str} (0x{INSTALL_MODE:02X})")
    
    time.sleep(0.5) 
    
    # 2. 开启点云
    cmd_cloud = send_command_int32(ser, 0x06) 
    ser.write(cmd_cloud)
    print(f"  -> [TX] 发送开启点云指令 (0x06)")
    
    time.sleep(0.5)
    ser.reset_input_buffer()
    print("[初始化] 完成，正在等待数据流...\n")

def process_point_cloud(payload):
    """ 
    解析并打印点云数据 
    """
    if len(payload) < 4: return
    
    # 解析点数
    num = struct.unpack('<i', payload[0:4])[0]
    points = []
    offset = 4
    
    # --- 【这里是打印输出部分】 ---
    if num > 0:
        print(f"--- 收到帧: {num} 个点 ---")
    
    for i in range(num):
        if offset + 20 > len(payload): break
        
        try:
            # 解析: cluster_id(4), x(4), y(4), z(4), speed(4)
            _, x, y, z, speed = struct.unpack('<iffff', payload[offset:offset+20])
            
            # 记录原始 Z 值用于打印检查
            raw_z = z 
            
            # 处理 NaN (如果 Z 是 nan，手动设为 0，防止绘图崩溃)
            is_nan = False
            if math.isnan(z) or math.isinf(z):
                z = 0.0
                is_nan = True
            if math.isnan(x): x = 0.0
            if math.isnan(y): y = 0.0

            # 过滤异常噪点 (可选)
            if abs(x) < 5.0 and 0 <= y < 8.0:
                 points.append((x, y, z, speed))
                 
                 # --- 【终端数值打印】只打印前 5 个点，避免刷屏太快 ---
                 if i < 50:
                     status = "⚠️NaN修正" if is_nan else "正常"
                     print(f"   Point[{i}]: X={x:5.2f}, Y={y:5.2f}, Z={z:5.2f} (原始Z:{raw_z}), V={speed:5.2f} | {status}")

        except struct.error:
            pass
            
        offset += 20
        
    if num > 5:
        print(f"   ... (剩余 {num-5} 个点未显示)")

    with data_lock:
        if points: 
            global_point_buffer.append(points)

def serial_thread_task():
    global global_point_buffer
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ 串口已连接: {SERIAL_PORT}")
        
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
                        if frame_type == 0x0A08:
                            process_point_cloud(payload) # 调用处理函数
                                    
                    buffer = buffer[total_len:]
                except Exception as e:
                    print(f"err: {e}")
                    buffer = buffer[1:]
    except Exception as e:
        print(f"❌ 串口错误: {e}")

def visualization_task():
    """ 3D 可视化窗口 """
    fig = plt.figure(figsize=(10, 8))
    fig.canvas.manager.set_window_title('HLK-LD6002B Real-time Data')
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=20, azim=-45)

    scatter_plot = None
    print("📊 启动 3D 窗口...")
    
    try:
        while not stop_flag and plt.fignum_exists(fig.number):
            all_points = []
            with data_lock:
                for frame in global_point_buffer:
                    all_points.extend(frame)
            
            if scatter_plot: 
                try: scatter_plot.remove()
                except: pass
                scatter_plot = None

            # 固定坐标轴，方便观察侧装效果
            ax.set_xlim(-2, 2)   # 左右
            ax.set_ylim(0, 5)    # 距离
            ax.set_zlim(-1, 2)   # 高度
            
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            
            # 画原点
            ax.scatter([0], [0], [0], c='r', marker='^', s=50)

            if all_points:
                xs = [p[0] for p in all_points]
                ys = [p[1] for p in all_points]
                zs = [p[2] for p in all_points]
                # 用 Z 轴高度作为颜色，越红越高
                scatter_plot = ax.scatter(xs, ys, zs, c=zs, cmap='jet', s=30, alpha=0.8)
            
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
        print("程序已停止")