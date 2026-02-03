import serial
import struct
import math
import time

# ---------------- 配置区域 ----------------
# 根据您的设备列表，雷达端口很可能是 /dev/ttyACM0
SERIAL_PORT = '/dev/ttyUSB0'  

# [cite_start]注意：LD2450 默认波特率通常是 256000 [cite: 43]
BAUD_RATE = 256000           
# ----------------------------------------

def parse_coordinate(raw_value):
    """
    解析坐标值 (X 或 Speed)。
    文档定义: 最高位(bit 15)为符号位。
    1 = 正数 (值 = 原始值 - 32768)
    0 = 负数 (值 = 0 - 原始值)
    [cite_start]参考: HLK-LD2450使用教程V1.1.pdf [cite: 871, 872]
    """
    if raw_value & 0x8000:
        return raw_value & 0x7FFF
    else:
        return -raw_value

def parse_y_coordinate(raw_value):
    """
    解析 Y 坐标。
    文档定义: Y一直为正坐标，计算方式是 Y - 2^15 (即去掉最高位)
    [cite_start]参考: HLK-LD2450使用教程V1.1.pdf [cite: 873]
    """
    if raw_value & 0x8000:
        return raw_value & 0x7FFF
    else:
        return 0

def read_radar_data():
    ser = None
    try:
        # 初始化串口
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print(f"成功打开串口: {SERIAL_PORT} @ {BAUD_RATE}")
        print("正在等待雷达数据 (按 Ctrl+C 停止)...")

        buffer = b''
        
        while True:
            # 读取数据
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer += data

            # 检查缓冲区是否有完整的一帧数据 (30字节)
            # [cite_start]帧头: AA FF 03 00 [cite: 263]
            # [cite_start]帧尾: 55 CC [cite: 263]
            while len(buffer) >= 30:
                # 查找帧头
                if buffer[0:4] == b'\xAA\xFF\x03\x00':
                    # 检查帧尾 (第29, 30字节)
                    if buffer[28:30] == b'\x55\xCC':
                        # 提取一帧有效数据
                        frame = buffer[:30]
                        buffer = buffer[30:] 
                        
                        # 解析目标数据
                        # [cite_start]数据负载从第4字节开始，包含3个目标，每个目标8字节 [cite: 265]
                        payload = frame[4:28]
                        
                        found_target = False
                        print("-" * 50)
                        
                        for i in range(3):
                            base_idx = i * 8
                            target_data = payload[base_idx : base_idx + 8]
                            
                            # [cite_start]解析: x(2B), y(2B), speed(2B), resolution(2B) [cite: 266]
                            raw_x, raw_y, raw_speed, resolution = struct.unpack('<HHHH', target_data)
                            
                            # [cite_start]如果 X 和 Y 原始值都为 0，通常表示该目标不存在 [cite: 270]
                            if raw_x == 0 and raw_y == 0:
                                continue

                            found_target = True
                            # 转换单位 mm
                            x_mm = parse_coordinate(raw_x)
                            y_mm = parse_y_coordinate(raw_y)
                            speed_cms = parse_coordinate(raw_speed) # 单位 cm/s
                            
                            # 计算直线距离
                            distance_mm = math.sqrt(x_mm**2 + y_mm**2)
                            
                            print(f"[目标 {i+1}]")
                            print(f"  位置: X={x_mm}mm, Y={y_mm}mm")
                            print(f"  直线距离: {distance_mm:.2f} mm")
                            print(f"  速度: {speed_cms} cm/s")
                        
                        if not found_target:
                            print("当前区域无目标")

                    else:
                        # 帧尾不匹配，丢弃一字节重新寻找帧头
                        buffer = buffer[1:]
                else:
                    # 帧头不匹配，丢弃一字节
                    buffer = buffer[1:]
                    
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print(f"提示: 请检查是否拥有读取 {SERIAL_PORT} 的权限 (尝试 sudo)")
    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        if ser and ser.is_open:
            ser.close()

if __name__ == "__main__":
    read_radar_data()