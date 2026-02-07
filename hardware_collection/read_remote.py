import serial
import time

# --- 配置 ---
SERIAL_PORT = '/dev/ttyACM1'  # 串口号
BAUD_RATE = 9600              # 波特率

def read_remote_robust():
    print(f"正在打开串口 {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print("✅ 串口已连接！请依次按下遥控器按钮...")
        print("----------------------------------------------------")
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        return

    # 数据缓存池
    buffer = bytearray()

    try:
        while True:
            # 1. 只要串口有数据，就全部读进缓存池
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

            # 2. 在缓存池中寻找完整的数据包
            # 完整帧长 21 字节 (根据你提供的日志分析)
            while len(buffer) >= 21:
                # 检查帧头是否为 AA 55
                if buffer[0] == 0xAA and buffer[1] == 0x55:
                    # 检查帧尾是否为 FE (根据你的日志，最后一位通常是 FE)
                    if buffer[20] == 0xFE:
                        # --- 找到完整包！提取数据 ---
                        frame = buffer[:21]
                        
                        # 提取按键值 (第14个字节，索引13)
                        # 数据格式: AA 55 ... [11:序列号] 00 [13:按键值] ... FE
                        key_value = frame[13]
                        
                        # 打印结果
                        hex_str = ' '.join([f'{b:02X}' for b in frame])
                        print(f"📦 完整帧: {hex_str}")
                        print(f"🔑 捕获按键值 (Hex): {key_value:02X}")
                        
                        # 简单过滤下松开按键的码 (假设 0D 是松开，方便你看清)
                        if key_value == 0x0D:
                            print("   (可能是按键松开/心跳)")
                        else:
                            print("   👉 ** 有效指令 **")
                        
                        print("----------------------------------------------------")
                        
                        # 处理完这一帧，从缓存中删掉
                        del buffer[:21]
                    else:
                        # 头对但尾不对？可能是中间有干扰，丢弃头部往后找
                        del buffer[0]
                else:
                    # 头部不是 AA 55，说明是垃圾数据或上一帧的残余，丢弃这一个字节，继续往后找
                    del buffer[0]
            
            # 防止CPU占用过高
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n👋 程序停止")
    finally:
        ser.close()

if __name__ == "__main__":
    read_remote_robust()