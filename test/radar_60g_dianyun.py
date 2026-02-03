import serial
import struct
import time

# 串口配置
SERIAL_PORT = '/dev/ttyUSB0'
# 注意：文档未明确波特率，HLK-LD6002通常为 115200 或 1382400 (高速点云)
# 如果运行无数据，请将此处改为 1382400
BAUD_RATE = 115200 

def calc_checksum(data_bytes):
    """
    计算校验和：异或后取反
    文档参考: "所有...先全部进行异或运算，再取反" 
    """
    checksum = 0
    for b in data_bytes:
        checksum ^= b
    return (~checksum) & 0xFF

def parse_frame(frame_type, data):
    """
    根据 TYPE 解析 DATA
    数据部分采用小端序 
    """
    try:
        # 4.1 报告有无人检测结果 0x0F09 [cite: 1190]
        if frame_type == 0x0F09:
            # DATA: 2 byte uint8. [is_human] 0000无人, 0100有人 [cite: 1196-1198]
            # 虽然文档说是uint8，但长度是2字节，且示例是01 00。
            # 小端序 01 00 -> 0x0001
            is_human = data[0] 
            status = "有人" if is_human == 1 else "无人"
            print(f"[0x0F09] 有无人状态: {status} (原始值: {data.hex()})")

        # 4.2 报告人员位置 0x0A04 [cite: 1199]
        elif frame_type == 0x0A04:
            # 格式: target_num(int32) + [x, y, z, dop, cluster] * N
            if len(data) >= 4:
                target_num = struct.unpack('<i', data[0:4])[0] # int32 [cite: 1206]
                print(f"[0x0A04] 3D目标数量: {target_num}")
                
                # 解析每个目标 (每个目标20字节: 4byte*5)
                offset = 4
                for i in range(target_num):
                    if offset + 20 > len(data):
                        break
                    # x(float), y(float), z(float), dop(int32), cluster(int32) [cite: 1202]
                    chunk = data[offset : offset+20]
                    x, y, z, dop, cluster = struct.unpack('<fffii', chunk)
                    print(f"  > 目标[{i}]: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")
                    offset += 20

        # 4.3 报告相位测试结果 0x0A13 [cite: 1214]
        elif frame_type == 0x0A13:
            # total phase(float), breath phase(float), heart phase(float) [cite: 1217]
            if len(data) >= 12:
                total, breath, heart = struct.unpack('<fff', data[0:12])
                print(f"[0x0A13] 相位 - 总: {total:.2f}, 呼吸: {breath:.2f}, 心跳: {heart:.2f}")

        # 4.4 报告呼吸速率 0x0A14 [cite: 1221]
        elif frame_type == 0x0A14:
            # rate(float) [cite: 1224]
            if len(data) >= 4:
                rate = struct.unpack('<f', data[0:4])[0]
                print(f"[0x0A14] 呼吸速率: {rate:.1f} RPM")

        # 4.5 报告心跳速率 0x0A15 [cite: 1225]
        elif frame_type == 0x0A15:
            # rate(float) [cite: 1227]
            if len(data) >= 4:
                rate = struct.unpack('<f', data[0:4])[0]
                print(f"[0x0A15] 心跳速率: {rate:.1f} BPM")

        # 4.6 报告检测目标距离 0x0A16 [cite: 1231]
        elif frame_type == 0x0A16:
            # flag(uint32), range(float) [cite: 1234]
            if len(data) >= 8:
                flag, dist = struct.unpack('<If', data[0:8])
                if flag == 1: # 标志为1时输出距离 [cite: 1236]
                    print(f"[0x0A16] 目标距离: {dist:.2f} m")

        # 4.7 报告跟踪目标位置信息 0x0A17 [cite: 1239]
        elif frame_type == 0x0A17:
             # x(float), y(float), z(float) [cite: 1241]
             if len(data) >= 12:
                 x, y, z = struct.unpack('<fff', data[0:12])
                 print(f"[0x0A17] 跟踪坐标: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

        else:
            print(f"[未知类型] TYPE: 0x{frame_type:04X}, Data Len: {len(data)}")

    except Exception as e:
        print(f"数据解析错误: {e}")

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"串口已打开: {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e:
        print(f"无法打开串口: {e}")
        return

    buffer = b""
    
    try:
        while True:
            # 读取数据
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

            # 协议最小长度校验：SOF(1)+ID(2)+LEN(2)+TYPE(2)+H_CS(1) + DATA(0) + D_CS(1) = 9 bytes
            # 但考虑到DATA通常不为0，至少需要解析头部
            while len(buffer) >= 8:
                # 1. 寻找帧头 SOF (0x01) 
                if buffer[0] != 0x01:
                    buffer = buffer[1:] # 滑动窗口，丢弃无效数据
                    continue

                # 2. 解析头部 (ID, LEN, TYPE) - 大端序 
                # 头部结构: SOF(1) + ID(2) + LEN(2) + TYPE(2) + HEAD_CKSUM(1)
                try:
                    # 获取头部数据用于校验 SOF(1) + ID(2) + LEN(2) + TYPE(2) = 7 bytes
                    header_bytes = buffer[0:7]
                    if len(buffer) < 8: # 需要至少读到 HEAD_CKSUM
                        break
                        
                    received_h_cksum = buffer[7]
                    
                    # 计算头校验: ~(SOF ^ ID ^ LEN ^ TYPE)
                    # 参考文档中 "从SOF位到TYPE位先全部进行异或运算，再取反" 
                    calc_h_cksum = calc_checksum(header_bytes)
                    
                    if received_h_cksum != calc_h_cksum:
                        print(f"头校验失败: 接收 {hex(received_h_cksum)} != 计算 {hex(calc_h_cksum)}")
                        buffer = buffer[1:] # 丢弃SOF，重新寻找
                        continue
                    
                    # 头部校验通过，解析长度和类型
                    # ID(2), LEN(2), TYPE(2) 均为大端序 
                    _, frame_id, data_len, frame_type = struct.unpack('>BHHH', header_bytes)
                    
                    # 3. 检查剩余数据长度是否足够 (Header(8) + Data(LEN) + Data_Checksum(1))
                    total_packet_len = 8 + data_len + 1
                    if len(buffer) < total_packet_len:
                        break # 数据不够，等待下次循环
                    
                    # 4. 提取数据和数据校验
                    payload = buffer[8 : 8 + data_len]
                    received_d_cksum = buffer[8 + data_len]
                    
                    # 计算数据校验: ~(DATA...) 
                    calc_d_cksum = calc_checksum(payload)
                    
                    if received_d_cksum != calc_d_cksum:
                        print(f"数据校验失败: 接收 {hex(received_d_cksum)} != 计算 {hex(calc_d_cksum)}")
                        buffer = buffer[total_packet_len:] # 丢弃整个包
                        continue

                    # 5. 解析具体业务数据
                    parse_frame(frame_type, payload)
                    
                    # 6. 移除已处理的包
                    buffer = buffer[total_packet_len:]
                    
                except Exception as e:
                    print(f"帧处理异常: {e}")
                    buffer = buffer[1:]

            time.sleep(0.01) # 防止CPU占用过高

    except KeyboardInterrupt:
        print("\n程序停止")
        ser.close()

if __name__ == "__main__":
    main()