import serial
import time
import cv2
import re

def send_data_from_file(file_path, com_port, baud_rate=115200):
    """
    从文件中读取数据并通过串口发送
    
    Args:
        file_path: 文本文件路径
        com_port: 串口号（如：'COM3' 或 '/dev/ttyUSB0'）
        baud_rate: 波特率，默认115200
    """
    try:
        # 1. 初始化串口
        ser = serial.Serial(com_port, baud_rate, timeout=1)
        print(f"串口 {com_port} 已打开，波特率 {baud_rate}")
        
        # 2. 读取文件
        with open(file_path, 'r') as file:
            lines = file.readlines()
            print(f"共读取到 {len(lines)} 行数据")
                 
            # 3. 处理每一行数据
            for i, line in enumerate(lines):
                line = line.strip()
                if not line.startswith("received"):
                    continue
                    
                # 提取 "cd" 开头 "dc" 结尾的数据
                try:
                    # 查找 "cd" 的位置
                    cd_index = line.find("cd")
                    if cd_index == -1:
                        print(f"第 {i+1} 行未找到 'cd'")
                        continue
                    
                    # 获取从 "cd" 到行尾的字符串
                    data_str = line[cd_index:]
                    
                    # 检查是否以 "dc" 结尾
                    if not data_str.endswith("dc"):
                        # 如果不是以dc结尾，尝试查找最后一个dc
                        if " dc" in data_str:
                            # 找到最后一个" dc"的位置，并包含dc
                            last_dc_index = data_str.rfind(" dc")
                            if last_dc_index != -1:
                                data_str = data_str[:last_dc_index + 4]  # 包括" dc"
                    
                    # 将十六进制字符串转换为字节
                    hex_str = data_str.replace(" ", "")
                    try:
                        # 检查是否是有效的十六进制字符串
                        if len(hex_str) % 2 != 0:
                            print(f"第 {i+1} 行十六进制长度错误: {hex_str}")
                            continue
                            
                        data_bytes = bytes.fromhex(hex_str)
                        
                        # 4. 通过串口发送
                        ser.write(data_bytes)
                        print(f"发送第 {i+1} 行: {len(data_bytes)} 字节")
                        
                        # 可选：添加延迟以避免发送过快
                        time.sleep(0.00001)
                        
                    except ValueError as e:
                        print(f"第 {i+1} 行十六进制转换错误: {e}")
                        
                except Exception as e:
                    print(f"第 {i+1} 行处理错误: {e}")
                    continue
        
        print("数据发送完成")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print("请检查串口号是否正确，或设备是否连接")
    except FileNotFoundError:
        print(f"文件不存在: {file_path}")
    except Exception as e:
        print(f"未知错误: {e}")
    finally:
        # 确保关闭串口
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")

# 更稳健的版本：使用正则表达式提取

def send_data_from_file_regex(file_path, com_port, baud_rate=115200):
    """
    使用正则表达式提取数据并通过串口发送
    """
    try:
        # 初始化串口
        ser = serial.Serial(com_port, baud_rate, timeout=1)
        print(f"串口 {com_port} 已打开，波特率 {baud_rate}")
        
        # 读取文件
        with open(file_path, 'r') as file:
            content = file.read()
            
            # 使用正则表达式匹配所有 cd 开头 dc 结尾的数据
            # 模式说明：匹配 "cd" 后跟任意字符，直到 "dc"（非贪婪匹配）
            pattern = r'cd(?: [0-9a-fA-F]{2})+ dc'
            matches = re.findall(pattern, content)
            
            print(f"共找到 {len(matches)} 条匹配的数据")
            
            for i, match in enumerate(matches):
                try:
                    # 去掉空格，转换为十六进制字节
                    hex_str = match.replace(" ", "")
                    data_bytes = bytes.fromhex(hex_str)
                    
                    # 通过串口发送
                    ser.write(data_bytes)
                    print(f"发送第 {i+1} 条: {len(data_bytes)} 字节")
                    
                    # 可选延迟
                    time.sleep(1/50)
                    
                except Exception as e:
                    print(f"第 {i+1} 条数据处理错误: {e}")
                    continue
        
        print("数据发送完成")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except FileNotFoundError:
        print(f"文件不存在: {file_path}")
    except Exception as e:
        print(f"未知错误: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")

# 使用示例
if __name__ == "__main__":
    # 配置参数
    FILE_PATH = "/home/luoxu/zuomian/1_ws/RM_6/1/Horizon_Rm_Vision_26_SP/txt/ori_received.txt"  # 你的文件路径
    COM_PORT = "/dev/pts/5"               # Windows串口号，Linux/macOS可能是 "/dev/ttyUSB0" 或 "/dev/ttyACM0"
    BAUD_RATE = 115200              # 波特率，根据你的设备设置
    
    # 方法1：使用基本方法
    #send_data_from_file(FILE_PATH, COM_PORT, BAUD_RATE)
    
    # 或者使用方法2：使用正则表达式（更稳健）
    send_data_from_file_regex(FILE_PATH, COM_PORT, BAUD_RATE)
