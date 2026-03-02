#!/usr/bin/env python3
"""
云台电控模拟程序
模拟接收视觉程序数据并返回云台状态
"""
#除了可以两个实体串口自收自发，也可以用虚拟串口 socat -d -d pty,b115200 pty,b115200

import serial
import struct
import time
import threading
import logging
from typing import Optional, Tuple
from dataclasses import dataclass

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("GimbalSimulator")

@dataclass
class VisionToGimbal:
    """视觉发送给云台的数据结构"""
    head: int = 0xCD
    pitch: float = 0.0
    yaw: float = 0.0
    mode: int = 0
    timestamp: int = 0
    tail: int = 0xDC

@dataclass
class GimbalToVision:
    """云台发送给视觉的数据结构"""
    head: int = 0xCD
    pitch: float = 0.0
    yaw: float = 0.0
    mode: int = 1  # 固定为自瞄模式
    timestamp: int = 0
    bullet_speed: int = 10  # 模拟弹速
    tail: int = 0xDC

class GimbalSimulator:
    """云台模拟器"""
    
    def __init__(self, serial_port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        
        # 云台状态
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.mode = 0  # 自瞄模式
        
        # 线程控制
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        self.send_thread: Optional[threading.Thread] = None
        
        # 数据包大小
        self.rx_packet_size = 15  # VisionToGimbal 大小
        self.tx_packet_size = 16  # GimbalToVision 大小
        
        logger.info(f"初始化云台模拟器，串口: {serial_port}, 波特率: {baudrate}")
    
    def open_serial(self) -> bool:
        """打开串口"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # 读取超时
                write_timeout=1.0  # 写入超时
            )
            logger.info(f"成功打开串口: {self.serial_port}")
            return True
        except Exception as e:
            logger.error(f"打开串口失败: {e}")
            return False
    
    def close_serial(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")
    
    def parse_vision_data(self, data: bytes) -> Optional[VisionToGimbal]:
        """解析视觉数据包"""
        if len(data) != self.rx_packet_size:
            logger.warning(f"数据包长度错误: {len(data)} != {self.rx_packet_size}")
            return None
        
        try:
            # 解析数据包: CD + pitch(4) + yaw(4) + mode(1) + timestamp(4) + DC
            unpacked = struct.unpack('<BffBIB', data)
            
            # 检查包头包尾
            if unpacked[0] != 0xCD or unpacked[-1] != 0xDC:
                logger.warning("包头或包尾错误")
                return None
            
            packet = VisionToGimbal(
                head=unpacked[0],
                pitch=unpacked[1],
                yaw=unpacked[2],
                mode=unpacked[3],
                timestamp=unpacked[4],
                tail=unpacked[5]
            )
            
            return packet
            
        except Exception as e:
            logger.error(f"解析数据包失败: {e}")
            return None
    
    def create_gimbal_packet(self) -> bytes:
        """创建云台数据包"""
        try:
            # 使用当前时间戳
            timestamp = int(time.time() * 1000) & 0xFFFFFFFF
            
            packet_data = struct.pack(
                '<BffBIBB',
                0xCD,  # head
                self.current_pitch,
                self.current_yaw,
                self.mode,  # 固定为自瞄模式
                timestamp,
                10,  # bullet_speed
                0xDC  # tail
            )
            
            return packet_data
            
        except Exception as e:
            logger.error(f"创建数据包失败: {e}")
            return b''
    
    def process_vision_command(self, vision_data: VisionToGimbal):
        """处理视觉指令"""
        logger.info(f"收到视觉数据 - 模式: {vision_data.mode}, Pitch: {vision_data.pitch:.3f}, Yaw: {vision_data.yaw:.3f}")
        
        # 如果模式是57（控制且开火），更新云台位置
        if vision_data.mode == 57:
            self.current_pitch = vision_data.pitch
            self.current_yaw = vision_data.yaw
            logger.info(f"更新云台位置 - Pitch: {self.current_pitch:.3f}, Yaw: {self.current_yaw:.3f}")
        else:
            # 其他模式保持当前位置不变
            logger.info("保持当前云台位置")
    
    def receive_data(self):
        """接收数据线程"""
        buffer = bytearray()
        
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue
            
            try:
                # 读取数据
                data = self.ser.read(1024)
                if data:
                    buffer.extend(data)
                    logger.debug(f"收到原始数据: {data.hex()}")
                    
                    # 处理缓冲区中的数据包
                    while len(buffer) >= self.rx_packet_size:
                        # 查找包头
                        if buffer[0] != 0xCD:
                            # 包头不匹配，丢弃第一个字节
                            buffer.pop(0)
                            continue
                        
                        # 检查是否有完整数据包
                        if len(buffer) < self.rx_packet_size:
                            break
                        
                        # 提取数据包
                        packet_data = bytes(buffer[:self.rx_packet_size])
                        
                        # 检查包尾
                        if packet_data[-1] != 0xDC:
                            # 包尾不匹配，丢弃第一个字节继续查找
                            buffer.pop(0)
                            continue
                        
                        # 解析数据包
                        vision_data = self.parse_vision_data(packet_data)
                        if vision_data:
                            self.process_vision_command(vision_data)
                        
                        # 移除已处理的数据
                        buffer = buffer[self.rx_packet_size:]
                        
            except Exception as e:
                logger.error(f"接收数据错误: {e}")
                time.sleep(0.1)
    
    def send_data(self):
        """发送数据线程"""
        send_interval = 0.02  # 50Hz发送频率
        
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue
            
            try:
                # 创建并发送云台数据包
                packet = self.create_gimbal_packet()
                if packet:
                    self.ser.write(packet)
                    logger.debug(f"发送云台数据 - Pitch: {self.current_pitch:.3f}, Yaw: {self.current_yaw:.3f}, 模式: {self.mode}")
                
                # 控制发送频率
                time.sleep(send_interval)
                
            except Exception as e:
                logger.error(f"发送数据错误: {e}")
                time.sleep(0.1)
    
    def start(self):
        """启动模拟器"""
        if not self.open_serial():
            logger.error("无法启动模拟器：串口打开失败")
            return False
        
        self.running = True
        
        # 启动接收线程
        self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.receive_thread.start()
        
        # 启动发送线程
        self.send_thread = threading.Thread(target=self.send_data, daemon=True)
        self.send_thread.start()
        
        logger.info("云台模拟器已启动")
        return True
    
    def stop(self):
        """停止模拟器"""
        self.running = False
        
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        
        if self.send_thread:
            self.send_thread.join(timeout=1.0)
        
        self.close_serial()
        logger.info("云台模拟器已停止")
    
    def run_interactive(self):
        """交互式运行"""
        try:
            self.start()
            
            print("云台模拟器运行中...")
            print("当前状态:")
            print(f"  Pitch: {self.current_pitch:.3f}")
            print(f"  Yaw: {self.current_yaw:.3f}")
            print(f"  模式: {self.mode} (自瞄模式)")
            print("按Ctrl+C停止")
            
            # 主循环
            while self.running:
                time.sleep(1)
                # 可以在这里添加状态显示或其他交互功能
                
        except KeyboardInterrupt:
            print("\n正在停止模拟器...")
        finally:
            self.stop()

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='云台电控模拟程序')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB0', 
                       help='串口设备路径 (默认: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                       help='波特率 (默认: 115200)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='详细日志输出')
    
    args = parser.parse_args()
    
    # 设置日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # 创建并运行模拟器
    simulator = GimbalSimulator(args.port, args.baudrate)
    simulator.run_interactive()

if __name__ == "__main__":
    main()
