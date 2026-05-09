#!/usr/bin/env python3
"""
云台电控模拟程序

通信模式说明：
- 普通模式 (NORMAL=0): 基础通信，不包含扩展数据
    VisionToGimbal: head(1) + pitch(4) + yaw(4) + mode(1) + tail(1) = 11字节
    GimbalToVision: head(1) + pitch(4) + yaw(4) + mode(1) + bullet_speed(1) + tail(1) = 12字节

- 只启用SR_VEL (SR_VEL_ONLY=1): 包含云台前馈角速度数据
    VisionToGimbal: 基础 + pitch_vel(4) + yaw_vel(4) = 19字节
    GimbalToVision: 基础 + pitch_vel(4) + yaw_vel(4) = 20字节
  扩展数据填0

- 只启用SENTRY_SR (SENTRY_SR_ONLY=2): 包含哨兵相关数据
    VisionToGimbal: 基础 + vx(4) + vy(4) + wz(4) + form(1) + gimbal(1) = 25字节
    GimbalToVision: 基础 + game_progress(1) + stage_remain_time(2) + current_hp(2) + ally_outpost_hp(2)
                  + state(1) + energy_state(1) + bullets(2) = 23字节
  扩展数据填0

- 同时启用两者 (BOTH_ENABLED=3): 包含所有扩展数据
    VisionToGimbal: 基础 + pitch_vel(4) + yaw_vel(4) + vx(4) + vy(4) + wz(4) + form(1) + gimbal(1) = 33字节
    GimbalToVision: 基础 + pitch_vel(4) + yaw_vel(4) + game_progress(1) + stage_remain_time(2)
                  + current_hp(2) + ally_outpost_hp(2) + state(1) + energy_state(1) + bullets(2) = 31字节
  扩展数据填0

新的使用方法：
python3 SPSREMU_V9.py --mode 0  --p=/dev/ttyUSB0 # 普通模式
python3 SPSREMU_V9.py --mode 1  --p=/dev/ttyUSB0 # 只启用SR_VEL
python3 SPSREMU_V9.py --mode 2  --p=/dev/ttyUSB0 # 只启用SENTRY_SR
python3 SPSREMU_V9.py --mode 3  --p=/dev/ttyUSB0 # 同时启用两者
"""
#除了可以两个实体串口自收自发，也可以用虚拟串口 socat -d -d pty,b115200 pty,b115200

import serial
import struct
import time
import threading
import logging
from typing import Optional
from dataclasses import dataclass
from enum import Enum

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("GimbalSimulator")

class CommunicationMode(Enum):
    """通信模式枚举"""
    NORMAL = 0          # 普通模式
    SR_VEL_ONLY = 1     # 只启用SR_VEL
    SENTRY_SR_ONLY = 2  # 只启用SENTRY_SR
    BOTH_ENABLED = 3    # 同时启用SR_VEL和SENTRY_SR

@dataclass
class VisionToGimbal:
    """视觉发送给云台的数据结构（与 gimbal.hpp 的 VisionToGimbal 一致）"""
    head: int = 0xCD
    pitch: float = 0.0
    yaw: float = 0.0
    mode: int = 0
    # SR_VEL扩展字段
    pitch_vel: float = 0.0
    yaw_vel: float = 0.0
    # SENTRY_SR扩展字段
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    form: int = 0
    gimbal: int = 0        # 云台编号，预留字段
    tail: int = 0xDC

@dataclass
class GimbalToVision:
    """云台发送给视觉的数据结构（与 gimbal.hpp 的 GimbalToVision 一致）"""
    head: int = 0xCD
    pitch: float = 0.0
    yaw: float = 0.0
    mode: int = 1  # 固定为自瞄模式
    bullet_speed: int = 150  # 模拟弹速
    # SR_VEL扩展字段
    pitch_vel: float = 0.0
    yaw_vel: float = 0.0
    # SENTRY_SR扩展字段（与 gimbal.hpp 一致）
    game_progress: int = 0
    stage_remain_time: int = 0
    current_hp: int = 0
    ally_outpost_hp: int = 0
    state: int = 0
    energy_state: int = 0
    bullets: int = 0         # 子弹数量（uint16_t，与 gimbal.hpp 一致）
    tail: int = 0xDC

class GimbalSimulator:
    """云台模拟器"""

    def __init__(self, serial_port: str = "/dev/ttyUSB0", baudrate: int = 115200, comm_mode: CommunicationMode = CommunicationMode.NORMAL):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.comm_mode = comm_mode

        self.ser: Optional[serial.Serial] = None

        # 云台状态
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.mode = 0  # 自瞄模式

        # 线程控制
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        self.send_thread: Optional[threading.Thread] = None

        # 根据通信模式计算数据包大小
        self.rx_packet_size = self._calculate_rx_packet_size()
        self.tx_packet_size = self._calculate_tx_packet_size()

        logger.info(f"初始化云台模拟器，串口: {serial_port}, 波特率: {baudrate}, 通信模式: {comm_mode.name}")

    def _calculate_rx_packet_size(self) -> int:
        """计算接收数据包大小（VisionToGimbal）"""
        # CD(1) + pitch(4) + yaw(4) + mode(1) + DC(1) = 11
        base_size = 11

        if self.comm_mode in [CommunicationMode.SR_VEL_ONLY, CommunicationMode.BOTH_ENABLED]:
            base_size += 8  # pitch_vel(4) + yaw_vel(4)

        if self.comm_mode in [CommunicationMode.SENTRY_SR_ONLY, CommunicationMode.BOTH_ENABLED]:
            base_size += 14  # vx(4) + vy(4) + wz(4) + form(1) + gimbal(1)

        return base_size

    def _calculate_tx_packet_size(self) -> int:
        """计算发送数据包大小（GimbalToVision）"""
        # CD(1) + pitch(4) + yaw(4) + mode(1) + bullet_speed(1) + DC(1) = 12
        base_size = 12

        if self.comm_mode in [CommunicationMode.SR_VEL_ONLY, CommunicationMode.BOTH_ENABLED]:
            base_size += 8  # pitch_vel(4) + yaw_vel(4)

        if self.comm_mode in [CommunicationMode.SENTRY_SR_ONLY, CommunicationMode.BOTH_ENABLED]:
            base_size += 11  # game_progress(1) + stage_remain_time(2) + current_hp(2) + ally_outpost_hp(2)
                            # + state(1) + energy_state(1) + bullets(2)

        return base_size

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
            # 基础格式: CD + pitch(4) + yaw(4) + mode(1)
            base_format = '<BffB'

            # 根据通信模式构建格式字符串
            format_str = base_format

            if self.comm_mode in [CommunicationMode.SR_VEL_ONLY, CommunicationMode.BOTH_ENABLED]:
                format_str += 'ff'  # pitch_vel(4) + yaw_vel(4)

            if self.comm_mode in [CommunicationMode.SENTRY_SR_ONLY, CommunicationMode.BOTH_ENABLED]:
                format_str += 'fffbb'  # vx(4) + vy(4) + wz(4) + form(1) + gimbal(1)

            format_str += 'B'  # tail(1)

            unpacked = struct.unpack(format_str, data)

            # 检查包头包尾
            if unpacked[0] != 0xCD or unpacked[-1] != 0xDC:
                logger.warning("包头或包尾错误")
                return None

            # 创建数据包对象
            packet = VisionToGimbal()
            packet.head = unpacked[0]
            packet.pitch = unpacked[1]
            packet.yaw = unpacked[2]
            packet.mode = unpacked[3]

            index = 4
            if self.comm_mode in [CommunicationMode.SR_VEL_ONLY, CommunicationMode.BOTH_ENABLED]:
                packet.pitch_vel = unpacked[index]
                packet.yaw_vel = unpacked[index + 1]
                index += 2

            if self.comm_mode in [CommunicationMode.SENTRY_SR_ONLY, CommunicationMode.BOTH_ENABLED]:
                packet.vx = unpacked[index]
                packet.vy = unpacked[index + 1]
                packet.wz = unpacked[index + 2]
                packet.form = unpacked[index + 3]
                packet.gimbal = unpacked[index + 4]
                index += 5

            packet.tail = unpacked[-1]

            return packet

        except Exception as e:
            logger.error(f"解析数据包失败: {e}")
            return None

    def create_gimbal_packet(self) -> bytes:
        """创建云台数据包（与 gimbal.hpp 的 GimbalToVision 结构一致）"""
        try:
            # 基础数据
            data_list = [
                0xCD,  # head
                self.current_pitch,
                self.current_yaw,
                self.mode,  # 固定为自瞄模式
                150,  # bullet_speed
            ]

            # 基础格式
            format_str = '<BffBB'

            # 根据通信模式添加扩展数据
            if self.comm_mode in [CommunicationMode.SR_VEL_ONLY, CommunicationMode.BOTH_ENABLED]:
                data_list.extend([0.0, 0.0])  # pitch_vel, yaw_vel (填0)
                format_str += 'ff'

            if self.comm_mode in [CommunicationMode.SENTRY_SR_ONLY, CommunicationMode.BOTH_ENABLED]:
                # game_progress(1) + stage_remain_time(2) + current_hp(2) + ally_outpost_hp(2)
                # + state(1) + energy_state(1) + bullets(2)
                data_list.extend([0, 0, 0, 0, 0, 0, 0])
                format_str += 'BHHHBBH'

            data_list.append(0xDC)  # tail
            format_str += 'B'

            packet_data = struct.pack(format_str, *data_list)

            return packet_data

        except Exception as e:
            logger.error(f"创建数据包失败: {e}")
            return b''

    def process_vision_command(self, vision_data: VisionToGimbal):
        """处理视觉指令"""
        log_msg = f"收到视觉数据 - 模式: {vision_data.mode}, Pitch: {vision_data.pitch:.3f}, Yaw: {vision_data.yaw:.3f}"

        if self.comm_mode in [CommunicationMode.SR_VEL_ONLY, CommunicationMode.BOTH_ENABLED]:
            log_msg += f", PitchVel: {vision_data.pitch_vel:.3f}, YawVel: {vision_data.yaw_vel:.3f}"

        if self.comm_mode in [CommunicationMode.SENTRY_SR_ONLY, CommunicationMode.BOTH_ENABLED]:
            log_msg += f", Vx: {vision_data.vx:.3f}, Vy: {vision_data.vy:.3f}, Wz: {vision_data.wz:.3f}, Form: {vision_data.form}, Gimbal: {vision_data.gimbal}"

        logger.info(log_msg)

        # 如果模式是57（控制且开火），更新云台位置
        if vision_data.mode == 57:
            self.current_pitch = 0
            self.current_yaw = 0
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
        send_interval = 0.001  # 1000Hz发送频率

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
            print(f"  通信模式: {self.comm_mode.name}")
            print(f"  RX数据包大小: {self.rx_packet_size} 字节")
            print(f"  TX数据包大小: {self.tx_packet_size} 字节")
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
    parser.add_argument('--mode', '-m', type=int, default=0, choices=[0, 1, 2, 3],
                       help='通信模式: 0=普通模式, 1=只启用SR_VEL, 2=只启用SENTRY_SR, 3=同时启用两者 (默认: 0)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='详细日志输出')

    args = parser.parse_args()

    # 设置日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # 转换通信模式
    comm_mode = CommunicationMode(args.mode)

    # 创建并运行模拟器
    simulator = GimbalSimulator(args.port, args.baudrate, comm_mode)
    simulator.run_interactive()

if __name__ == "__main__":
    main()
