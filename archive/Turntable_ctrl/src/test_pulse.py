import argparse
import os
import struct
import sys
import time

import serial

if os.name == "nt":
    import msvcrt
else:
    import select
    import termios
    import tty


DEFAULT_PORT = "COM9" if os.name == "nt" else "/dev/lift_port"


class ConsoleKeyReader:
    def __init__(self):
        self._fd = None
        self._old_settings = None

    def __enter__(self):
        if os.name != "nt":
            if not sys.stdin.isatty():
                raise RuntimeError("交互模式需要终端 TTY；可改用 --smoke-test 做连通性验证。")
            self._fd = sys.stdin.fileno()
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old_settings is not None and self._fd is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def read_key(self):
        if os.name == "nt":
            if not msvcrt.kbhit():
                return None
            key = msvcrt.getwch()
            if key in ("\x00", "\xe0") and msvcrt.kbhit():
                msvcrt.getwch()
                return None
            return key

        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if ready:
            return sys.stdin.read(1)
        return None


class YZAIM_Motor:
    def __init__(self, port=DEFAULT_PORT, baudrate=19200, slave_id=1):
        """
        初始化电机通信
        根据手册P9，默认通信参数为: 19200, 8, N, 1
        :param port: 串口号，Linux 默认 /dev/lift_port，Windows 默认 COM9
        :param baudrate: 波特率，默认为19200
        :param slave_id: 设备地址，默认为1
        """
        self.port = port
        self.slave_id = slave_id
        self.ser = None

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            if self.ser.is_open:
                print(f"成功连接电机于端口 {port}")
            else:
                print("串口打开失败")
                return
        except Exception as e:
            print(f"打开串口时发生错误: {e}")
            self.ser = None
            return

        # ===== 速度参数修改区域 =====
        self.test_speed = 2000
        self.test_accel = 10000
        self.pulse_per_move = 500
        self.send_interval = 0.01
        # ============================

        self._initial_setup()

    def _crc16(self, data: bytes):
        """
        CRC16-Modbus 校验计算
        代码完全根据手册P12-P13的C代码示例转换而来。
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc.to_bytes(2, byteorder="little")

    def _send_modbus_command(self, function_code, register_address, data_value=None, is_write_single=True):
        """
        构造并发送Modbus命令，并读取回复（如果必要）。
        :param function_code: 功能码
        :param register_address: 寄存器地址
        :param data_value: 要写入的数据值（对于写命令）
        :param is_write_single: True为写单个寄存器(0x06)，False为写多个寄存器(0x10)或读(0x03)
        :return: 从机回复的字节数据
        """
        if not self.ser:
            return None

        if is_write_single and data_value is not None:
            frame = bytearray()
            frame.append(self.slave_id)
            frame.append(function_code)
            frame.extend(register_address.to_bytes(2, byteorder="big"))
            frame.extend(data_value.to_bytes(2, byteorder="big"))
        elif not is_write_single and data_value is not None and function_code == 0x10:
            frame = bytearray()
            frame.append(self.slave_id)
            frame.append(0x10)
            frame.extend(struct.pack(">H", 0x000C))
            frame.extend(struct.pack(">H", 0x0002))
            frame.append(0x04)
            if data_value < 0:
                data_value = 0xFFFFFFFF + data_value + 1
            pu_8_15 = (data_value >> 8) & 0xFF
            pu_0_7 = data_value & 0xFF
            pu_24_31 = (data_value >> 24) & 0xFF
            pu_16_23 = (data_value >> 16) & 0xFF
            frame.append(pu_8_15)
            frame.append(pu_0_7)
            frame.append(pu_24_31)
            frame.append(pu_16_23)
        else:
            return None

        frame.extend(self._crc16(frame))
        self.ser.write(frame)
        time.sleep(0.01)
        return self.ser.read(8)

    def _initial_setup(self):
        """
        进行驱动器初始化设置，使其进入准备状态。
        步骤遵循手册P13“modbus方式主机控制过程”的说明，但参数调整为测试用小值。
        """
        if not self.ser:
            return

        print("正在进行电机初始化设置...")
        print("  步骤1: 使能Modbus控制...")
        self._send_modbus_command(0x06, 0x0000, 1)
        time.sleep(0.05)

        print(f"  步骤2: 设置加速度为 {self.test_accel} (r/min)/s...")
        self._send_modbus_command(0x06, 0x0003, self.test_accel)
        time.sleep(0.05)

        print(f"  步骤3: 设置目标速度为 {self.test_speed} RPM...")
        self._send_modbus_command(0x06, 0x0002, self.test_speed)
        time.sleep(0.05)

        print("  步骤4: 使能驱动器输出...")
        self._send_modbus_command(0x06, 0x0001, 1)
        time.sleep(0.1)

        print("初始化设置完成。电机已准备就绪。")
        print("提示: 'A' 键正向运动, 'S' 键反向运动, 'Q' 键停止, 'ESC' 键退出程序。")

    def move_incremental(self, pulses):
        """
        发送增量位置指令，使电机移动指定的脉冲数。
        使用功能码 0x10，写入目标位置寄存器（地址0x0C, 0x0D）。
        :param pulses: 有符号整数，正数为正转，负数为反转。
        """
        self._send_modbus_command(0x10, 0x000C, pulses, is_write_single=False)

    def stop_motor(self):
        """
        停止电机。
        这里采用发送0增量指令，使电机目标位置不变，从而停止。
        """
        self.move_incremental(0)

    def run_interactive_test(self):
        """
        运行交互式测试：按A键持续正向运动，按S键持续反向运动，按Q键停止，按ESC退出。
        """
        if not self.ser:
            return

        current_direction = 0
        last_send_time = 0
        print("开始按键控制测试...")
        print("提示: 'A' 键正向运动, 'S' 键反向运动, 'Q' 键停止, 'ESC' 键退出程序。")

        try:
            with ConsoleKeyReader() as key_reader:
                while True:
                    current_time = time.time()
                    key = key_reader.read_key()

                    if key:
                        key_lower = key.lower()
                        if key_lower == "a":
                            if current_direction != 1:
                                print("[正向运动]")
                                current_direction = 1
                        elif key_lower == "s":
                            if current_direction != -1:
                                print("[反向运动]")
                                current_direction = -1
                        elif key_lower == "q":
                            if current_direction != 0:
                                print("[停止]")
                                current_direction = 0
                                self.stop_motor()
                        elif key == "\x1b":
                            print("收到退出指令。")
                            break

                    if current_direction != 0 and current_time - last_send_time >= self.send_interval:
                        self.move_incremental(self.pulse_per_move * current_direction)
                        last_send_time = current_time

                    time.sleep(0.005)
        except KeyboardInterrupt:
            print("程序被用户中断。")
        finally:
            self.cleanup()

    def run_smoke_test(self, hold_seconds=0.2):
        if not self.ser:
            return

        print(f"开始非交互冒烟测试，端口 {self.port}，等待 {hold_seconds:.1f} 秒后退出。")
        try:
            time.sleep(max(hold_seconds, 0))
        finally:
            self.cleanup()

    def cleanup(self):
        """程序结束前清理，停止电机并关闭串口"""
        print("正在停止电机并关闭串口...")
        if self.ser and self.ser.is_open:
            self.stop_motor()
            time.sleep(0.1)
            self._send_modbus_command(0x06, 0x0001, 0)
            self.ser.close()
        print("程序退出。")


def parse_args():
    parser = argparse.ArgumentParser(description="YZAIM 电机串口脉冲测试")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"串口设备，默认 {DEFAULT_PORT}")
    parser.add_argument("--baudrate", type=int, default=19200, help="串口波特率，默认 19200")
    parser.add_argument("--slave-id", type=int, default=1, help="Modbus 从站地址，默认 1")
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="仅做非交互连通性验证，完成初始化后短暂停留并退出",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.2,
        help="冒烟测试停留时间，默认 0.2 秒",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    motor = YZAIM_Motor(port=args.port, baudrate=args.baudrate, slave_id=args.slave_id)

    if not motor.ser:
        print("电机初始化失败，请检查串口连接。")
        return 1

    if args.smoke_test:
        motor.run_smoke_test(args.hold_seconds)
    else:
        motor.run_interactive_test()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
