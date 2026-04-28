#!/usr/bin/env python3
"""
ROS2 四轮小车控制节点。

功能:
- 订阅 geometry_msgs/msg/Twist 的 /cmd_vel
- 将线速度/角速度转换为左右轮速度
- 通过串口向四个电机发送控制指令
- 发布估计轮速，供独立里程计节点使用

说明:
- 当前底盘串口协议已知的下行控制命令为 JSON 格式:
  {"T": 10010, "id": 电机ID, "cmd": 速度, "act": 加速时间}
- 若底盘串口存在上行速度反馈，可在 parse_feedback_line() 中按实际协议补充解析。
"""

import json
import threading
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

import serial
import serial.tools.list_ports


SERIAL_PORT = "/dev/chassis_serial_port"
BAUD_RATE = 115200

WHEEL_IDS = {
    "left_front_wheel": 1,
    "right_front_wheel": 2,
    "left_rear_wheel": 3,
    "right_rear_wheel": 4,
}

WHEEL_ORDER = (
    "left_front_wheel",
    "right_front_wheel",
    "left_rear_wheel",
    "right_rear_wheel",
)

RIGHT_WHEELS = {"right_front_wheel", "right_rear_wheel"}

MAX_COMMAND = 100


class CarController(Node):
    def __init__(self) -> None:
        super().__init__("car_controller")

        self.declare_parameter("serial_port", SERIAL_PORT)
        self.declare_parameter("baud_rate", BAUD_RATE)
        self.declare_parameter("track_width", 0.30)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("command_scale", 50.0)
        self.declare_parameter("max_command", MAX_COMMAND)
        self.declare_parameter("acceleration_time", 3)
        self.declare_parameter("prefer_pivot_turn", True)
        self.declare_parameter("pivot_turn_angular_threshold", 0.01)
        self.declare_parameter("min_pivot_wheel_speed", 0.08)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_speed_topic", "/wheel_speeds")
        self.declare_parameter("joint_state_topic", "/wheel_joint_states")
        self.declare_parameter("feedback_timeout", 0.5)

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.command_scale = float(self.get_parameter("command_scale").value)
        self.max_command = int(self.get_parameter("max_command").value)
        self.acceleration_time = int(self.get_parameter("acceleration_time").value)
        self.prefer_pivot_turn = bool(self.get_parameter("prefer_pivot_turn").value)
        self.pivot_turn_angular_threshold = float(
            self.get_parameter("pivot_turn_angular_threshold").value
        )
        self.min_pivot_wheel_speed = float(self.get_parameter("min_pivot_wheel_speed").value)
        self.feedback_timeout = float(self.get_parameter("feedback_timeout").value)

        wheel_speed_topic = self.get_parameter("wheel_speed_topic").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.serial_conn: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.running = True

        self.latest_commanded = {name: 0.0 for name in WHEEL_ORDER}
        self.latest_feedback = {name: 0.0 for name in WHEEL_ORDER}
        self.last_feedback_time = self.get_clock().now()

        self.wheel_speed_pub = self.create_publisher(Float32MultiArray, wheel_speed_topic, 10)
        self.joint_state_pub = self.create_publisher(JointState, joint_state_topic, 10)

        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.get_logger().info(
            f"控制节点启动: 订阅 {cmd_vel_topic}, 发布 {wheel_speed_topic} 和 {joint_state_topic}"
        )

        self.init_serial_connection()
        if self.serial_conn is not None:
            self.serial_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.serial_thread.start()

    def init_serial_connection(self) -> None:
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1,
            )
            self.get_logger().info(f"已连接到底盘串口 {self.serial_port}")
        except serial.SerialException as exc:
            self.serial_conn = None
            self.get_logger().error(f"底盘串口打开失败: {exc}")
            self.list_available_ports()

    def list_available_ports(self) -> None:
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.get_logger().warn("未检测到可用串口")
            return
        for port in ports:
            self.get_logger().info(f"可用串口: {port.device} ({port.description})")

    def cmd_vel_callback(self, msg: Twist) -> None:
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        wheel_linear = self.compute_wheel_linear_speeds(linear_x, angular_z)
        wheel_command = self.compute_wheel_commands(wheel_linear)

        self.send_wheel_commands(wheel_command)
        self.publish_wheel_states(wheel_linear)

    def compute_wheel_linear_speeds(self, linear_x: float, angular_z: float) -> Dict[str, float]:
        half_track = self.track_width / 2.0
        if self.prefer_pivot_turn and abs(angular_z) > self.pivot_turn_angular_threshold:
            turn_direction = 1.0 if angular_z > 0.0 else -1.0
            turn_speed = max(abs(angular_z) * half_track, self.min_pivot_wheel_speed)
            left_speed = -turn_direction * turn_speed
            right_speed = turn_direction * turn_speed
        else:
            left_speed = linear_x - angular_z * half_track
            right_speed = linear_x + angular_z * half_track

        wheel_linear = {
            "left_front_wheel": left_speed,
            "right_front_wheel": right_speed,
            "left_rear_wheel": left_speed,
            "right_rear_wheel": right_speed,
        }
        self.latest_commanded = wheel_linear.copy()
        return wheel_linear

    def compute_wheel_commands(self, wheel_linear: Dict[str, float]) -> Dict[str, int]:
        commands: Dict[str, int] = {}
        for wheel_name, velocity_mps in wheel_linear.items():
            raw_command = int(round(velocity_mps * self.command_scale))
            commands[wheel_name] = max(-self.max_command, min(self.max_command, raw_command))
        return commands

    def create_motor_command(self, wheel_name: str, command: int) -> Dict[str, int]:
        signed_command = -command if wheel_name in RIGHT_WHEELS else command
        return {
            "T": 10010,
            "id": WHEEL_IDS[wheel_name],
            "cmd": signed_command,
            "act": self.acceleration_time,
        }

    def send_wheel_commands(self, wheel_command: Dict[str, int]) -> None:
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn("底盘串口未连接，已仅发布 ROS 轮速状态")
            return

        with self.serial_lock:
            try:
                for wheel_name in WHEEL_ORDER:
                    command = self.create_motor_command(wheel_name, wheel_command[wheel_name])
                    payload = json.dumps(command, ensure_ascii=True) + "\n"
                    self.serial_conn.write(payload.encode("utf-8"))
                self.serial_conn.flush()
            except serial.SerialException as exc:
                self.get_logger().error(f"串口发送失败: {exc}")

    def read_serial_loop(self) -> None:
        while rclpy.ok() and self.running and self.serial_conn is not None:
            try:
                line = self.serial_conn.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f"串口读取失败: {exc}")
                return

            if not line:
                continue

            parsed = self.parse_feedback_line(line.decode("utf-8", errors="ignore").strip())
            if parsed is None:
                continue

            self.latest_feedback.update(parsed)
            self.last_feedback_time = self.get_clock().now()
            self.publish_wheel_states(self.latest_feedback)

    def parse_feedback_line(self, line: str) -> Optional[Dict[str, float]]:
        """
        预留底盘速度反馈解析。

        支持以下两种常见 JSON 格式:
        1. {"left_front_wheel": 0.1, "right_front_wheel": 0.1, ...}
        2. {"wheel_speeds": {"left_front_wheel": 0.1, ...}}

        单位约定为 m/s。如果实际底盘返回的不是这个格式，请在这里按真实协议修改。
        """
        if not line:
            return None

        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return None

        if isinstance(data, dict) and "wheel_speeds" in data and isinstance(data["wheel_speeds"], dict):
            data = data["wheel_speeds"]

        if not isinstance(data, dict):
            return None

        result: Dict[str, float] = {}
        for wheel_name in WHEEL_ORDER:
            if wheel_name in data:
                result[wheel_name] = float(data[wheel_name])

        return result if result else None

    def get_best_wheel_state(self) -> Dict[str, float]:
        age = (self.get_clock().now() - self.last_feedback_time).nanoseconds / 1e9
        if age <= self.feedback_timeout:
            return self.latest_feedback.copy()
        return self.latest_commanded.copy()

    def publish_wheel_states(self, wheel_linear: Optional[Dict[str, float]] = None) -> None:
        wheel_linear = self.get_best_wheel_state() if wheel_linear is None else wheel_linear

        speeds_msg = Float32MultiArray()
        speeds_msg.data = [float(wheel_linear[name]) for name in WHEEL_ORDER]
        self.wheel_speed_pub.publish(speeds_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = list(WHEEL_ORDER)
        joint_msg.position = []
        joint_msg.effort = []
        joint_msg.velocity = [
            float(wheel_linear[name] / self.wheel_radius) if self.wheel_radius > 0.0 else 0.0
            for name in WHEEL_ORDER
        ]
        self.joint_state_pub.publish(joint_msg)

    def stop_vehicle(self) -> None:
        stop_command = {wheel_name: 0 for wheel_name in WHEEL_ORDER}
        self.latest_commanded = {wheel_name: 0.0 for wheel_name in WHEEL_ORDER}
        self.latest_feedback = {wheel_name: 0.0 for wheel_name in WHEEL_ORDER}
        self.send_wheel_commands(stop_command)
        self.publish_wheel_states(self.latest_commanded)

    def close_serial(self) -> None:
        self.running = False
        if self.serial_conn is not None and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except serial.SerialException as exc:
                self.get_logger().error(f"关闭串口失败: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
