#!/usr/bin/env python3
"""ROS 2 differential drive controller for Waveshare DDSM Driver HAT (A)."""

import json
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime on robot
    serial = None


WHEEL_NAMES = [
    "left_front_wheel",
    "right_front_wheel",
    "left_rear_wheel",
    "right_rear_wheel",
]


class DDSMDriverHat:
    """Small JSON serial client for Waveshare DDSM Driver HAT (A)."""

    def __init__(self, port: str, baudrate: int, timeout: float, logger):
        if serial is None:
            raise RuntimeError("python3-serial is not installed")

        self._logger = logger
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=timeout,
            dsrdtr=None,
        )
        self._serial.setRTS(False)
        self._serial.setDTR(False)
        time.sleep(0.2)
        self._serial.reset_input_buffer()

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def send(self, command: Dict[str, int], log_debug: bool = False) -> None:
        payload = json.dumps(command, separators=(",", ":")).encode("utf-8") + b"\n"
        if log_debug:
            self._logger.debug(f"serial tx: {payload.decode('utf-8').strip()}")
        self._serial.write(payload)

    def send_batch(self, commands: List[Dict[str, int]], log_debug: bool = False) -> None:
        payload = b"".join(
            json.dumps(command, separators=(",", ":")).encode("utf-8") + b"\n"
            for command in commands
        )
        if log_debug:
            self._logger.debug(f"serial tx batch: {payload.decode('utf-8').strip()}")
        self._serial.write(payload)

    def send_commands(self, commands: List[Dict[str, int]], gap_s: float, log_debug: bool = False) -> None:
        for command in commands:
            self.send(command, log_debug)
            self.flush()
            if gap_s > 0.0:
                time.sleep(gap_s)

    def flush(self) -> None:
        self._serial.flush()

    def readline(self) -> bytes:
        return self._serial.readline()


class DDSMHatDiffDriveNode(Node):
    def __init__(self) -> None:
        super().__init__("ddsm_hat_diff_drive")

        self.declare_parameter("port", "/dev/chassis_serial_port")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_track", 0.33)
        self.declare_parameter("max_rpm", 330)
        self.declare_parameter("accel_rpm_per_sec", 600.0)
        self.declare_parameter("command_rate_hz", 30.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("heartbeat_ms", 1000)
        self.declare_parameter("hat_act", 3)
        self.declare_parameter("motor_ids", [1, 2, 3, 4])
        self.declare_parameter("motor_signs", [1, -1, 1, -1])
        self.declare_parameter("command_order", [3, 2, 1, 4])
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_speed_topic", "/wheel_speeds")
        self.declare_parameter("joint_state_topic", "/wheel_joint_states")
        self.declare_parameter("wheel_speed_source_topic", "/wheel_speed_source")
        self.declare_parameter("use_motor_feedback", True)
        self.declare_parameter("feedback_timeout", 0.25)
        self.declare_parameter("init_hat", True)
        self.declare_parameter("log_serial_tx", False)
        self.declare_parameter("log_serial_rx", False)
        self.declare_parameter("per_motor_command_gap", 0.003)
        self.declare_parameter("feedback_log_rate_hz", 0.0)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_track = float(self.get_parameter("wheel_track").value)
        self.max_rpm = int(self.get_parameter("max_rpm").value)
        self.accel_rpm_per_sec = float(self.get_parameter("accel_rpm_per_sec").value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.heartbeat_ms = int(self.get_parameter("heartbeat_ms").value)
        self.hat_act = int(self.get_parameter("hat_act").value)
        self.motor_ids = [int(x) for x in self.get_parameter("motor_ids").value]
        self.motor_signs = [int(x) for x in self.get_parameter("motor_signs").value]
        self.command_order = [int(x) for x in self.get_parameter("command_order").value]
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.wheel_speed_topic = self.get_parameter("wheel_speed_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.wheel_speed_source_topic = self.get_parameter("wheel_speed_source_topic").value
        self.use_motor_feedback = bool(self.get_parameter("use_motor_feedback").value)
        self.feedback_timeout = float(self.get_parameter("feedback_timeout").value)
        self.init_hat = bool(self.get_parameter("init_hat").value)
        self.log_serial_tx = bool(self.get_parameter("log_serial_tx").value)
        self.log_serial_rx = bool(self.get_parameter("log_serial_rx").value)
        self.per_motor_command_gap = float(self.get_parameter("per_motor_command_gap").value)
        self.feedback_log_rate_hz = float(self.get_parameter("feedback_log_rate_hz").value)

        if len(self.motor_ids) != 4 or len(self.motor_signs) != 4:
            raise RuntimeError("motor_ids and motor_signs must both contain four values")
        if sorted(self.command_order) != sorted(self.motor_ids):
            raise RuntimeError("command_order must contain the same IDs as motor_ids")
        if self.wheel_radius <= 0.0 or self.wheel_track <= 0.0:
            raise RuntimeError("wheel_radius and wheel_track must be positive")
        if self.command_rate_hz <= 0.0:
            raise RuntimeError("command_rate_hz must be positive")
        if self.per_motor_command_gap < 0.0:
            raise RuntimeError("per_motor_command_gap must be greater than or equal to zero")
        if self.feedback_log_rate_hz < 0.0:
            raise RuntimeError("feedback_log_rate_hz must be greater than or equal to zero")

        self._hat = DDSMDriverHat(
            self.port,
            self.baudrate,
            timeout=0.1,
            logger=self.get_logger(),
        )

        if self.init_hat:
            self._initialize_hat()

        self._cmd_lock = threading.Lock()
        self._linear = 0.0
        self._angular = 0.0
        self._last_cmd_time = self.get_clock().now()
        self._target_rpms = [0.0, 0.0, 0.0, 0.0]
        self._current_rpms = [0.0, 0.0, 0.0, 0.0]
        self._feedback_lock = threading.Lock()
        self._feedback_rpms: List[Optional[float]] = [None, None, None, None]
        self._feedback_times = [0.0, 0.0, 0.0, 0.0]
        self._command_lock = threading.Lock()
        self._last_commanded_rpms = {motor_id: 0 for motor_id in self.motor_ids}
        self._last_tx_time = time.monotonic()
        self._running = True

        self._wheel_speed_pub = self.create_publisher(
            Float32MultiArray, self.wheel_speed_topic, 10
        )
        self._joint_state_pub = self.create_publisher(
            JointState, self.joint_state_topic, 10
        )
        self._wheel_speed_source_pub = self.create_publisher(
            String, self.wheel_speed_source_topic, 10
        )
        self._cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10
        )
        self._feedback_log_timer = None
        if self.feedback_log_rate_hz > 0.0:
            self._feedback_log_timer = self.create_timer(
                1.0 / self.feedback_log_rate_hz,
                self._log_feedback_snapshot,
            )
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()

    def destroy_node(self) -> bool:
        self._running = False
        if hasattr(self, "_tx_thread") and self._tx_thread.is_alive():
            self._tx_thread.join(timeout=1.0)
        if hasattr(self, "_rx_thread") and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self._send_stop()
        if hasattr(self, "_hat"):
            self._hat.close()
        return super().destroy_node()

    def _initialize_hat(self) -> None:
        self._hat.send({"T": 11002, "type": 115}, self.log_serial_tx)
        self._hat.flush()
        time.sleep(0.05)
        self._hat.send({"T": 11001, "time": self.heartbeat_ms}, self.log_serial_tx)
        self._hat.flush()
        time.sleep(0.05)
        for motor_id in self.motor_ids:
            self._hat.send({"T": 10012, "id": motor_id, "mode": 2}, self.log_serial_tx)
        self._hat.flush()
        time.sleep(0.05)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._linear = float(msg.linear.x)
            self._angular = float(msg.angular.z)
            self._last_cmd_time = self.get_clock().now()

    def _tx_loop(self) -> None:
        period = 1.0 / self.command_rate_hz
        next_wake = time.monotonic()
        while self._running:
            self._update_targets_from_cmd()
            self._ramp_current_rpms()
            self._send_current_rpms()

            next_wake += period
            sleep_s = next_wake - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_wake = time.monotonic()

    def _update_targets_from_cmd(self) -> None:
        with self._cmd_lock:
            linear = self._linear
            angular = self._angular
            elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9

        if self.cmd_vel_timeout > 0.0 and elapsed > self.cmd_vel_timeout:
            linear = 0.0
            angular = 0.0

        left_vel = linear - angular * self.wheel_track / 2.0
        right_vel = linear + angular * self.wheel_track / 2.0
        rpm_per_mps = 60.0 / (2.0 * math.pi * self.wheel_radius)
        wheel_rpms = [left_vel * rpm_per_mps, right_vel * rpm_per_mps,
                      left_vel * rpm_per_mps, right_vel * rpm_per_mps]

        self._target_rpms = [
            self._clamp_rpm(wheel_rpms[i] * self.motor_signs[i])
            for i in range(4)
        ]

    def _ramp_current_rpms(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last_tx_time, 0.2)
        self._last_tx_time = now
        max_step = self.accel_rpm_per_sec * dt

        for i in range(4):
            diff = self._target_rpms[i] - self._current_rpms[i]
            if abs(diff) <= max_step:
                self._current_rpms[i] = self._target_rpms[i]
            else:
                self._current_rpms[i] += math.copysign(max_step, diff)

    def _send_current_rpms(self) -> None:
        rpm_by_id = dict(zip(self.motor_ids, self._current_rpms))
        commands = [
            {
                "T": 10010,
                "id": motor_id,
                "cmd": int(round(rpm_by_id[motor_id])),
                "act": self.hat_act,
            }
            for motor_id in self.command_order
        ]
        with self._command_lock:
            self._last_commanded_rpms = {
                command["id"]: command["cmd"]
                for command in commands
            }
        self._send_batch(commands)
        self._publish_wheel_states()

    def _send_stop(self) -> None:
        commands = [
            {"T": 10010, "id": motor_id, "cmd": 0, "act": self.hat_act}
            for motor_id in self.command_order
        ]
        try:
            with self._command_lock:
                self._last_commanded_rpms = {
                    command["id"]: command["cmd"]
                    for command in commands
                }
            self._send_batch(commands)
            self._current_rpms = [0.0, 0.0, 0.0, 0.0]
            self._target_rpms = [0.0, 0.0, 0.0, 0.0]
            self._publish_wheel_states()
        except Exception as exc:  # pragma: no cover - shutdown best effort
            self.get_logger().warn(f"failed to stop motors during shutdown: {exc}")

    def _send_batch(self, commands: List[Dict[str, int]]) -> None:
        # HAT 固件在一次写入多条 JSON 时可能只返回最后一条命令的反馈。
        # 逐个电机发送并留出极短间隔，便于读取 1/2/3/4 四个电机的返回转速。
        self._hat.send_commands(commands, self.per_motor_command_gap, self.log_serial_tx)

    def _clamp_rpm(self, rpm: float) -> float:
        return max(-self.max_rpm, min(self.max_rpm, rpm))

    def _current_physical_wheel_speeds(self) -> Tuple[List[float], List[str]]:
        circumference = 2.0 * math.pi * self.wheel_radius
        speeds = []
        sources = []
        now = time.monotonic()
        with self._feedback_lock:
            feedback_rpms = list(self._feedback_rpms)
            feedback_times = list(self._feedback_times)

        for index, (command_rpm, sign) in enumerate(zip(self._current_rpms, self.motor_signs)):
            rpm_source = command_rpm
            source = "command_fallback"
            if (
                self.use_motor_feedback
                and feedback_rpms[index] is not None
                and now - feedback_times[index] <= self.feedback_timeout
            ):
                rpm_source = feedback_rpms[index]
                source = "feedback"
            physical_rpm = rpm_source / sign if sign != 0 else 0.0
            speeds.append(physical_rpm * circumference / 60.0)
            sources.append(source)
        return speeds, sources

    def _publish_wheel_states(self) -> None:
        wheel_speeds, sources = self._current_physical_wheel_speeds()

        speeds_msg = Float32MultiArray()
        speeds_msg.data = [float(value) for value in wheel_speeds]
        self._wheel_speed_pub.publish(speeds_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = WHEEL_NAMES
        joint_msg.position = []
        joint_msg.effort = []
        joint_msg.velocity = [
            float(value / self.wheel_radius) if self.wheel_radius > 0.0 else 0.0
            for value in wheel_speeds
        ]
        self._joint_state_pub.publish(joint_msg)

        source_msg = String()
        source_msg.data = json.dumps(
            dict(zip(WHEEL_NAMES, sources)),
            separators=(",", ":"),
        )
        self._wheel_speed_source_pub.publish(source_msg)

    def _rx_loop(self) -> None:
        while self._running and rclpy.ok():
            try:
                raw = self._hat.readline()
            except Exception as exc:
                if self._running:
                    self.get_logger().warn(f"serial rx failed: {exc}")
                return

            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parsed = self._parse_feedback_line(line)
            if parsed is None:
                continue

            motor_id, rpm = parsed
            if motor_id not in self.motor_ids:
                continue
            index = self.motor_ids.index(motor_id)
            with self._feedback_lock:
                self._feedback_rpms[index] = rpm
                self._feedback_times[index] = time.monotonic()
            self._publish_wheel_states()

    def _parse_feedback_line(self, line: str) -> Optional[Tuple[int, float]]:
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return None

        if not isinstance(data, dict):
            return None

        motor_id = self._read_int_field(data, ("id", "ID"))
        rpm = self._read_float_field(data, ("spd", "rpm", "speed"))
        if motor_id is None or rpm is None:
            return None

        return motor_id, rpm

    def _last_published_rpm(self, motor_id: int) -> int:
        with self._command_lock:
            return int(self._last_commanded_rpms.get(motor_id, 0))

    def _direction_from_rpm(self, rpm: int) -> str:
        if rpm > 0:
            return "forward"
        if rpm < 0:
            return "backward"
        return "stop"

    def _log_feedback_snapshot(self) -> None:
        now = time.monotonic()
        with self._command_lock:
            commanded_rpms = dict(self._last_commanded_rpms)
        with self._feedback_lock:
            feedback_rpms = list(self._feedback_rpms)
            feedback_times = list(self._feedback_times)

        for index, motor_id in enumerate(self.motor_ids):
            published_rpm = int(commanded_rpms.get(motor_id, 0))
            returned_rpm = "无反馈"
            if (
                feedback_rpms[index] is not None
                and now - feedback_times[index] <= self.feedback_timeout
            ):
                returned_rpm = feedback_rpms[index]
            self.get_logger().info(
                f"ID：{motor_id}，朝向：{self._direction_from_rpm(published_rpm)}，"
                f"发布的转速：{published_rpm}，返回的转速：{returned_rpm}"
            )

    def _read_int_field(self, data: Dict[str, object], names: Tuple[str, ...]) -> Optional[int]:
        for name in names:
            value = data.get(name)
            if value is None:
                continue
            try:
                return int(value)
            except (TypeError, ValueError):
                return None
        return None

    def _read_float_field(self, data: Dict[str, object], names: Tuple[str, ...]) -> Optional[float]:
        for name in names:
            value = data.get(name)
            if value is None:
                continue
            try:
                return float(value)
            except (TypeError, ValueError):
                return None
        return None


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DDSMHatDiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
