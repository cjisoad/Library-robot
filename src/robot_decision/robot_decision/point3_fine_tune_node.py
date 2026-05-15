#!/usr/bin/env python3

import math
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class Point3FineTuneNode(Node):
    def __init__(self) -> None:
        super().__init__("point3_fine_tune_node")

        default_points_file = (
            get_package_share_directory("robot_decision") + "/config/points.yaml"
        )
        self.declare_parameter("points_file", default_points_file)

        points_file = str(self.get_parameter("points_file").value)
        self._load_config(points_file)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.service = self.create_service(Trigger, self.start_service, self._handle_start)
        self._active = False

        self.get_logger().info(
            "point3 fine tune ready: service=%s target=%s"
            % (self.start_service, self.target_point)
        )

    def _load_config(self, points_file: str) -> None:
        with open(points_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        fine_tune = data.get("fine_tune", {})
        if not isinstance(fine_tune, dict):
            raise RuntimeError("fine_tune config must be a mapping in %s" % points_file)

        self.enabled = self._read_bool(fine_tune.get("enabled", True), "fine_tune.enabled")
        self.target_point = str(fine_tune.get("target_point", "point3"))
        points = data.get("points", {})
        if self.target_point not in points:
            raise RuntimeError("fine_tune target point '%s' is missing" % self.target_point)

        target = points[self.target_point]
        self.target_x = float(target["x"])
        self.target_y = float(target["y"])
        self.target_yaw = float(target["yaw"])
        self.target_frame = str(fine_tune.get("target_frame", target.get("frame_id", "map")))
        self.base_frame = str(fine_tune.get("base_frame", "base_link"))
        self.start_service = str(fine_tune.get("start_service", "/point3_fine_tune/start"))
        self.cmd_vel_topic = str(fine_tune.get("cmd_vel_topic", "/cmd_vel"))
        self.control_rate_hz = float(fine_tune.get("control_rate_hz", 10.0))
        self.x_tolerance = float(fine_tune.get("x_tolerance", 0.03))
        self.yaw_tolerance = float(fine_tune.get("yaw_tolerance", 0.05))
        self.max_linear_speed = float(fine_tune.get("max_linear_speed", 0.05))
        self.max_angular_speed = float(fine_tune.get("max_angular_speed", 0.20))
        self.k_x = float(fine_tune.get("k_x", 0.6))
        self.k_yaw = float(fine_tune.get("k_yaw", 1.0))
        self.timeout_sec = float(fine_tune.get("timeout_sec", 8.0))

        if self.control_rate_hz <= 0.0:
            raise RuntimeError("fine_tune.control_rate_hz must be positive")
        if self.timeout_sec <= 0.0:
            raise RuntimeError("fine_tune.timeout_sec must be positive")

    def _read_bool(self, value, name: str) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ("true", "1", "yes", "on"):
                return True
            if lowered in ("false", "0", "no", "off"):
                return False
        raise RuntimeError("%s must be a boolean" % name)

    def _handle_start(self, _request, response):
        if not self.enabled:
            response.success = True
            response.message = "fine tune disabled"
            return response

        if self._active:
            response.success = False
            response.message = "fine tune is already active"
            return response

        self._active = True
        try:
            success, message = self._run_fine_tune()
        finally:
            self._publish_stop()
            self._active = False

        response.success = success
        response.message = message
        return response

    def _run_fine_tune(self) -> tuple[bool, str]:
        deadline = time.monotonic() + self.timeout_sec
        period = 1.0 / self.control_rate_hz

        while rclpy.ok() and time.monotonic() < deadline:
            try:
                x_error, yaw_error = self._calculate_errors()
            except TransformException as exc:
                self.get_logger().warn("waiting for TF: %s" % exc)
                time.sleep(period)
                continue

            if abs(x_error) <= self.x_tolerance and abs(yaw_error) <= self.yaw_tolerance:
                msg = "fine tune complete: x_error=%.3f yaw_error=%.3f" % (
                    x_error,
                    yaw_error,
                )
                self.get_logger().info(msg)
                return True, msg

            cmd = Twist()
            cmd.linear.x = clamp(self.k_x * x_error, self.max_linear_speed)
            cmd.angular.z = clamp(self.k_yaw * yaw_error, self.max_angular_speed)
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(
                "fine tuning x_error=%.3f yaw_error=%.3f cmd_x=%.3f cmd_yaw=%.3f"
                % (x_error, yaw_error, cmd.linear.x, cmd.angular.z)
            )
            time.sleep(period)

        return False, "fine tune timed out"

    def _calculate_errors(self) -> tuple[float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.2),
        )

        trans = transform.transform.translation
        rot = transform.transform.rotation
        current_yaw = quaternion_to_yaw(rot.x, rot.y, rot.z, rot.w)

        dx_map = self.target_x - trans.x
        dy_map = self.target_y - trans.y
        x_error_base = math.cos(current_yaw) * dx_map + math.sin(current_yaw) * dy_map
        yaw_error = normalize_angle(self.target_yaw - current_yaw)
        return x_error_base, yaw_error

    def _publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = Point3FineTuneNode()
    except Exception as exc:
        rclpy.logging.get_logger("point3_fine_tune_node").error(str(exc))
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
