#!/usr/bin/env python3
"""Wheel + IMU odometry node."""

import math
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(orientation: Quaternion) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CarOdometry(Node):
    def __init__(self) -> None:
        super().__init__("car_odometry")

        self.declare_parameter("wheel_speed_topic", "/wheel_speeds")
        self.declare_parameter("joint_state_topic", "/wheel_joint_states")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("use_imu_orientation", True)
        self.declare_parameter("use_imu_angular_velocity", True)
        self.declare_parameter("use_wheel_angular_fallback", False)
        self.declare_parameter("track_width", 0.33)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("imu_angular_velocity_scale", 1.0)
        self.declare_parameter("imu_angular_velocity_bias", 0.0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("imu_timeout", 0.5)
        self.declare_parameter("imu_error_log_period", 1.0)
        self.declare_parameter("enable_angle_debug", False)
        self.declare_parameter("debug_log_period", 0.5)
        self.declare_parameter("debug_odom_yaw_topic", "/debug/odom_yaw_deg")
        self.declare_parameter("debug_imu_yaw_topic", "/debug/imu_yaw_deg")
        self.declare_parameter("debug_angular_velocity_topic", "/debug/angular_velocity_deg_s")
        self.declare_parameter("debug_gyro_integrated_yaw_topic", "/debug/gyro_integrated_yaw_deg")
        self.declare_parameter("debug_x_topic", "/debug/odom_x_m")
        self.declare_parameter("debug_y_topic", "/debug/odom_y_m")
        self.declare_parameter("debug_distance_topic", "/debug/odom_distance_m")

        wheel_speed_topic = self.get_parameter("wheel_speed_topic").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.use_imu_orientation = bool(self.get_parameter("use_imu_orientation").value)
        self.use_imu_angular_velocity = bool(self.get_parameter("use_imu_angular_velocity").value)
        self.use_wheel_angular_fallback = bool(
            self.get_parameter("use_wheel_angular_fallback").value
        )
        self.track_width = float(self.get_parameter("track_width").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.imu_angular_velocity_scale = float(
            self.get_parameter("imu_angular_velocity_scale").value
        )
        self.imu_angular_velocity_bias = float(
            self.get_parameter("imu_angular_velocity_bias").value
        )
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout").value)
        self.imu_error_log_period = float(self.get_parameter("imu_error_log_period").value)
        self.enable_angle_debug = bool(self.get_parameter("enable_angle_debug").value)
        self.debug_log_period = float(self.get_parameter("debug_log_period").value)
        debug_odom_yaw_topic = self.get_parameter("debug_odom_yaw_topic").value
        debug_imu_yaw_topic = self.get_parameter("debug_imu_yaw_topic").value
        debug_angular_velocity_topic = self.get_parameter("debug_angular_velocity_topic").value
        debug_gyro_integrated_yaw_topic = self.get_parameter("debug_gyro_integrated_yaw_topic").value
        debug_x_topic = self.get_parameter("debug_x_topic").value
        debug_y_topic = self.get_parameter("debug_y_topic").value
        debug_distance_topic = self.get_parameter("debug_distance_topic").value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.debug_odom_yaw_pub = None
        self.debug_imu_yaw_pub = None
        self.debug_angular_velocity_pub = None
        self.debug_gyro_integrated_yaw_pub = None
        self.debug_x_pub = None
        self.debug_y_pub = None
        self.debug_distance_pub = None
        if self.enable_angle_debug:
            self.debug_odom_yaw_pub = self.create_publisher(Float32, debug_odom_yaw_topic, 10)
            self.debug_imu_yaw_pub = self.create_publisher(Float32, debug_imu_yaw_topic, 10)
            self.debug_angular_velocity_pub = self.create_publisher(Float32, debug_angular_velocity_topic, 10)
            self.debug_gyro_integrated_yaw_pub = self.create_publisher(Float32, debug_gyro_integrated_yaw_topic, 10)
            self.debug_x_pub = self.create_publisher(Float32, debug_x_topic, 10)
            self.debug_y_pub = self.create_publisher(Float32, debug_y_topic, 10)
            self.debug_distance_pub = self.create_publisher(Float32, debug_distance_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float32MultiArray, wheel_speed_topic, self.wheel_speed_callback, 10)
        self.create_subscription(JointState, joint_state_topic, self.joint_state_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.latest_wheel_speed: Optional[List[float]] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_imu_receive_time: Optional[float] = None
        self.last_imu_error_log_time = 0.0
        self.last_update_time = self.get_clock().now()
        self.last_debug_log_time = self.get_clock().now()
        self.latest_imu_yaw_deg = 0.0
        self.latest_imu_yaw_wrapped_deg = 0.0
        self.accumulated_yaw = 0.0
        self.gyro_integrated_yaw = 0.0
        self.previous_imu_yaw: Optional[float] = None
        self.previous_imu_stamp_sec: Optional[float] = None
        self.imu_orientation_angular_z = 0.0

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self.update_odometry)

        self.get_logger().info(
            f"odometry ready: subscribed {wheel_speed_topic}, {joint_state_topic}, {imu_topic}; "
            f"publishing {odom_topic}"
        )

    def wheel_speed_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("received /wheel_speeds with fewer than 4 elements")
            return
        self.latest_wheel_speed = [float(value) for value in msg.data[:4]]

    def joint_state_callback(self, msg: JointState) -> None:
        if self.latest_wheel_speed is not None:
            return
        if len(msg.velocity) < 4 or self.wheel_radius <= 0.0:
            return
        self.latest_wheel_speed = [float(value) * self.wheel_radius for value in msg.velocity[:4]]

    def imu_callback(self, msg: Imu) -> None:
        self.latest_imu = msg
        self.latest_imu_receive_time = time.monotonic()

    def _stamp_to_sec(self, msg: Imu) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

    def extract_base_linear_and_angular(self, wheel_speed: Sequence[float]) -> Tuple[float, float]:
        left_speed = (wheel_speed[0] + wheel_speed[2]) / 2.0
        right_speed = (wheel_speed[1] + wheel_speed[3]) / 2.0
        linear_x = (left_speed + right_speed) / 2.0 * self.linear_scale
        angular_from_wheels = 0.0
        if self.track_width > 0.0:
            angular_from_wheels = (right_speed - left_speed) / self.track_width * self.angular_scale
        return linear_x, angular_from_wheels

    def update_odometry(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_update_time = now

        if not self._has_fresh_imu():
            self._log_imu_error()
            return

        linear_x = 0.0
        angular_z = 0.0
        previous_accumulated_yaw = self.accumulated_yaw
        delta_theta = 0.0

        if self.latest_wheel_speed is not None:
            linear_x, angular_from_wheels = self.extract_base_linear_and_angular(self.latest_wheel_speed)
            if self.use_wheel_angular_fallback:
                angular_z = angular_from_wheels

        if self.use_imu_orientation:
            yaw = quaternion_to_yaw(self.latest_imu.orientation)
            imu_stamp_sec = self._stamp_to_sec(self.latest_imu)
            self.latest_imu_yaw_wrapped_deg = math.degrees(yaw)
            if self.previous_imu_yaw is None:
                self.previous_imu_yaw = yaw
                self.previous_imu_stamp_sec = imu_stamp_sec
                self.theta = self.accumulated_yaw
            elif imu_stamp_sec != self.previous_imu_stamp_sec:
                imu_dt = imu_stamp_sec - self.previous_imu_stamp_sec
                delta_theta = normalize_angle(yaw - self.previous_imu_yaw)
                self.previous_imu_yaw = yaw
                self.previous_imu_stamp_sec = imu_stamp_sec
                self.accumulated_yaw += delta_theta
                self.theta = self.accumulated_yaw
                if imu_dt > 0.0:
                    self.imu_orientation_angular_z = delta_theta / imu_dt
            self.latest_imu_yaw_deg = math.degrees(self.accumulated_yaw)
            angular_z = self.imu_orientation_angular_z
        elif self.use_imu_angular_velocity:
            angular_z = (
                float(self.latest_imu.angular_velocity.z) - self.imu_angular_velocity_bias
            ) * self.imu_angular_velocity_scale

        gyro_delta_theta = angular_z * dt
        self.gyro_integrated_yaw += gyro_delta_theta

        if not self.use_imu_orientation:
            delta_theta = gyro_delta_theta
            self.accumulated_yaw = self.gyro_integrated_yaw
            self.theta = normalize_angle(self.theta + delta_theta)
            self.latest_imu_yaw_deg = math.degrees(self.accumulated_yaw)

        heading = previous_accumulated_yaw + delta_theta / 2.0
        self.x += linear_x * math.cos(heading) * dt
        self.y += linear_x * math.sin(heading) * dt
        self.linear_x = linear_x
        self.angular_z = angular_z

        if self.enable_angle_debug:
            self.publish_debug_angle(now)
        self.publish_odometry(now)

    def _has_fresh_imu(self) -> bool:
        if self.latest_imu is None or self.latest_imu_receive_time is None:
            return False
        if self.imu_timeout <= 0.0:
            return True
        return time.monotonic() - self.latest_imu_receive_time <= self.imu_timeout

    def _log_imu_error(self) -> None:
        now = time.monotonic()
        if now - self.last_imu_error_log_time < self.imu_error_log_period:
            return
        self.last_imu_error_log_time = now
        self.get_logger().error("IMU 无反馈或已超时，跳过本次里程计更新")

    def publish_debug_angle(self, stamp) -> None:
        odom_yaw_deg = math.degrees(self.accumulated_yaw)
        angular_velocity_deg_s = math.degrees(self.angular_z)

        for pub, value in (
            (self.debug_odom_yaw_pub, odom_yaw_deg),
            (self.debug_imu_yaw_pub, self.latest_imu_yaw_deg),
            (self.debug_angular_velocity_pub, angular_velocity_deg_s),
            (self.debug_gyro_integrated_yaw_pub, math.degrees(self.gyro_integrated_yaw)),
            (self.debug_x_pub, self.x),
            (self.debug_y_pub, self.y),
            (self.debug_distance_pub, math.hypot(self.x, self.y)),
        ):
            msg = Float32()
            msg.data = float(value)
            pub.publish(msg)

    def publish_odometry(self, stamp) -> None:
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.theta)
        odom_msg.twist.twist.linear.x = self.linear_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_z
        odom_msg.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        odom_msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.20, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        self.odom_pub.publish(odom_msg)

        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CarOdometry()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
