#!/usr/bin/env python3
"""
ROS2 四轮小车里程计节点。

功能:
- 订阅轮速话题和 IMU 话题
- 融合轮速线速度与 IMU 姿态/角速度
- 发布 nav_msgs/msg/Odometry
- 发布 odom -> base_link TF

默认假设:
- 小车为四轮差速结构，x 轴为车体前方，z 轴向上
- IMU 按“x 方向为前方”安装，IMU 数据已在 base_link 坐标系下表达
- 轮速话题 /wheel_speeds 的顺序为:
  [left_front, right_front, left_rear, right_rear]，单位 m/s
"""

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
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
        self.declare_parameter("track_width", 0.30)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("publish_rate", 50.0)
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
        self.track_width = float(self.get_parameter("track_width").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
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
        self.last_update_time = self.get_clock().now()
        self.last_debug_log_time = self.get_clock().now()
        self.latest_imu_yaw_deg = 0.0
        self.latest_imu_yaw_wrapped_deg = 0.0
        self.accumulated_yaw = 0.0
        self.gyro_integrated_yaw = 0.0
        self.previous_imu_yaw: Optional[float] = None

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self.update_odometry)

        self.get_logger().info(
            f"里程计节点启动: 订阅 {wheel_speed_topic}, {joint_state_topic}, {imu_topic}, 发布 {odom_topic}"
        )

    def wheel_speed_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            self.get_logger().warn("收到的 /wheel_speeds 数据不足 4 个元素，已忽略")
            return
        self.latest_wheel_speed = [float(value) for value in msg.data[:4]]

    def joint_state_callback(self, msg: JointState) -> None:
        if self.latest_wheel_speed is not None:
            return
        if len(msg.velocity) < 4:
            return
        if self.wheel_radius <= 0.0:
            return
        self.latest_wheel_speed = [float(value) * self.wheel_radius for value in msg.velocity[:4]]

    def imu_callback(self, msg: Imu) -> None:
        self.latest_imu = msg

    def extract_base_linear_and_angular(self, wheel_speed: Sequence[float]) -> Tuple[float, float]:
        left_speed = (wheel_speed[0] + wheel_speed[2]) / 2.0
        right_speed = (wheel_speed[1] + wheel_speed[3]) / 2.0
        linear_x = (left_speed + right_speed) / 2.0 * self.linear_scale
        angular_from_wheels = 0.0
        if self.track_width > 0.0:
            angular_from_wheels = (right_speed - left_speed) / self.track_width
        return linear_x, angular_from_wheels

    def update_odometry(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_update_time = now

        linear_x = 0.0
        angular_z = 0.0
        previous_accumulated_yaw = self.accumulated_yaw
        delta_theta = 0.0

        if self.latest_wheel_speed is not None:
            linear_x, angular_from_wheels = self.extract_base_linear_and_angular(self.latest_wheel_speed)
            angular_z = angular_from_wheels

        if self.latest_imu is not None:
            if self.use_imu_angular_velocity:
                angular_z = float(self.latest_imu.angular_velocity.z)
            if self.use_imu_orientation:
                yaw = quaternion_to_yaw(self.latest_imu.orientation)
                self.latest_imu_yaw_wrapped_deg = math.degrees(yaw)
                if self.previous_imu_yaw is None:
                    self.previous_imu_yaw = yaw
                    self.theta = yaw
                else:
                    delta_theta = normalize_angle(yaw - self.previous_imu_yaw)
                    self.previous_imu_yaw = yaw
                    self.theta = yaw
                    self.accumulated_yaw += delta_theta
                self.latest_imu_yaw_deg = math.degrees(self.accumulated_yaw)

        gyro_delta_theta = angular_z * dt
        self.gyro_integrated_yaw += gyro_delta_theta

        if not (self.use_imu_orientation and self.latest_imu is not None):
            delta_theta = gyro_delta_theta
            self.accumulated_yaw = self.gyro_integrated_yaw
            self.theta = normalize_angle(self.theta + delta_theta)
            self.latest_imu_yaw_deg = math.degrees(self.accumulated_yaw)

        heading = previous_accumulated_yaw + delta_theta / 2.0

        delta_x = linear_x * math.cos(heading) * dt
        delta_y = linear_x * math.sin(heading) * dt

        self.x += delta_x
        self.y += delta_y
        self.linear_x = linear_x
        self.angular_z = angular_z

        if self.enable_angle_debug:
            self.publish_debug_angle(now)
        self.publish_odometry(now)

    def publish_debug_angle(self, stamp) -> None:
        odom_yaw_deg = math.degrees(self.accumulated_yaw)
        angular_velocity_deg_s = math.degrees(self.angular_z)

        odom_yaw_msg = Float32()
        odom_yaw_msg.data = float(odom_yaw_deg)
        self.debug_odom_yaw_pub.publish(odom_yaw_msg)

        imu_yaw_msg = Float32()
        imu_yaw_msg.data = float(self.latest_imu_yaw_deg)
        self.debug_imu_yaw_pub.publish(imu_yaw_msg)

        angular_velocity_msg = Float32()
        angular_velocity_msg.data = float(angular_velocity_deg_s)
        self.debug_angular_velocity_pub.publish(angular_velocity_msg)

        gyro_integrated_yaw_msg = Float32()
        gyro_integrated_yaw_msg.data = float(math.degrees(self.gyro_integrated_yaw))
        self.debug_gyro_integrated_yaw_pub.publish(gyro_integrated_yaw_msg)

        x_msg = Float32()
        x_msg.data = float(self.x)
        self.debug_x_pub.publish(x_msg)

        y_msg = Float32()
        y_msg.data = float(self.y)
        self.debug_y_pub.publish(y_msg)

        distance_msg = Float32()
        distance_msg.data = float(math.hypot(self.x, self.y))
        self.debug_distance_pub.publish(distance_msg)

        elapsed = (stamp - self.last_debug_log_time).nanoseconds / 1e9
        if elapsed < self.debug_log_period:
            return

        self.last_debug_log_time = stamp
        self.get_logger().info(
            "角度调试: "
            f"odom_yaw_total={odom_yaw_deg:.2f} deg, "
            f"imu_yaw_total={self.latest_imu_yaw_deg:.2f} deg, "
            f"imu_yaw_wrapped={self.latest_imu_yaw_wrapped_deg:.2f} deg, "
            f"gyro_yaw_total={math.degrees(self.gyro_integrated_yaw):.2f} deg, "
            f"angular_z={angular_velocity_deg_s:.2f} deg/s, "
            f"linear_x={self.linear_x:.3f} m/s, "
            f"x={self.x:.3f} m, "
            f"y={self.y:.3f} m, "
            f"distance={math.hypot(self.x, self.y):.3f} m"
        )

    def publish_odometry(self, stamp) -> None:
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        if self.latest_imu is not None and self.use_imu_orientation:
            odom_msg.pose.pose.orientation = self.latest_imu.orientation
        else:
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
    node = CarOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
