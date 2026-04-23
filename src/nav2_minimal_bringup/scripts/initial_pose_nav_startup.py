#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes, SetInitialPose
from rclpy.node import Node


class InitialPoseNavStartup(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_nav_startup")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter(
            "manage_nodes_service",
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self.declare_parameter(
            "set_initial_pose_service",
            "/set_initial_pose",
        )

        initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self.manage_nodes_service = str(self.get_parameter("manage_nodes_service").value)
        self.set_initial_pose_service = str(
            self.get_parameter("set_initial_pose_service").value
        )

        self.startup_requested = False
        self.startup_complete = False
        self.startup_future = None
        self.initial_pose_request_sent = False

        self.navigation_client = self.create_client(
            ManageLifecycleNodes, self.manage_nodes_service
        )
        self.initial_pose_client = self.create_client(
            SetInitialPose, self.set_initial_pose_service
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            self._initial_pose_callback,
            10,
        )

        self.get_logger().info(
            f"Waiting for initial pose on {initial_pose_topic} before starting navigation lifecycle."
        )

    def _initial_pose_callback(self, _msg: PoseWithCovarianceStamped) -> None:
        if self.initial_pose_request_sent or self.startup_complete:
            return

        if not self.initial_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"Initial pose service {self.set_initial_pose_service} is not available yet."
            )
            return

        request = SetInitialPose.Request()
        request.pose = _msg

        self.initial_pose_request_sent = True
        future = self.initial_pose_client.call_async(request)
        future.add_done_callback(self._handle_initial_pose_response)
        self.get_logger().info("Initial pose received, forwarding to AMCL.")

    def _handle_initial_pose_response(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # pragma: no cover - runtime ROS failure path
            self.initial_pose_request_sent = False
            self.get_logger().error(f"Failed to set initial pose in AMCL: {exc}")
            return

        self.get_logger().info("Initial pose accepted by AMCL, requesting navigation startup.")
        self._request_navigation_startup()

    def _request_navigation_startup(self) -> None:
        if self.startup_requested or self.startup_complete:
            return

        if not self.navigation_client.wait_for_service(timeout_sec=1.0):
            self.initial_pose_request_sent = False
            self.get_logger().warn(
                f"Navigation lifecycle service {self.manage_nodes_service} is not available yet."
            )
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP

        self.startup_requested = True
        self.startup_future = self.navigation_client.call_async(request)
        self.startup_future.add_done_callback(self._handle_startup_response)

    def _handle_startup_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime ROS failure path
            self.startup_requested = False
            self.initial_pose_request_sent = False
            self.get_logger().error(f"Navigation startup request failed: {exc}")
            return

        if response.success:
            self.startup_complete = True
            self.get_logger().info("Navigation lifecycle startup completed.")
        else:
            self.startup_requested = False
            self.initial_pose_request_sent = False
            self.get_logger().error("Navigation lifecycle startup was rejected.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPoseNavStartup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
