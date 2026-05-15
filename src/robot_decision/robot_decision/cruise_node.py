#!/usr/bin/env python3

import threading

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

from robot_decision.pose_utils import make_pose_stamped


class CruiseNode(Node):
    def __init__(self) -> None:
        super().__init__("cruise_node")

        default_points_file = (
            get_package_share_directory("robot_decision") + "/config/points.yaml"
        )
        self.declare_parameter("points_file", default_points_file)
        self.declare_parameter("navigate_action_name", "navigate_to_pose")
        self.declare_parameter("server_timeout_sec", 2.0)
        self.declare_parameter("shutdown_when_done", True)

        points_file = str(self.get_parameter("points_file").value)
        action_name = str(self.get_parameter("navigate_action_name").value)
        self.server_timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.shutdown_when_done = bool(self.get_parameter("shutdown_when_done").value)

        self.points, self.loop, self.fine_tune_config = self._load_points(points_file)
        self.action_client = ActionClient(self, NavigateToPose, action_name)
        self.fine_tune_client = self.create_client(
            Trigger,
            self.fine_tune_config["start_service"],
        )
        self.current_index = 0
        self.loop_count = 1
        self._active_goal_handle = None
        self._waiting_for_terminal_continue = False
        self._startup_timer = self.create_timer(0.5, self._start_when_ready)

        self.get_logger().info(
            "loaded %d cruise points from %s, loop=%s, fine_tune=%s target=%s"
            % (
                len(self.points),
                points_file,
                self.loop,
                self.fine_tune_config["enabled"],
                self.fine_tune_config["target_point"],
            )
        )

    def _load_points(self, points_file: str) -> tuple[list[dict], bool, dict]:
        with open(points_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        cruise_config = data.get("cruise", {})
        if not isinstance(cruise_config, dict):
            raise RuntimeError("cruise config must be a mapping in %s" % points_file)

        point_order = cruise_config.get("point_order", [])
        if not isinstance(point_order, list) or not point_order:
            raise RuntimeError("cruise.point_order must be a non-empty list in %s" % points_file)

        loop = self._read_bool(cruise_config.get("loop", False), "cruise.loop")
        configured_points = data.get("points", {})
        if not isinstance(configured_points, dict):
            raise RuntimeError("points config must be a mapping in %s" % points_file)

        points = []
        for name in point_order:
            name = str(name)
            if name not in configured_points:
                raise RuntimeError("missing cruise point '%s' in %s" % (name, points_file))

            raw_point = configured_points[name]
            if not isinstance(raw_point, dict):
                raise RuntimeError("cruise point '%s' must be a mapping" % name)

            points.append(
                {
                    "name": name,
                    "frame_id": str(raw_point.get("frame_id", "map")),
                    "x": float(raw_point["x"]),
                    "y": float(raw_point["y"]),
                    "z": float(raw_point.get("z", 0.0)),
                    "yaw": float(raw_point["yaw"]),
                }
            )

        fine_tune_config = self._load_fine_tune_config(data, points_file)
        return points, loop, fine_tune_config

    def _load_fine_tune_config(self, data: dict, points_file: str) -> dict:
        fine_tune = data.get("fine_tune", {})
        if not isinstance(fine_tune, dict):
            raise RuntimeError("fine_tune config must be a mapping in %s" % points_file)

        enabled = self._read_bool(fine_tune.get("enabled", False), "fine_tune.enabled")
        target_point = str(fine_tune.get("target_point", "point3"))
        start_service = str(fine_tune.get("start_service", "/point3_fine_tune/start"))
        wait_for_terminal_continue = self._read_bool(
            fine_tune.get("wait_for_terminal_continue", True),
            "fine_tune.wait_for_terminal_continue",
        )
        return {
            "enabled": enabled,
            "target_point": target_point,
            "start_service": start_service,
            "wait_for_terminal_continue": wait_for_terminal_continue,
        }

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

    def _start_when_ready(self) -> None:
        if not self.action_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            self.get_logger().warn("waiting for Nav2 NavigateToPose action server")
            return

        self._startup_timer.cancel()
        self._send_next_goal()

    def _send_next_goal(self) -> None:
        if self.current_index >= len(self.points):
            if self.loop:
                self.loop_count += 1
                self.current_index = 0
                self.get_logger().info("starting cruise loop %d" % self.loop_count)
            else:
                self.get_logger().info("cruise completed")
                if self.shutdown_when_done:
                    rclpy.shutdown()
                return

        point = self.points[self.current_index]
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(
            self,
            point["frame_id"],
            point["x"],
            point["y"],
            point["z"],
            point["yaw"],
        )

        self.get_logger().info(
            "sending %s loop=%d (%d/%d): x=%.3f y=%.3f yaw=%.3f"
            % (
                point["name"],
                self.loop_count,
                self.current_index + 1,
                len(self.points),
                point["x"],
                point["y"],
                point["yaw"],
            )
        )
        future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._handle_goal_response)

    def _handle_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            point = self.points[self.current_index]
            self.get_logger().error("goal %s was rejected" % point["name"])
            rclpy.shutdown()
            return

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_goal_result)

    def _handle_goal_result(self, future) -> None:
        result = future.result()
        point = self.points[self.current_index]

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                "goal %s finished with status %d" % (point["name"], result.status)
            )
            rclpy.shutdown()
            return

        self.get_logger().info("reached %s" % point["name"])
        if self._should_fine_tune(point):
            self._start_fine_tune(point)
            return

        self.current_index += 1
        self._send_next_goal()

    def _should_fine_tune(self, point: dict) -> bool:
        return (
            self.fine_tune_config["enabled"]
            and point["name"] == self.fine_tune_config["target_point"]
        )

    def _start_fine_tune(self, point: dict) -> None:
        if not self.fine_tune_client.wait_for_service(timeout_sec=self.server_timeout_sec):
            self.get_logger().error(
                "fine tune service %s is not available"
                % self.fine_tune_config["start_service"]
            )
            rclpy.shutdown()
            return

        self.get_logger().info("starting fine tune for %s" % point["name"])
        future = self.fine_tune_client.call_async(Trigger.Request())
        future.add_done_callback(self._handle_fine_tune_result)

    def _handle_fine_tune_result(self, future) -> None:
        point = self.points[self.current_index]
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime ROS failure path
            self.get_logger().error("fine tune service failed: %s" % exc)
            rclpy.shutdown()
            return

        if not response.success:
            self.get_logger().error("fine tune failed for %s: %s" % (point["name"], response.message))
            rclpy.shutdown()
            return

        self.get_logger().info("fine tune succeeded for %s: %s" % (point["name"], response.message))
        if self.fine_tune_config["wait_for_terminal_continue"]:
            self._wait_for_terminal_continue(point)
        else:
            self._continue_after_fine_tune()

    def _wait_for_terminal_continue(self, point: dict) -> None:
        if self._waiting_for_terminal_continue:
            return

        self._waiting_for_terminal_continue = True
        self.get_logger().info(
            "fine tune complete at %s; enter 1 in this terminal to continue cruise"
            % point["name"]
        )
        thread = threading.Thread(target=self._terminal_continue_loop, daemon=True)
        thread.start()

    def _terminal_continue_loop(self) -> None:
        while rclpy.ok():
            try:
                value = input("Enter 1 to continue cruise: ").strip()
            except EOFError:
                self.get_logger().error("terminal input closed while waiting to continue")
                rclpy.shutdown()
                return

            if value == "1":
                self.get_logger().info("continue command received")
                self._waiting_for_terminal_continue = False
                self._continue_after_fine_tune()
                return

            self.get_logger().warn("ignored input '%s'; enter 1 to continue" % value)

    def _continue_after_fine_tune(self) -> None:
        self.current_index += 1
        self._send_next_goal()

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if self.current_index >= len(self.points):
            return

        remaining = getattr(feedback, "distance_remaining", None)
        if remaining is None:
            return

        self.get_logger().debug(
            "%s distance remaining %.3f m"
            % (self.points[self.current_index]["name"], remaining)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = CruiseNode()
    except Exception as exc:
        rclpy.logging.get_logger("cruise_node").error(str(exc))
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
