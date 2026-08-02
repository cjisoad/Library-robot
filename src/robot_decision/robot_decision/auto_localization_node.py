#!/usr/bin/env python3

"""Restore a verified AMCL pose or safely request global localization at startup."""

from __future__ import annotations

import json
import math
import os
import time
from datetime import UTC, datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import SetInitialPose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Empty

from robot_decision.pose_utils import make_initial_pose


class AutoLocalizationNode(Node):
    """Keep a per-map pose cache and expose a conservative localization state."""

    def __init__(self) -> None:
        super().__init__("auto_localization_node")
        self._declare_parameters()
        self.active_map_id = self._required_string("active_map_id")
        self.state_file = Path(str(self.get_parameter("state_file").value)).expanduser()
        self.pose_topic = self._required_string("pose_topic")
        self.initial_pose_topic = self._required_string("initial_pose_topic")
        self.status_topic = self._required_string("status_topic")
        self.set_initial_pose_service = self._required_string("set_initial_pose_service")
        self.global_localization_service = self._required_string("global_localization_service")
        self.nomotion_update_service = self._required_string("nomotion_update_service")
        self.max_saved_pose_age_seconds = float(self.get_parameter("max_saved_pose_age_seconds").value)
        self.initialization_timeout_seconds = float(self.get_parameter("initialization_timeout_seconds").value)
        self.min_stable_samples = int(self.get_parameter("min_stable_samples").value)
        self.max_position_variance = float(self.get_parameter("max_position_variance").value)
        self.max_yaw_variance = float(self.get_parameter("max_yaw_variance").value)
        self.pose_stale_timeout_seconds = float(self.get_parameter("pose_stale_timeout_seconds").value)
        self.nomotion_update_period_seconds = float(
            self.get_parameter("nomotion_update_period_seconds").value
        )
        if self.max_saved_pose_age_seconds <= 0 or self.initialization_timeout_seconds <= 0:
            raise RuntimeError("localization timeout parameters must be greater than zero")
        if (
            self.min_stable_samples < 1
            or self.max_position_variance <= 0
            or self.max_yaw_variance <= 0
            or self.pose_stale_timeout_seconds <= 0
            or self.nomotion_update_period_seconds <= 0
        ):
            raise RuntimeError("localization stability parameters are invalid")

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_publisher = self.create_publisher(String, self.status_topic, status_qos)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.initial_pose_topic, 10
        )
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self._handle_amcl_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.initial_pose_topic, self._handle_manual_initial_pose, 10)
        self.initial_pose_client = self.create_client(SetInitialPose, self.set_initial_pose_service)
        self.global_localization_client = self.create_client(Empty, self.global_localization_service)
        self.nomotion_update_client = self.create_client(Empty, self.nomotion_update_service)

        self._status = "starting"
        self._stable_samples = 0
        self._initial_pose_timers = []
        self._manual_initial_pose: tuple[float, float, float] | None = None
        self._phase_started_at = self.get_clock().now()
        self._initial_request_sent = False
        self._global_request_sent = False
        self._last_stable_pose_at: float | None = None
        self._nomotion_request_pending = False
        self._startup_timer = self.create_timer(0.5, self._begin_startup_localization)
        self._watchdog_timer = self.create_timer(1.0, self._watchdog)
        self._nomotion_update_timer = self.create_timer(
            self.nomotion_update_period_seconds, self._request_nomotion_update
        )
        self._publish_status()

    def _declare_parameters(self) -> None:
        self.declare_parameter("active_map_id", "")
        self.declare_parameter("state_file", "~/.local/state/lr-robot/last_localization.json")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("status_topic", "/localization/status")
        self.declare_parameter("set_initial_pose_service", "/set_initial_pose")
        self.declare_parameter("global_localization_service", "/reinitialize_global_localization")
        self.declare_parameter("nomotion_update_service", "/request_nomotion_update")
        self.declare_parameter("max_saved_pose_age_seconds", 604800.0)
        self.declare_parameter("initialization_timeout_seconds", 30.0)
        self.declare_parameter("min_stable_samples", 3)
        self.declare_parameter("max_position_variance", 0.5)
        self.declare_parameter("max_yaw_variance", 0.35)
        self.declare_parameter("pose_stale_timeout_seconds", 5.0)
        self.declare_parameter("nomotion_update_period_seconds", 1.0)

    def _required_string(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise RuntimeError(f"{name} must be configured")
        return value

    def _begin_startup_localization(self) -> None:
        if self._status != "starting":
            self._startup_timer.cancel()
            return
        if not self.initial_pose_client.wait_for_service(timeout_sec=0.1):
            return
        saved_pose = self._read_saved_pose()
        if saved_pose:
            self._set_status("restoring_saved_pose")
            request = SetInitialPose.Request()
            request.pose = make_initial_pose(self, "map", *saved_pose)
            self.initial_pose_client.call_async(request).add_done_callback(self._handle_saved_pose_response)
            self.get_logger().info("restoring the last verified AMCL pose")
        else:
            if not self._request_global_localization():
                return
        self._startup_timer.cancel()

    def _handle_saved_pose_response(self, future) -> None:
        try:
            future.result()
        except Exception as error:  # pragma: no cover - ROS transport failures are runtime-only.
            self.get_logger().error(f"saved pose recovery failed: {error}")
            self._global_request_sent = False
            self._request_global_localization()

    def _request_global_localization(self) -> bool:
        if self._global_request_sent:
            return True
        if not self.global_localization_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("waiting for AMCL global localization service")
            return False
        self._set_status("global_localizing")
        self.global_localization_client.call_async(Empty.Request()).add_done_callback(self._handle_global_localization_response)
        self._global_request_sent = True
        self.get_logger().info("no verified pose is available; requested AMCL global localization")
        return True

    def _handle_global_localization_response(self, future) -> None:
        try:
            future.result()
        except Exception as error:  # pragma: no cover - ROS transport failures are runtime-only.
            self.get_logger().error(f"global localization request failed: {error}")
            self._set_status("manual_required")

    def _handle_manual_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != "map" or self._status in {"localized", "restoring_saved_pose"}:
            return
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self.get_logger().warn("ignoring manual initial pose with invalid coordinates")
            return
        self._stable_samples = 0
        self._manual_initial_pose = (x, y, yaw)
        self._set_status("manual_initializing")
        self.get_logger().info("received a manually confirmed initial pose")
        # RViz and the web gateway already publish /initialpose for AMCL. Do
        # not call /set_initial_pose here: Nav2's service also republishes the
        # topic, which would recursively trigger this subscription.
        holder = {}

        def accept(timer_holder=holder) -> None:
            timer_holder["timer"].cancel()
            self._accept_manual_pose_once()

        timer = self.create_timer(0.2, accept)
        holder["timer"] = timer
        self._initial_pose_timers.append(timer)

    def _accept_manual_pose_once(self) -> None:
        if self._status != "manual_initializing" or not self._manual_initial_pose:
            return
        self._mark_localized(*self._manual_initial_pose)
        self.get_logger().info("manually confirmed pose accepted; navigation can start")

    def _handle_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != "map":
            return
        if self._status not in {
            "restoring_saved_pose",
            "global_localizing",
            "manual_initializing",
            # Keep evaluating AMCL after the initial search deadline. The UI
            # can request a manual pose meanwhile, but a later stable laser
            # match must restore autonomous realtime localization by itself.
            "manual_required",
            "localized",
        }:
            return
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        covariance = message.pose.covariance
        stable = (
            math.isfinite(x)
            and math.isfinite(y)
            and math.isfinite(yaw)
            and covariance[0] <= self.max_position_variance
            and covariance[7] <= self.max_position_variance
            and covariance[35] <= self.max_yaw_variance
        )
        self._stable_samples = self._stable_samples + 1 if stable else 0
        if self._status == "localized":
            if stable:
                self._last_stable_pose_at = time.monotonic()
            else:
                self._set_status("manual_required")
                self.get_logger().warn("AMCL covariance is no longer reliable; navigation is blocked")
            return
        if self._stable_samples < self.min_stable_samples:
            return
        self._mark_localized(x, y, yaw, publish_initial_pose=True)
        self.get_logger().info("AMCL localization is stable; navigation can start")

    def _mark_localized(self, x: float, y: float, yaw: float, publish_initial_pose: bool = False) -> None:
        self._save_pose(x, y, yaw)
        # The navigation lifecycle waits for this topic before it starts when
        # localization was established automatically rather than by an operator.
        if publish_initial_pose:
            self._publish_initial_pose(x, y, 0.0, yaw)
        self._set_status("localized")
        self._last_stable_pose_at = time.monotonic()

    def _watchdog(self) -> None:
        if self._status == "localized":
            if (
                self._last_stable_pose_at is not None
                and time.monotonic() - self._last_stable_pose_at > self.pose_stale_timeout_seconds
            ):
                self._set_status("manual_required")
                self.get_logger().warn("AMCL pose timed out; navigation is blocked until localization recovers")
            return
        if self._status not in {"restoring_saved_pose", "global_localizing", "manual_initializing"}:
            return
        elapsed = (self.get_clock().now() - self._phase_started_at).nanoseconds / 1_000_000_000
        if elapsed < self.initialization_timeout_seconds:
            return
        if self._status == "restoring_saved_pose":
            self._global_request_sent = False
            self._request_global_localization()
            return
        self._set_status("manual_required")
        self.get_logger().warn(
            "automatic localization did not converge yet; awaiting a stable AMCL pose or operator confirmation"
        )

    def _request_nomotion_update(self) -> None:
        if self._nomotion_request_pending or self._status not in {
            "restoring_saved_pose",
            "global_localizing",
            "manual_initializing",
            "manual_required",
            "localized",
        }:
            return
        if not self.nomotion_update_client.wait_for_service(timeout_sec=0.0):
            return
        self._nomotion_request_pending = True
        future = self.nomotion_update_client.call_async(Empty.Request())
        future.add_done_callback(self._handle_nomotion_update_response)

    def _handle_nomotion_update_response(self, future) -> None:
        self._nomotion_request_pending = False
        try:
            future.result()
        except Exception as error:  # pragma: no cover - ROS transport failures are runtime-only.
            self.get_logger().warn(f"AMCL no-motion update request failed: {error}")

    def _read_saved_pose(self) -> tuple[float, float, float, float] | None:
        try:
            data = json.loads(self.state_file.read_text(encoding="utf-8"))
            if not isinstance(data, dict) or data.get("mapId") != self.active_map_id:
                return None
            saved_at = datetime.fromisoformat(str(data["savedAt"]).replace("Z", "+00:00"))
            if saved_at.tzinfo is None:
                return None
            age_seconds = (datetime.now(UTC) - saved_at.astimezone(UTC)).total_seconds()
            values = (data["x"], data["y"], data["yaw"])
            if age_seconds < 0 or age_seconds > self.max_saved_pose_age_seconds:
                return None
            if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values):
                return None
            return (float(values[0]), float(values[1]), 0.0, float(values[2]))
        except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
            return None

    def _save_pose(self, x: float, y: float, yaw: float) -> None:
        payload = {
            "mapId": self.active_map_id,
            "x": x,
            "y": y,
            "yaw": yaw,
            "savedAt": datetime.now(UTC).isoformat(),
        }
        try:
            self.state_file.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            temporary = self.state_file.with_suffix(".tmp")
            temporary.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
            os.replace(temporary, self.state_file)
        except OSError as error:  # Persisting is useful but must not block a valid localization.
            self.get_logger().error(f"failed to save verified pose: {error}")

    def _publish_initial_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        pose = make_initial_pose(self, "map", x, y, z, yaw)
        # Repeat briefly so the Nav2 startup helper observes the pose after boot.
        self.initial_pose_publisher.publish(pose)
        for delay in (0.2, 0.4):
            holder = {}

            def republish(timer_holder=holder) -> None:
                self.initial_pose_publisher.publish(pose)
                timer_holder["timer"].cancel()

            timer = self.create_timer(delay, republish)
            holder["timer"] = timer
            self._initial_pose_timers.append(timer)

    def _set_status(self, status: str) -> None:
        if self._status == status:
            return
        self._status = status
        self._phase_started_at = self.get_clock().now()
        self._publish_status()

    def _publish_status(self) -> None:
        self.status_publisher.publish(String(data=self._status))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutoLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
