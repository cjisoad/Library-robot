#!/usr/bin/env python3

"""Bridge trusted MQTT fleet commands into safe local navigation and teleoperation.

This node is deliberately a robot-local boundary. It validates command identity,
expiry, map identity, and pose bounds before it creates any ROS action or velocity.
Manual velocity remains local, bounded, and short-lived; the physical emergency
stop remains independent of this software path.
"""

from __future__ import annotations

import json
import math
import queue
import ssl
import threading
import time
from collections import OrderedDict
from datetime import UTC, datetime
from typing import Any
from uuid import uuid4

import paho.mqtt.client as mqtt
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from robot_decision.bookarm_client import BookArmApiError, BookArmClient, BookArmHealth
from robot_decision.edge_gateway_protocol import (
    BookArmTestSession,
    BookTask,
    GatewayCommand,
    InitialPose,
    CruiseRoute,
    LiftMove,
    ManualControlStart,
    ManualJog,
    NavigationGoal,
    parse_gateway_command,
    parse_bookarm_test_session_id,
    parse_bookarm_test_start,
    parse_book_task,
    parse_cruise_route,
    parse_initial_pose,
    parse_lift_move,
    parse_manual_control_start,
    parse_manual_jog,
    parse_navigation_goal,
)
from robot_decision.pose_utils import make_initial_pose


class EdgeGatewayNode(Node):
    """Receive central commands from MQTT and execute only permitted Nav2 actions."""

    def __init__(self) -> None:
        super().__init__("edge_gateway_node")
        self._declare_parameters()
        self.robot_id = self._required_string_parameter("robot_id")
        self.mqtt_host = self._required_string_parameter("mqtt_host")
        self.mqtt_port = self._positive_int_parameter("mqtt_port")
        self.mqtt_protocol_version = self._required_string_parameter("mqtt_protocol_version")
        self.mqtt_tls_enabled = bool(self.get_parameter("mqtt_tls_enabled").value)
        self.mqtt_ca_file = str(self.get_parameter("mqtt_ca_file").value).strip()
        self.mqtt_client_cert_file = str(self.get_parameter("mqtt_client_cert_file").value).strip()
        self.mqtt_client_key_file = str(self.get_parameter("mqtt_client_key_file").value).strip()
        self.mqtt_username = str(self.get_parameter("mqtt_username").value)
        self.mqtt_password = str(self.get_parameter("mqtt_password").value)
        self.allowed_map_ids = {
            str(value).strip() for value in self.get_parameter("allowed_map_ids").value if str(value).strip()
        }
        self.map_min_x = float(self.get_parameter("map_min_x").value)
        self.map_max_x = float(self.get_parameter("map_max_x").value)
        self.map_min_y = float(self.get_parameter("map_min_y").value)
        self.map_max_y = float(self.get_parameter("map_max_y").value)
        self.max_abs_yaw = float(self.get_parameter("max_abs_yaw").value)
        self.command_queue_size = self._positive_int_parameter("command_queue_size")
        self.navigate_action_name = self._required_string_parameter("navigate_action_name")
        self.manual_cmd_vel_topic = self._required_string_parameter("manual_cmd_vel_topic")
        self.active_map_id = self._required_string_parameter("active_map_id")
        self.amcl_pose_topic = self._required_string_parameter("amcl_pose_topic")
        self.initial_pose_topic = self._required_string_parameter("initial_pose_topic")
        self.localization_status_topic = self._required_string_parameter("localization_status_topic")
        self.localization_scan_mode_topic = self._required_string_parameter("localization_scan_mode_topic")
        self.chassis_status_topic = self._required_string_parameter("chassis_status_topic")
        self.chassis_status_timeout_seconds = float(
            self.get_parameter("chassis_status_timeout_seconds").value
        )
        self.telemetry_period_seconds = float(self.get_parameter("telemetry_period_seconds").value)
        self.pose_stale_timeout_seconds = float(self.get_parameter("pose_stale_timeout_seconds").value)
        self.max_position_variance = float(self.get_parameter("max_position_variance").value)
        self.max_yaw_variance = float(self.get_parameter("max_yaw_variance").value)
        self.manual_command_timeout_seconds = float(
            self.get_parameter("manual_command_timeout_seconds").value
        )
        self.bookarm_api_url = self._required_string_parameter("bookarm_api_url")
        self.bookarm_timeout_seconds = float(self.get_parameter("bookarm_timeout_seconds").value)
        self.bookarm_poll_period_seconds = float(
            self.get_parameter("bookarm_poll_period_seconds").value
        )
        self.bookarm_rod_device = self._required_string_parameter("bookarm_rod_device")
        self.bookarm_lift_device = self._required_string_parameter("bookarm_lift_device")
        self._validate_configuration()

        self.command_topic = f"fleet/{self.robot_id}/command"
        self.event_topic = f"fleet/{self.robot_id}/event"
        self._pending_commands: queue.Queue[GatewayCommand] = queue.Queue(maxsize=self.command_queue_size)
        self._seen_command_ids: OrderedDict[str, None] = OrderedDict()
        self._seen_lock = threading.Lock()
        self._active_command: GatewayCommand | None = None
        self._active_goal_handle = None
        self._pending_navigation_command: GatewayCommand | None = None
        self._active_cruise_route: CruiseRoute | None = None
        self._cruise_waypoint_index = 0
        self._cruise_loop_index = 0
        self._manual_lease: ManualControlStart | None = None
        self._manual_command_deadline: float | None = None
        self._active_book_task: BookTask | None = None
        self._book_task_command: GatewayCommand | None = None
        self._book_task_phase: str | None = None
        self._bookarm_workflow_id: str | None = None
        self._book_task_last_state: tuple[int, bool, bool, str] | None = None
        self._bookarm_test_session: BookArmTestSession | None = None
        self._bookarm_test_command: GatewayCommand | None = None
        self._bookarm_test_last_state: tuple[int, bool, bool, str] | None = None
        self._bookarm_health = BookArmHealth(False, False, False, False, False, "机械臂状态尚未读取")
        self._bookarm_client = BookArmClient(
            self.bookarm_api_url,
            self.bookarm_timeout_seconds,
            rod_device_path=self.bookarm_rod_device,
            lift_device_path=self.bookarm_lift_device,
        )
        self._latest_pose: tuple[float, float, float] | None = None
        self._latest_pose_received_at: float | None = None
        self._localization_status = "starting"
        self._localization_scan_mode = "unavailable"
        self._chassis_feedback_healthy = False
        self._chassis_status_received_at: float | None = None
        self._action_client = ActionClient(self, NavigateToPose, self.navigate_action_name)
        self._amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self._handle_amcl_pose,
            10,
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )
        self._manual_velocity_publisher = self.create_publisher(Twist, self.manual_cmd_vel_topic, 10)
        self._localization_status_subscription = self.create_subscription(
            String,
            self.localization_status_topic,
            self._handle_localization_status,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self._chassis_status_subscription = self.create_subscription(
            String,
            self.chassis_status_topic,
            self._handle_chassis_status,
            10,
        )
        self._localization_scan_mode_subscription = self.create_subscription(
            String,
            self.localization_scan_mode_topic,
            self._handle_localization_scan_mode,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self._mqtt = self._create_mqtt_client()
        self._mqtt.loop_start()
        self._mqtt.connect_async(self.mqtt_host, self.mqtt_port, keepalive=30)
        self._queue_timer = self.create_timer(0.1, self._process_command_queue)
        self._telemetry_timer = self.create_timer(self.telemetry_period_seconds, self._publish_telemetry)
        self._manual_safety_timer = self.create_timer(0.05, self._enforce_manual_stop)
        self._bookarm_health_timer = self.create_timer(2.0, self._refresh_bookarm_health)
        self._bookarm_workflow_timer = self.create_timer(
            self.bookarm_poll_period_seconds, self._poll_bookarm_workflow
        )

        self.get_logger().info(
            f"Edge Gateway ready for {self.robot_id}; subscribing to {self.command_topic}, "
            f"Nav2 action {self.navigate_action_name}, pose topic {self.amcl_pose_topic}, "
            f"manual velocity topic {self.manual_cmd_vel_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("robot_id", "")
        self.declare_parameter("mqtt_host", "")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_protocol_version", "3.1.1")
        self.declare_parameter("mqtt_tls_enabled", False)
        self.declare_parameter("mqtt_ca_file", "")
        self.declare_parameter("mqtt_client_cert_file", "")
        self.declare_parameter("mqtt_client_key_file", "")
        self.declare_parameter("mqtt_username", "")
        self.declare_parameter("mqtt_password", "")
        # An empty Python list is inferred as BYTE_ARRAY by rclpy; declare an
        # explicit string-array default so YAML map IDs can override it safely.
        self.declare_parameter("allowed_map_ids", [""])
        self.declare_parameter("map_min_x", 0.0)
        self.declare_parameter("map_max_x", 0.0)
        self.declare_parameter("map_min_y", 0.0)
        self.declare_parameter("map_max_y", 0.0)
        self.declare_parameter("max_abs_yaw", math.tau)
        self.declare_parameter("command_queue_size", 10)
        self.declare_parameter("navigate_action_name", "navigate_to_pose")
        self.declare_parameter("manual_cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("active_map_id", "")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("localization_status_topic", "/localization/status")
        self.declare_parameter("localization_scan_mode_topic", "/localization/scan_mode")
        # The base controller publishes a JSON map of wheel feedback sources.
        # Gateway must see fresh encoder feedback before it permits manual motion.
        self.declare_parameter("chassis_status_topic", "/wheel_speed_source")
        self.declare_parameter("chassis_status_timeout_seconds", 2.5)
        self.declare_parameter("telemetry_period_seconds", 0.5)
        self.declare_parameter("pose_stale_timeout_seconds", 5.0)
        self.declare_parameter("max_position_variance", 0.5)
        self.declare_parameter("max_yaw_variance", 0.35)
        self.declare_parameter("manual_command_timeout_seconds", 1.0)
        # The MotorStudio WebUI is deliberately loopback-only on the robot.
        # Its actual CAN/serial worker API is never exposed through MQTT.
        self.declare_parameter("bookarm_api_url", "http://127.0.0.1:18080/api/v1")
        self.declare_parameter("bookarm_timeout_seconds", 3.0)
        self.declare_parameter("bookarm_poll_period_seconds", 1.0)
        self.declare_parameter("bookarm_rod_device", "/dev/rodmotor")
        self.declare_parameter("bookarm_lift_device", "/dev/lift_port")

    def _required_string_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise RuntimeError(f"{name} must be configured")
        return value

    def _positive_int_parameter(self, name: str) -> int:
        value = int(self.get_parameter(name).value)
        if value <= 0:
            raise RuntimeError(f"{name} must be greater than zero")
        return value

    def _validate_configuration(self) -> None:
        if not self.allowed_map_ids:
            raise RuntimeError("allowed_map_ids must contain the map currently loaded by Nav2")
        if self.active_map_id not in self.allowed_map_ids:
            raise RuntimeError("active_map_id must be included in allowed_map_ids")
        if self.map_min_x >= self.map_max_x or self.map_min_y >= self.map_max_y:
            raise RuntimeError("configured map bounds are invalid")
        if self.max_abs_yaw <= 0.0:
            raise RuntimeError("max_abs_yaw must be greater than zero")
        if self.telemetry_period_seconds < 0.2:
            raise RuntimeError("telemetry_period_seconds must be at least 0.2 seconds")
        if self.pose_stale_timeout_seconds <= 0.0:
            raise RuntimeError("pose_stale_timeout_seconds must be greater than zero")
        if self.max_position_variance <= 0.0 or self.max_yaw_variance <= 0.0:
            raise RuntimeError("AMCL covariance limits must be greater than zero")
        if self.manual_command_timeout_seconds <= 0.0 or self.manual_command_timeout_seconds > 2.0:
            raise RuntimeError("manual_command_timeout_seconds must be from 0 to 2")
        if self.chassis_status_timeout_seconds <= 0.0 or self.chassis_status_timeout_seconds > 10.0:
            raise RuntimeError("chassis_status_timeout_seconds must be from 0 to 10")
        bookarm_timeout_seconds = float(getattr(self, "bookarm_timeout_seconds", 3.0))
        bookarm_poll_period_seconds = float(getattr(self, "bookarm_poll_period_seconds", 1.0))
        if bookarm_timeout_seconds <= 0.0 or bookarm_timeout_seconds > 15.0:
            raise RuntimeError("bookarm_timeout_seconds must be from 0 to 15")
        if bookarm_poll_period_seconds < 0.5 or bookarm_poll_period_seconds > 10.0:
            raise RuntimeError("bookarm_poll_period_seconds must be from 0.5 to 10")
        if self.mqtt_tls_enabled and not self.mqtt_ca_file:
            raise RuntimeError("mqtt_ca_file is required when mqtt_tls_enabled is true")
        if self.mqtt_tls_enabled and not (self.mqtt_client_cert_file and self.mqtt_client_key_file):
            raise RuntimeError("mqtt client certificate and key are required when mqtt_tls_enabled is true")
        if bool(self.mqtt_client_cert_file) != bool(self.mqtt_client_key_file):
            raise RuntimeError("mqtt_client_cert_file and mqtt_client_key_file must be configured together")
        if self.mqtt_protocol_version not in {"3.1.1", "5"}:
            raise RuntimeError("mqtt_protocol_version must be 3.1.1 or 5")
        if not self.mqtt_tls_enabled:
            self.get_logger().warn("MQTT TLS is disabled; use this only on an isolated development network")

    def _create_mqtt_client(self) -> mqtt.Client:
        callback_version = getattr(mqtt, "CallbackAPIVersion", None)
        protocol = mqtt.MQTTv5 if self.mqtt_protocol_version == "5" else mqtt.MQTTv311
        client_options = {"client_id": f"lr-edge-{self.robot_id}", "protocol": protocol}
        if callback_version is not None:
            client_options["callback_api_version"] = callback_version.VERSION2
        if protocol == mqtt.MQTTv311:
            client_options["clean_session"] = False
        client = mqtt.Client(**client_options)
        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)
        if self.mqtt_tls_enabled:
            client.tls_set(
                ca_certs=self.mqtt_ca_file,
                certfile=self.mqtt_client_cert_file or None,
                keyfile=self.mqtt_client_key_file or None,
                tls_version=ssl.PROTOCOL_TLS_CLIENT,
            )
            client.tls_insecure_set(False)
        client.reconnect_delay_set(min_delay=1, max_delay=30)
        client.on_connect = self._on_mqtt_connect
        client.on_disconnect = self._on_mqtt_disconnect
        client.on_message = self._on_mqtt_message
        return client

    def _on_mqtt_connect(self, client, _userdata, _flags, reason_code, _properties=None) -> None:
        if int(reason_code) != 0:
            self.get_logger().error(f"MQTT connection refused: {reason_code}")
            return
        result, _mid = client.subscribe(self.command_topic, qos=1)
        if result != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(f"MQTT subscribe failed with code {result}")
            return
        self.get_logger().info(f"MQTT connected; subscribed to {self.command_topic}")
        self._publish_event("gateway_online", details={"allowedMapIds": sorted(self.allowed_map_ids)})

    def _on_mqtt_disconnect(self, _client, _userdata, _rc, *arguments) -> None:
        # Paho MQTT 1.x passes ``rc``; 2.x passes flags, reason code, properties.
        reason_code = arguments[0] if arguments else _rc
        self.get_logger().warn(f"MQTT disconnected: {reason_code}")

    def _on_mqtt_message(self, _client, _userdata, message) -> None:
        if message.topic != self.command_topic:
            return
        try:
            command = parse_gateway_command(message.payload, self.robot_id)
        except ValueError as error:
            self._publish_event("command_rejected", details={"reason": str(error)})
            return
        if not self._remember_command(command.command_id):
            self._publish_event("command_duplicate", command, {"reason": "commandId was already received"})
            return
        try:
            self._pending_commands.put_nowait(command)
        except queue.Full:
            self._publish_event("command_rejected", command, {"reason": "gateway command queue is full"})
            return
        self._publish_event("command_received", command)

    def _remember_command(self, command_id: str) -> bool:
        with self._seen_lock:
            if command_id in self._seen_command_ids:
                return False
            self._seen_command_ids[command_id] = None
            # QoS redeliveries only need a bounded idempotency cache on the robot.
            if len(self._seen_command_ids) > 512:
                self._seen_command_ids.popitem(last=False)
            return True

    def _process_command_queue(self) -> None:
        try:
            command = self._pending_commands.get_nowait()
        except queue.Empty:
            return
        if command.command == "navigate":
            self._start_navigation(command)
            return
        if command.command == "set_initial_pose":
            self._set_initial_pose(command)
            return
        if command.command == "cruise":
            self._start_cruise(command)
            return
        if command.command == "start_manual_control":
            self._start_manual_control(command)
            return
        if command.command == "manual_jog":
            self._manual_jog(command)
            return
        if command.command == "stop_manual_control":
            self._stop_manual_control(command)
            return
        if command.command == "book_task":
            self._start_book_task(command)
            return
        if command.command == "book_task_continue":
            self._continue_book_task(command)
            return
        if command.command == "book_task_cancel":
            self._cancel_book_task(command, "operator cancelled book task")
            return
        if command.command == "bookarm_test_start":
            self._start_bookarm_test(command)
            return
        if command.command == "bookarm_test_continue":
            self._continue_bookarm_test(command)
            return
        if command.command == "bookarm_test_cancel":
            self._cancel_bookarm_test(command, "operator cancelled maintenance test")
            return
        if command.command == "lift_move":
            self._move_lift(command)
            return
        self._cancel_navigation(command)

    def _set_initial_pose(self, command: GatewayCommand) -> None:
        """Forward a web-confirmed location to AMCL; this never moves the robot."""
        if self._active_command or self._pending_navigation_command:
            self._publish_event(
                "command_rejected",
                command,
                {"reason": "cannot change localization while a navigation goal is active"},
            )
            return
        try:
            initial_pose = parse_initial_pose(
                command.payload,
                self.allowed_map_ids,
                self.map_min_x,
                self.map_max_x,
                self.map_min_y,
                self.map_max_y,
                self.max_abs_yaw,
            )
        except ValueError as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        self._initial_pose_publisher.publish(
            make_initial_pose(self, "map", initial_pose.x, initial_pose.y, 0.0, initial_pose.yaw)
        )
        # AMCL can legitimately wait for motion before it emits /amcl_pose.
        # Preserve the operator-confirmed pose for telemetry once localization
        # manager reports that AMCL accepted it.
        self._latest_pose = (initial_pose.x, initial_pose.y, initial_pose.yaw)
        self._publish_event(
            "localization_initializing",
            command,
            {
                "mapId": initial_pose.map_id,
                "x": initial_pose.x,
                "y": initial_pose.y,
                "yaw": initial_pose.yaw,
            },
        )

    def _start_navigation(self, command: GatewayCommand) -> None:
        if self._manual_lease or self._bookarm_test_command:
            self._publish_event(
                "command_rejected", command, {"reason": "manual or book-arm maintenance control is active"}
            )
            return
        if self._localization_status != "localized" or not self._latest_pose:
            self._publish_event(
                "command_rejected",
                command,
                {"reason": "robot localization is not stable; navigation is blocked"},
            )
            return
        try:
            goal = parse_navigation_goal(
                command.payload,
                self.allowed_map_ids,
                self.map_min_x,
                self.map_max_x,
                self.map_min_y,
                self.map_max_y,
                self.max_abs_yaw,
            )
        except ValueError as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        if self._active_command or self._pending_navigation_command:
            self._publish_event("command_rejected", command, {"reason": "a navigation goal is already active"})
            return
        if not self._action_client.server_is_ready():
            self._publish_event("command_rejected", command, {"reason": "Nav2 NavigateToPose server is unavailable"})
            return

        self._pending_navigation_command = command
        ros_goal = NavigateToPose.Goal()
        ros_goal.pose = self._pose_from_goal(goal)
        future = self._action_client.send_goal_async(ros_goal, feedback_callback=self._navigation_feedback)
        future.add_done_callback(lambda result: self._handle_goal_response(command, goal, result))
        self._publish_event("navigation_dispatching", command, {"mapId": goal.map_id, "annotationId": goal.annotation_id})

    def _start_cruise(self, command: GatewayCommand) -> None:
        if self._manual_lease or self._bookarm_test_command:
            self._publish_event(
                "command_rejected", command, {"reason": "manual or book-arm maintenance control is active"}
            )
            return
        if self._localization_status != "localized" or not self._latest_pose:
            self._publish_event(
                "command_rejected", command, {"reason": "robot localization is not stable; cruise is blocked"}
            )
            return
        if self._active_command or self._pending_navigation_command:
            self._publish_event(
                "command_rejected", command, {"reason": "a navigation goal is already active"}
            )
            return
        if not self._action_client.server_is_ready():
            self._publish_event(
                "command_rejected", command, {"reason": "Nav2 NavigateToPose server is unavailable"}
            )
            return
        try:
            route = parse_cruise_route(
                command.payload,
                self.allowed_map_ids,
                self.map_min_x,
                self.map_max_x,
                self.map_min_y,
                self.map_max_y,
                self.max_abs_yaw,
            )
        except ValueError as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        self._active_command = command
        self._active_cruise_route = route
        self._cruise_waypoint_index = 0
        self._cruise_loop_index = 0
        self._publish_event(
            "cruise_dispatching",
            command,
            {"mapId": route.map_id, "waypointCount": len(route.waypoints), "loopCount": route.loop_count},
        )
        self._send_next_cruise_waypoint()

    # ---- Book-arm maintenance ------------------------------------

    def _start_bookarm_test(self, command: GatewayCommand) -> None:
        if self._manual_lease:
            self._publish_event("command_rejected", command, {"reason": "manual control lease is active"})
            return
        if (
            self._active_book_task
            or self._bookarm_test_command
            or self._active_command
            or self._pending_navigation_command
        ):
            self._publish_event(
                "command_rejected",
                command,
                {"reason": "another navigation, book task, or maintenance test is active"},
            )
            return
        try:
            session = parse_bookarm_test_start(command.payload)
            health = self._bookarm_client.health()
            if not (health.lift_ready and health.camera_ready and health.rod_ready):
                raise BookArmApiError(health.message)
            health = self._bookarm_client.prepare_arm()
            if not health.arm_ready:
                raise BookArmApiError("机械臂连接或使能未确认")
            workflow = self._bookarm_client.start_workflow(session.workflow_mode, auto=False)
        except (ValueError, BookArmApiError) as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        self._bookarm_health = health
        self._bookarm_test_session = session
        self._bookarm_test_command = command
        self._bookarm_workflow_id = str(workflow["id"])
        self._bookarm_test_last_state = None
        self._publish_event(
            "bookarm_test_started",
            command,
            {
                "sessionId": session.session_id,
                "workflowId": self._bookarm_workflow_id,
                "workflowMode": session.workflow_mode,
                "stepMode": "manual",
            },
        )
        self._publish_bookarm_test_state(command, workflow)

    def _continue_bookarm_test(self, command: GatewayCommand) -> None:
        session = self._bookarm_test_session
        active_command = self._bookarm_test_command
        if not session or not active_command or not self._bookarm_workflow_id:
            self._publish_event("command_rejected", command, {"reason": "no book-arm maintenance test is active"})
            return
        try:
            session_id = parse_bookarm_test_session_id(command.payload)
        except ValueError as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        if session_id != session.session_id:
            self._publish_event("command_rejected", command, {"reason": "maintenance sessionId does not match"})
            return
        try:
            current = self._bookarm_client.workflow(self._bookarm_workflow_id)
            if current.get("waiting") is True:
                raise BookArmApiError("机械臂当前步骤仍在执行")
            if current.get("active") is not True:
                raise BookArmApiError("机械臂维护流程已经结束")
            workflow = self._bookarm_client.continue_workflow(self._bookarm_workflow_id)
        except BookArmApiError as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        self._publish_bookarm_test_state(active_command, workflow)
        self._publish_event(
            "command_completed",
            command,
            {"result": "book-arm maintenance step accepted", "sessionId": session.session_id},
        )

    def _cancel_bookarm_test(self, command: GatewayCommand, reason: str) -> None:
        session = self._bookarm_test_session
        active_command = self._bookarm_test_command
        if not session or not active_command:
            self._publish_event("command_rejected", command, {"reason": "no book-arm maintenance test is active"})
            return
        if command.command == "bookarm_test_cancel":
            try:
                session_id = parse_bookarm_test_session_id(command.payload)
            except ValueError as error:
                self._publish_event("command_rejected", command, {"reason": str(error)})
                return
            if session_id != session.session_id:
                self._publish_event("command_rejected", command, {"reason": "maintenance sessionId does not match"})
                return
        try:
            if command.command == "emergency_stop":
                self._bookarm_client.emergency_stop()
            elif self._bookarm_workflow_id:
                self._bookarm_client.cancel_workflow(self._bookarm_workflow_id)
        except BookArmApiError as error:
            self.get_logger().warn(f"book-arm maintenance stop failed: {error}")
        self._publish_event(
            "bookarm_test_cancelled",
            active_command,
            {"sessionId": session.session_id, "reason": reason},
        )
        self._clear_bookarm_test()
        self._publish_event(
            "command_completed",
            command,
            {"result": "book-arm maintenance test stopped", "sessionId": session.session_id},
        )

    def _publish_bookarm_test_state(self, command: GatewayCommand, workflow: dict[str, Any]) -> None:
        session = self._bookarm_test_session
        if not session or self._bookarm_test_command is not command:
            return
        step_index = workflow.get("step_index")
        step_count = workflow.get("step_count")
        active = workflow.get("active")
        waiting = workflow.get("waiting")
        message = str(workflow.get("message", ""))[:240]
        if not isinstance(step_index, int) or not isinstance(step_count, int) or step_count <= 0:
            self._fail_bookarm_test(command, "机械臂维护工作流状态无效")
            return
        preflight_state = workflow.get("preflight_state")
        preflight_complete = workflow.get("preflight_complete")
        state_key = (
            step_index,
            bool(active),
            bool(waiting),
            message,
            preflight_state,
            bool(preflight_complete),
        )
        if self._bookarm_test_last_state == state_key:
            return
        self._bookarm_test_last_state = state_key
        details: dict[str, Any] = {
            "sessionId": session.session_id,
            "workflowId": self._bookarm_workflow_id,
            "workflowMode": session.workflow_mode,
            "stepIndex": step_index,
            "stepCount": step_count,
            "stepName": workflow.get("step_name"),
            "waiting": bool(waiting),
            "message": message,
        }
        capture_result = workflow.get("capture_result")
        recognition_result = workflow.get("recognition_result")
        if isinstance(capture_result, dict):
            details["captureResult"] = capture_result
        if isinstance(recognition_result, dict):
            details["recognitionResult"] = recognition_result
        if isinstance(workflow.get("preflight_required"), bool):
            details["preflightRequired"] = workflow["preflight_required"]
        if isinstance(preflight_complete, bool):
            details["preflightComplete"] = preflight_complete
        if isinstance(preflight_state, str):
            details["preflightState"] = preflight_state
        if isinstance(workflow.get("preflight_name"), str):
            details["preflightName"] = workflow["preflight_name"]
        if bool(active):
            self._publish_event("bookarm_test_workflow", command, details)
            return
        if step_index >= step_count and not workflow.get("safety_stop_state"):
            self._publish_event(
                "bookarm_test_succeeded",
                command,
                {**details, "message": message or "机械臂维护测试已完成"},
            )
            self._clear_bookarm_test()
            return
        self._fail_bookarm_test(command, message or "book-arm maintenance workflow stopped")

    def _fail_bookarm_test(self, command: GatewayCommand, reason: str) -> None:
        session = self._bookarm_test_session
        if session and self._bookarm_test_command is command:
            self._publish_event(
                "bookarm_test_failed",
                command,
                {"sessionId": session.session_id, "reason": str(reason)[:300]},
            )
            self._clear_bookarm_test()
            return
        self._publish_event("command_rejected", command, {"reason": str(reason)[:300]})

    def _clear_bookarm_test(self) -> None:
        self._bookarm_test_session = None
        self._bookarm_test_command = None
        self._bookarm_workflow_id = None
        self._bookarm_test_last_state = None

    # ---- Book circulation -----------------------------------------

    def _start_book_task(self, command: GatewayCommand) -> None:
        """Navigate to a shelf, then delegate arm work to local MotorStudio."""
        if self._manual_lease:
            self._publish_event("command_rejected", command, {"reason": "manual control lease is active"})
            return
        if (
            self._active_book_task
            or self._bookarm_test_command
            or self._active_command
            or self._pending_navigation_command
        ):
            self._publish_event("command_rejected", command, {"reason": "another navigation or book task is active"})
            return
        if self._localization_status != "localized" or not self._latest_pose:
            self._publish_event("command_rejected", command, {"reason": "robot localization is not stable"})
            return
        if not self._chassis_ready() or not self._action_client.server_is_ready():
            self._publish_event("command_rejected", command, {"reason": "chassis feedback or Nav2 is unavailable"})
            return
        try:
            task = parse_book_task(
                command.payload,
                self.allowed_map_ids,
                self.map_min_x,
                self.map_max_x,
                self.map_min_y,
                self.map_max_y,
                self.max_abs_yaw,
            )
            health = self._bookarm_client.health()
        except (ValueError, BookArmApiError) as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        # Never convert a calibrated absolute target into an unreferenced
        # relative move. The lift service must explicitly attest homing and
        # physical position feedback before an automated book workflow starts.
        if not health.lift_absolute_positioning_ready:
            self._publish_event(
                "command_rejected",
                command,
                {"reason": "lift absolute homing or position feedback is unavailable"},
            )
            return
        self._bookarm_health = health
        if not (health.lift_ready and health.camera_ready and health.rod_ready):
            self._publish_event("command_rejected", command, {"reason": health.message})
            return
        try:
            health = self._bookarm_client.prepare_arm()
        except BookArmApiError as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        self._bookarm_health = health
        if not health.arm_ready:
            self._publish_event("command_rejected", command, {"reason": "机械臂连接或使能未确认"})
            return
        self._active_book_task = task
        self._book_task_command = command
        self._book_task_phase = "navigating_to_shelf"
        self._bookarm_workflow_id = None
        self._book_task_last_state = None
        self._pending_navigation_command = command
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_from_goal(task.shelf)
        future = self._action_client.send_goal_async(goal, feedback_callback=self._navigation_feedback)
        future.add_done_callback(lambda result: self._handle_book_goal_response(command, "shelf", result))
        self._publish_event(
            "book_task_dispatching",
            command,
            {"taskId": task.task_id, "operation": task.operation, "phase": self._book_task_phase},
        )

    def _handle_book_goal_response(self, command: GatewayCommand, destination: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - ROS transport failures are runtime-only.
            self._fail_book_task(command, f"Nav2 goal failed: {error}")
            return
        if self._book_task_command is not command or self._pending_navigation_command is not command:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self._pending_navigation_command = None
        if not goal_handle.accepted:
            self._fail_book_task(command, "Nav2 rejected the book-task navigation goal")
            return
        self._active_command = command
        self._active_goal_handle = goal_handle
        self._book_task_phase = f"navigating_to_{destination}"
        self._publish_event(
            "book_task_navigating",
            command,
            {"destination": destination, "phase": self._book_task_phase},
        )
        goal_handle.get_result_async().add_done_callback(
            lambda result: self._handle_book_goal_result(command, destination, result)
        )

    def _handle_book_goal_result(self, command: GatewayCommand, destination: str, future) -> None:
        if self._book_task_command is not command or self._active_command is not command:
            return
        self._active_goal_handle = None
        self._active_command = None
        try:
            result = future.result()
        except Exception as error:  # pragma: no cover - ROS transport failures are runtime-only.
            self._fail_book_task(command, f"Nav2 result failed: {error}")
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail_book_task(command, f"book-task navigation ended with Nav2 status {int(result.status)}")
            return
        if destination == "delivery":
            self._complete_book_task(command, "图书已送达交付点")
            return
        self._start_bookarm_workflow(command)

    def _start_bookarm_workflow(self, command: GatewayCommand) -> None:
        task = self._active_book_task
        if not task:
            return
        try:
            # takeout is the only original workflow whose automatic mode is
            # supported. Return workflows preserve their existing step-by-step
            # confirmation and are advanced through book_task_continue.
            workflow = self._bookarm_client.start_workflow(
                task.workflow_mode, auto=task.operation == "borrow"
            )
        except BookArmApiError as error:
            self._fail_book_task(command, str(error))
            return
        self._bookarm_workflow_id = str(workflow["id"])
        self._book_task_phase = "book_workflow"
        self._publish_book_workflow_state(command, workflow)

    def _continue_book_task(self, command: GatewayCommand) -> None:
        task = self._active_book_task
        active_command = self._book_task_command
        if not task or not active_command or not self._bookarm_workflow_id:
            self._publish_event("command_rejected", command, {"reason": "no book workflow is waiting for confirmation"})
            return
        if command.payload.get("taskId") != task.task_id:
            self._publish_event("command_rejected", command, {"reason": "book taskId does not match the active task"})
            return
        if task.operation != "return":
            self._publish_event("command_rejected", command, {"reason": "borrow workflow advances automatically"})
            return
        try:
            current = self._bookarm_client.workflow(self._bookarm_workflow_id)
        except BookArmApiError as error:
            self._fail_book_task(active_command, str(error))
            return
        if current.get("waiting") is True:
            self._publish_event("command_rejected", command, {"reason": "book workflow is still executing its current step"})
            return
        if current.get("active") is not True:
            self._publish_event("command_rejected", command, {"reason": "book workflow is not ready for a next step"})
            return
        try:
            workflow = self._bookarm_client.continue_workflow(self._bookarm_workflow_id)
        except BookArmApiError as error:
            self._fail_book_task(active_command, str(error))
            return
        self._publish_book_workflow_state(active_command, workflow)

    def _poll_bookarm_workflow(self) -> None:
        workflow_id = self._bookarm_workflow_id
        test_command = self._bookarm_test_command
        if workflow_id and test_command:
            try:
                workflow = self._bookarm_client.workflow(workflow_id)
            except BookArmApiError as error:
                self._fail_bookarm_test(test_command, str(error))
                return
            self._publish_bookarm_test_state(test_command, workflow)
            return
        command = self._book_task_command
        if not workflow_id or not command:
            return
        try:
            workflow = self._bookarm_client.workflow(workflow_id)
        except BookArmApiError as error:
            self._fail_book_task(command, str(error))
            return
        self._publish_book_workflow_state(command, workflow)

    def _publish_book_workflow_state(self, command: GatewayCommand, workflow: dict[str, Any]) -> None:
        task = self._active_book_task
        if not task or self._book_task_command is not command:
            return
        step_index = workflow.get("step_index")
        step_count = workflow.get("step_count")
        active = workflow.get("active")
        waiting = workflow.get("waiting")
        message = str(workflow.get("message", ""))[:240]
        if not isinstance(step_index, int) or not isinstance(step_count, int):
            self._fail_book_task(command, "机械臂工作流状态无效")
            return
        preflight_state = workflow.get("preflight_state")
        preflight_complete = workflow.get("preflight_complete")
        state_key = (
            step_index,
            bool(active),
            bool(waiting),
            message,
            preflight_state,
            bool(preflight_complete),
        )
        if self._book_task_last_state == state_key:
            return
        self._book_task_last_state = state_key
        details = {
            "taskId": task.task_id,
            "operation": task.operation,
            "workflowId": self._bookarm_workflow_id,
            "workflowMode": task.workflow_mode,
            "stepIndex": step_index,
            "stepCount": step_count,
            "stepName": workflow.get("step_name"),
            "waiting": bool(waiting),
            "message": message,
        }
        capture_result = workflow.get("capture_result")
        recognition_result = workflow.get("recognition_result")
        if isinstance(capture_result, dict):
            details["captureResult"] = capture_result
        if isinstance(recognition_result, dict):
            details["recognitionResult"] = recognition_result
        if isinstance(workflow.get("preflight_required"), bool):
            details["preflightRequired"] = workflow["preflight_required"]
        if isinstance(preflight_complete, bool):
            details["preflightComplete"] = preflight_complete
        if isinstance(preflight_state, str):
            details["preflightState"] = preflight_state
        if isinstance(workflow.get("preflight_name"), str):
            details["preflightName"] = workflow["preflight_name"]
        if bool(active):
            self._book_task_phase = "book_workflow"
            self._publish_event("book_task_workflow", command, details)
            return
        if step_index >= step_count and not workflow.get("safety_stop_state"):
            if task.operation == "borrow" and task.delivery:
                self._start_book_delivery(command, task.delivery)
            else:
                self._complete_book_task(command, "还书工作流已完成")
            return
        self._fail_book_task(command, message or "book workflow was cancelled or stopped")

    def _start_book_delivery(self, command: GatewayCommand, goal: NavigationGoal) -> None:
        if self._book_task_command is not command:
            return
        if not self._action_client.server_is_ready():
            self._fail_book_task(command, "Nav2 is unavailable before delivery")
            return
        self._book_task_phase = "navigating_to_delivery"
        self._pending_navigation_command = command
        ros_goal = NavigateToPose.Goal()
        ros_goal.pose = self._pose_from_goal(goal)
        future = self._action_client.send_goal_async(ros_goal, feedback_callback=self._navigation_feedback)
        future.add_done_callback(lambda result: self._handle_book_goal_response(command, "delivery", result))
        self._publish_event("book_task_delivery_dispatching", command, {"phase": self._book_task_phase})

    def _cancel_book_task(self, command: GatewayCommand, reason: str) -> None:
        active_command = self._book_task_command
        task = self._active_book_task
        if not active_command or not task:
            self._publish_event("command_rejected", command, {"reason": "no active book task"})
            return
        if command.command == "book_task_cancel" and command.payload.get("taskId") != task.task_id:
            self._publish_event("command_rejected", command, {"reason": "book taskId does not match the active task"})
            return
        workflow_id = self._bookarm_workflow_id
        if workflow_id:
            try:
                self._bookarm_client.cancel_workflow(workflow_id)
            except BookArmApiError as error:
                self.get_logger().warn(f"book workflow cancel failed: {error}")
        self._cancel_active_navigation(reason)
        self._publish_event("book_task_cancelled", active_command, {"reason": reason})
        self._clear_book_task()
        self._publish_event("command_completed", command, {"result": "book task cancellation requested"})

    def _complete_book_task(self, command: GatewayCommand, message: str) -> None:
        task = self._active_book_task
        if task:
            self._publish_event("book_task_succeeded", command, {"taskId": task.task_id, "message": message})
        self._clear_book_task()

    def _fail_book_task(self, command: GatewayCommand, reason: str) -> None:
        task = self._active_book_task
        if task and self._book_task_command is command:
            self._publish_event("book_task_failed", command, {"taskId": task.task_id, "reason": str(reason)[:300]})
            self._clear_book_task()
            return
        self._publish_event("command_rejected", command, {"reason": str(reason)[:300]})

    def _clear_book_task(self) -> None:
        self._active_book_task = None
        self._book_task_command = None
        self._book_task_phase = None
        self._bookarm_workflow_id = None
        self._book_task_last_state = None
        self._active_goal_handle = None
        self._active_command = None
        self._pending_navigation_command = None

    def _move_lift(self, command: GatewayCommand) -> None:
        if (
            self._active_book_task
            or self._bookarm_test_command
            or self._active_command
            or self._pending_navigation_command
            or self._manual_lease
        ):
            self._publish_event("command_rejected", command, {"reason": "cannot move lift while navigation or another control session is active"})
            return
        try:
            move: LiftMove = parse_lift_move(command.payload)
            health = self._bookarm_client.health()
            if not health.lift_ready:
                raise BookArmApiError(health.message)
            result = self._bookarm_client.move_lift(move.distance_cm)
        except (ValueError, BookArmApiError) as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        verified = bool(result.get("physical_completion_verified")) if isinstance(result, dict) else False
        event = "lift_move_completed" if verified else "lift_move_driver_acknowledged"
        self._publish_event(
            event,
            command,
            {
                "distanceCm": move.distance_cm,
                "result": result,
                "physicalCompletionVerified": verified,
                "warning": (
                    None
                    if verified
                    else "升降台驱动器已接收命令，但没有位置反馈，未确认物理到位"
                ),
            },
        )

    def _refresh_bookarm_health(self) -> None:
        self._bookarm_health = self._bookarm_client.health()

    def _send_next_cruise_waypoint(self) -> None:
        command = self._active_command
        route = self._active_cruise_route
        if not command or not route:
            return
        if self._cruise_waypoint_index >= len(route.waypoints):
            self._cruise_waypoint_index = 0
            self._cruise_loop_index += 1
        if self._cruise_loop_index >= route.loop_count:
            self._publish_event("cruise_succeeded", command)
            self._clear_cruise()
            return
        if datetime.now(UTC) >= command.expires_at:
            self._publish_event("cruise_failed", command, {"reason": "cruise command expired"})
            self._clear_cruise()
            return
        waypoint = route.waypoints[self._cruise_waypoint_index]
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_from_goal(
            NavigationGoal(route.map_id, waypoint.annotation_id, waypoint.x, waypoint.y, waypoint.yaw)
        )
        waypoint_index = self._cruise_waypoint_index
        loop_index = self._cruise_loop_index
        future = self._action_client.send_goal_async(goal, feedback_callback=self._navigation_feedback)
        future.add_done_callback(
            lambda result: self._handle_cruise_goal_response(
                command, route, waypoint_index, loop_index, result
            )
        )

    def _handle_cruise_goal_response(
        self,
        command: GatewayCommand,
        route: CruiseRoute,
        waypoint_index: int,
        loop_index: int,
        future,
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - ROS transport failures are runtime-only.
            if self._active_command is not command:
                return
            self._publish_event("cruise_failed", command, {"reason": str(error)})
            self._clear_cruise()
            return
        # A manual takeover or stop may have arrived while Nav2 was accepting
        # this waypoint. Cancel that late goal rather than allowing it to move
        # the robot after the cruise was cleared.
        if self._active_command is not command or self._active_cruise_route is not route:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self._publish_event("cruise_failed", command, {"reason": "Nav2 rejected cruise waypoint"})
            self._clear_cruise()
            return
        self._active_goal_handle = goal_handle
        waypoint = route.waypoints[waypoint_index]
        self._publish_event(
            "cruise_started",
            command,
            {
                "waypointIndex": waypoint_index + 1,
                "waypointCount": len(route.waypoints),
                "loopIndex": loop_index + 1,
                "loopCount": route.loop_count,
                "annotationId": waypoint.annotation_id,
            },
        )
        goal_handle.get_result_async().add_done_callback(
            lambda result: self._handle_cruise_goal_result(command, goal_handle, result)
        )

    def _handle_cruise_goal_result(self, command: GatewayCommand, goal_handle, future) -> None:
        if self._active_command is not command or self._active_goal_handle is not goal_handle:
            return
        self._active_goal_handle = None
        try:
            result = future.result()
        except Exception as error:  # pragma: no cover - ROS transport failures are runtime-only.
            self._publish_event("cruise_failed", command, {"reason": str(error)})
            self._clear_cruise()
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._cruise_waypoint_index += 1
            self._send_next_cruise_waypoint()
            return
        if result.status == GoalStatus.STATUS_CANCELED:
            self._publish_event("cruise_cancelled", command)
        else:
            self._publish_event("cruise_failed", command, {"nav2Status": int(result.status)})
        self._clear_cruise()

    def _clear_cruise(self) -> None:
        self._active_goal_handle = None
        self._active_command = None
        self._active_cruise_route = None
        self._cruise_waypoint_index = 0
        self._cruise_loop_index = 0

    def _pose_from_goal(self, goal: NavigationGoal) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        # Nav2 resolves a zero timestamp against the latest available TF. A
        # wall-clock stamp can become stale while MQTT/ROS queues a command,
        # leading to "extrapolation into the past" and a rejected goal.
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = goal.x
        pose.pose.position.y = goal.y
        pose.pose.orientation.z = math.sin(goal.yaw / 2.0)
        pose.pose.orientation.w = math.cos(goal.yaw / 2.0)
        return pose

    def _handle_goal_response(self, command: GatewayCommand, goal: NavigationGoal, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # ROS futures expose transport errors as exceptions.
            if self._pending_navigation_command is not command:
                return
            self._pending_navigation_command = None
            self._publish_event("navigation_failed", command, {"reason": str(error)})
            return
        # The operator can take over before Nav2 accepts an asynchronous goal.
        # Reject the late result locally and cancel it if Nav2 did accept it.
        if self._pending_navigation_command is not command:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self._pending_navigation_command = None
        if not goal_handle.accepted:
            self._publish_event("command_rejected", command, {"reason": "Nav2 rejected the navigation goal"})
            return
        self._active_command = command
        self._active_goal_handle = goal_handle
        self._publish_event(
            "navigation_started",
            command,
            {"mapId": goal.map_id, "annotationId": goal.annotation_id, "x": goal.x, "y": goal.y, "yaw": goal.yaw},
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._handle_navigation_result(command, result))

    def _navigation_feedback(self, _feedback) -> None:
        # Nav2 feedback is intentionally not mirrored at high frequency over MQTT.
        return

    def _handle_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != "map":
            return
        covariance = message.pose.covariance
        if (
            covariance[0] > self.max_position_variance
            or covariance[7] > self.max_position_variance
            or covariance[35] > self.max_yaw_variance
        ):
            self._latest_pose_received_at = None
            return
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw):
            self._latest_pose = (x, y, yaw)
            self._latest_pose_received_at = time.monotonic()

    def _handle_localization_status(self, message: String) -> None:
        status = message.data.strip()
        if status in {
            "starting",
            "restoring_saved_pose",
            "global_localizing",
            "manual_initializing",
            "localized",
            "manual_required",
        }:
            self._localization_status = status

    def _handle_localization_scan_mode(self, message: String) -> None:
        mode = message.data.strip()
        if mode in {"dual", "single_laser_1", "single_laser_2", "unavailable"}:
            self._localization_scan_mode = mode

    def _handle_chassis_status(self, message: String) -> None:
        """Accept only fresh, complete encoder feedback from the local base driver."""
        try:
            sources = json.loads(message.data)
        except json.JSONDecodeError:
            self._chassis_feedback_healthy = False
            self._chassis_status_received_at = time.monotonic()
            return
        self._chassis_feedback_healthy = (
            isinstance(sources, dict)
            and len(sources) == 4
            and all(source == "feedback" for source in sources.values())
        )
        self._chassis_status_received_at = time.monotonic()

    def _chassis_ready(self) -> bool:
        return bool(
            self._chassis_feedback_healthy
            and self._chassis_status_received_at is not None
            and time.monotonic() - self._chassis_status_received_at
            <= self.chassis_status_timeout_seconds
        )

    def _publish_telemetry(self) -> None:
        pose_is_fresh = (
            self._latest_pose_received_at is not None
            and time.monotonic() - self._latest_pose_received_at <= self.pose_stale_timeout_seconds
        )
        localization_status = self._localization_status
        if localization_status == "localized" and not pose_is_fresh:
            localization_status = "manual_required"
        details: dict[str, Any] = {
            "mapId": self.active_map_id,
            "frameId": "map",
            "nav2Ready": self._action_client.server_is_ready(),
            "chassisReady": self._chassis_ready(),
            "localizationAvailable": self._latest_pose is not None and pose_is_fresh,
            "localizationStatus": localization_status,
            "localizationScanMode": self._localization_scan_mode,
        }
        bookarm_health = getattr(self, "_bookarm_health", None)
        if isinstance(bookarm_health, BookArmHealth):
            details.update(
                {
                    "armReady": bookarm_health.arm_ready,
                    "armConnected": bookarm_health.arm_connected,
                    "armFeedbackHealthy": bookarm_health.arm_feedback_healthy,
                    "armCanFps": bookarm_health.arm_can_fps,
                    "armFeedbackReason": bookarm_health.arm_feedback_reason,
                    "liftReady": bookarm_health.lift_ready,
                    "liftAbsolutePositioningReady": bookarm_health.lift_absolute_positioning_ready,
                    "cameraReady": bookarm_health.camera_ready,
                    "rodReady": bookarm_health.rod_ready,
                    "bookArmMessage": bookarm_health.message,
                }
            )
        # An AMCL global-search candidate can be far from the real robot and
        # carries high covariance. Never render or dispatch it as a live pose.
        if self._latest_pose and pose_is_fresh and localization_status == "localized":
            x, y, yaw = self._latest_pose
            details.update({"x": x, "y": y, "yaw": yaw})
        self._publish_event("telemetry", details=details)

    def _handle_navigation_result(self, command: GatewayCommand, future) -> None:
        if self._active_command is not command:
            return
        self._active_goal_handle = None
        self._active_command = None
        try:
            result = future.result()
        except Exception as error:
            self._publish_event("navigation_failed", command, {"reason": str(error)})
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_event("navigation_succeeded", command)
        elif result.status == GoalStatus.STATUS_CANCELED:
            self._publish_event("navigation_cancelled", command)
        else:
            self._publish_event("navigation_failed", command, {"nav2Status": int(result.status)})

    def _start_manual_control(self, command: GatewayCommand) -> None:
        try:
            lease = parse_manual_control_start(command.payload)
        except ValueError as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        if self._active_book_task or self._bookarm_test_command:
            self._publish_event(
                "command_rejected",
                command,
                {"reason": "book task or book-arm maintenance test is active"},
            )
            return
        if not self._chassis_ready():
            self._publish_event(
                "command_rejected",
                command,
                {"reason": "chassis encoder feedback is unavailable or stale"},
            )
            return
        is_renewal = self._manual_lease is not None and self._manual_lease.lease_id == lease.lease_id
        if not is_renewal:
            self._cancel_active_navigation("manual control takeover")
        self._manual_lease = lease
        if not is_renewal:
            self._manual_command_deadline = None
            self._publish_zero_manual_velocity()
        self._publish_event(
            "manual_control_started",
            command,
            {
                "leaseId": lease.lease_id,
                "expiresAt": lease.expires_at.isoformat(),
                "maxLinearSpeed": lease.max_linear_speed,
                "maxAngularSpeed": lease.max_angular_speed,
            },
        )

    def _manual_jog(self, command: GatewayCommand) -> None:
        try:
            jog = parse_manual_jog(command.payload)
        except ValueError as error:
            self._publish_event("command_rejected", command, {"reason": str(error)})
            return
        lease = self._manual_lease
        now = datetime.now(UTC)
        if not lease or lease.lease_id != jog.lease_id or lease.expires_at <= now:
            expired_lease_id = lease.lease_id if lease and lease.expires_at <= now else None
            self._clear_manual_control()
            if expired_lease_id:
                self._publish_event("manual_control_expired", details={"leaseId": expired_lease_id})
            self._publish_event("command_rejected", command, {"reason": "manual control lease is invalid or expired"})
            return
        if (
            abs(jog.linear) > lease.max_linear_speed
            or abs(jog.angular) > lease.max_angular_speed
            or jog.duration_ms > lease.max_jog_duration_ms
        ):
            self._publish_event("command_rejected", command, {"reason": "manual jog exceeds active safety limits"})
            return
        if command.expires_at <= now:
            self._publish_event("command_rejected", command, {"reason": "manual jog command has expired"})
            return
        velocity = Twist()
        velocity.linear.x = jog.linear
        velocity.angular.z = jog.angular
        self._manual_velocity_publisher.publish(velocity)
        active_for_seconds = min(
            jog.duration_ms / 1000.0,
            (lease.expires_at - now).total_seconds(),
            self.manual_command_timeout_seconds,
        )
        self._manual_command_deadline = time.monotonic() + max(0.0, active_for_seconds)
        self._publish_event("manual_jog_accepted", command, {"durationMs": jog.duration_ms})

    def _stop_manual_control(self, command: GatewayCommand) -> None:
        lease_id = command.payload.get("leaseId")
        if not isinstance(lease_id, str) or not self._manual_lease or lease_id != self._manual_lease.lease_id:
            self._publish_event("command_rejected", command, {"reason": "manual control lease is invalid"})
            return
        self._clear_manual_control()
        self._publish_event("manual_control_stopped", command)

    def _enforce_manual_stop(self) -> None:
        lease = self._manual_lease
        if not lease:
            return
        if lease.expires_at <= datetime.now(UTC):
            self._clear_manual_control()
            self._publish_event("manual_control_expired", details={"leaseId": lease.lease_id})
            return
        if not self._chassis_ready():
            self._clear_manual_control()
            self._publish_event(
                "manual_control_stopped",
                details={"reason": "chassis encoder feedback is unavailable or stale"},
            )
            return
        if self._manual_command_deadline is not None and time.monotonic() >= self._manual_command_deadline:
            self._publish_zero_manual_velocity()
            self._manual_command_deadline = None

    def _publish_zero_manual_velocity(self) -> None:
        self._manual_velocity_publisher.publish(Twist())

    def _clear_manual_control(self) -> None:
        self._publish_zero_manual_velocity()
        self._manual_lease = None
        self._manual_command_deadline = None

    def _cancel_navigation(self, command: GatewayCommand) -> None:
        self._clear_manual_control()
        if self._bookarm_test_command:
            self._cancel_bookarm_test(command, "operator stopped book-arm maintenance test")
            return
        if self._active_book_task:
            self._cancel_book_task(command, "operator cancelled navigation")
            return
        if command.command == "emergency_stop":
            self._publish_event(
                "emergency_stop_requested",
                command,
                {"warning": "Only the active Nav2 goal was cancelled; use the physical emergency stop for hazards."},
            )
        if not self._cancel_active_navigation("operator cancelled navigation"):
            self._publish_event("command_completed", command, {"result": "no active Nav2 goal"})
            return
        self._publish_event("navigation_cancelling", command)

    def _cancel_active_navigation(self, reason: str) -> bool:
        """Cancel the active or still-pending Nav2 work without stale callbacks reviving it."""
        goal_handle = self._active_goal_handle
        active_command = self._active_command
        pending_command = self._pending_navigation_command
        is_cruise = self._active_cruise_route is not None

        self._active_goal_handle = None
        self._active_command = None
        self._active_cruise_route = None
        self._cruise_waypoint_index = 0
        self._cruise_loop_index = 0
        self._pending_navigation_command = None

        if active_command:
            self._publish_event(
                "cruise_cancelled" if is_cruise else "navigation_cancelled",
                active_command,
                {"reason": reason},
            )
        elif pending_command:
            self._publish_event("navigation_cancelled", pending_command, {"reason": reason})

        if goal_handle:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(lambda _result: None)
        return bool(goal_handle or active_command or pending_command)

    def _publish_event(
        self,
        event: str,
        command: GatewayCommand | None = None,
        details: dict[str, Any] | None = None,
    ) -> None:
        # Timers may fire before the asynchronous MQTT connection completes.
        if not self._mqtt.is_connected():
            return
        payload = {
            "eventId": f"EVT-{uuid4().hex[:12].upper()}",
            "event": event,
            "robotId": self.robot_id,
            "commandId": command.command_id if command else None,
            "timestamp": datetime.now(UTC).isoformat(),
            "details": details or {},
        }
        result = self._mqtt.publish(self.event_topic, json.dumps(payload), qos=1, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().warn(f"failed to publish {event}: MQTT result {result.rc}")

    def destroy_node(self) -> bool:
        # A process restart must be treated like a released manual-control lease.
        if getattr(self, "_manual_lease", None):
            self._clear_manual_control()
        if getattr(self, "_book_task_command", None):
            self._cancel_book_task(self._book_task_command, "robot runtime is stopping")
        if getattr(self, "_bookarm_test_command", None):
            self._cancel_bookarm_test(self._bookarm_test_command, "robot runtime is stopping")
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EdgeGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
