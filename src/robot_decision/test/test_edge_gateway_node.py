from datetime import UTC, datetime, timedelta
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

from robot_decision.bookarm_client import BookArmHealth
from robot_decision.edge_gateway_node import EdgeGatewayNode
from robot_decision.edge_gateway_protocol import BookTask, GatewayCommand, ManualControlStart


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[Twist] = []

    def publish(self, message: Twist) -> None:
        self.messages.append(message)


class FakeGoalHandle:
    def __init__(self) -> None:
        self.cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True
        return SimpleNamespace(add_done_callback=lambda callback: callback(None))


def test_gateway_rejects_an_unreliable_amcl_candidate() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    node.max_position_variance = 0.5
    node.max_yaw_variance = 0.35
    node._latest_pose = (1.0, 2.0, 0.3)
    node._latest_pose_received_at = 1.0

    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.6
    message.pose.covariance[7] = 0.1
    message.pose.covariance[35] = 0.1

    node._handle_amcl_pose(message)

    assert node._latest_pose == (1.0, 2.0, 0.3)
    assert node._latest_pose_received_at is None


def test_gateway_stale_pose_is_not_published_as_live(monkeypatch) -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    node._latest_pose = (1.0, 2.0, 0.3)
    node._latest_pose_received_at = 1.0
    node.pose_stale_timeout_seconds = 5.0
    node._localization_status = "localized"
    node._localization_scan_mode = "dual"
    node._chassis_feedback_healthy = True
    node._chassis_status_received_at = 9.0
    node.chassis_status_timeout_seconds = 2.5
    node.active_map_id = "MAP-01"
    node._action_client = SimpleNamespace(server_is_ready=lambda: True)
    events = []
    node._publish_event = lambda event, **kwargs: events.append((event, kwargs["details"]))
    monkeypatch.setattr("robot_decision.edge_gateway_node.time.monotonic", lambda: 10.0)

    node._publish_telemetry()

    assert events == [
        (
            "telemetry",
            {
                "mapId": "MAP-01",
                "frameId": "map",
                "nav2Ready": True,
                "chassisReady": True,
                "localizationAvailable": False,
                "localizationStatus": "manual_required",
                "localizationScanMode": "dual",
            },
        )
    ]


def test_gateway_telemetry_includes_non_actuating_arm_feedback_diagnostics(monkeypatch) -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    node._latest_pose = None
    node._latest_pose_received_at = None
    node.pose_stale_timeout_seconds = 5.0
    node._localization_status = "manual_required"
    node._localization_scan_mode = "dual"
    node._chassis_feedback_healthy = True
    node._chassis_status_received_at = 9.0
    node.chassis_status_timeout_seconds = 2.5
    node.active_map_id = "MAP-01"
    node._action_client = SimpleNamespace(server_is_ready=lambda: True)
    node._bookarm_health = BookArmHealth(
        available=True,
        arm_ready=False,
        lift_ready=True,
        camera_ready=True,
        rod_ready=True,
        message="机械臂本机通信已打开，但 CAN 反馈未就绪",
        arm_connected=True,
        arm_feedback_healthy=False,
        arm_can_fps=0.0,
        arm_feedback_reason="未收到 CAN 反馈帧",
    )
    events = []
    node._publish_event = lambda event, **kwargs: events.append((event, kwargs["details"]))
    monkeypatch.setattr("robot_decision.edge_gateway_node.time.monotonic", lambda: 10.0)

    node._publish_telemetry()

    assert events[0][1]["armConnected"] is True
    assert events[0][1]["armFeedbackHealthy"] is False
    assert events[0][1]["armCanFps"] == 0.0
    assert events[0][1]["armFeedbackReason"] == "未收到 CAN 反馈帧"


def test_gateway_requires_recent_encoder_feedback_for_manual_control(monkeypatch) -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    node._chassis_feedback_healthy = True
    node._chassis_status_received_at = 1.0
    node.chassis_status_timeout_seconds = 2.5
    monkeypatch.setattr("robot_decision.edge_gateway_node.time.monotonic", lambda: 4.0)

    assert node._chassis_ready() is False


def test_cancelling_cruise_cancels_the_existing_nav2_goal() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    command = GatewayCommand("CMD-001", "LR-01", "cruise", {}, datetime.now(UTC) + timedelta(minutes=1))
    goal_handle = FakeGoalHandle()
    events = []
    node._active_goal_handle = goal_handle
    node._active_command = command
    node._pending_navigation_command = None
    node._active_cruise_route = object()
    node._cruise_waypoint_index = 1
    node._cruise_loop_index = 0
    node._publish_event = lambda event, command=None, details=None: events.append((event, command, details))

    assert node._cancel_active_navigation("manual control takeover") is True
    assert goal_handle.cancelled is True
    assert node._active_goal_handle is None
    assert node._active_command is None
    assert events == [("cruise_cancelled", command, {"reason": "manual control takeover"})]


def test_expired_manual_lease_publishes_zero_velocity_and_expiry_event() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    lease = ManualControlStart(
        lease_id="lease-001",
        expires_at=datetime.now(UTC) - timedelta(seconds=1),
        max_linear_speed=0.15,
        max_angular_speed=0.3,
        max_jog_duration_ms=750,
    )
    publisher = FakePublisher()
    events = []
    node._manual_lease = lease
    node._manual_command_deadline = None
    node._manual_velocity_publisher = publisher
    node._publish_event = lambda event, command=None, details=None: events.append((event, command, details))

    node._enforce_manual_stop()

    assert publisher.messages[-1].linear.x == 0.0
    assert publisher.messages[-1].angular.z == 0.0
    assert node._manual_lease is None
    assert events == [("manual_control_expired", None, {"leaseId": "lease-001"})]


def test_navigation_goal_uses_latest_available_transform() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    pose = node._pose_from_goal(SimpleNamespace(x=1.0, y=-0.5, yaw=0.25))

    assert pose.header.frame_id == "map"
    assert pose.header.stamp.sec == 0
    assert pose.header.stamp.nanosec == 0


def test_gateway_requires_client_certificate_for_mqtt_tls() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    node.allowed_map_ids = {"MAP-01"}
    node.active_map_id = "MAP-01"
    node.map_min_x = -1.0
    node.map_max_x = 1.0
    node.map_min_y = -1.0
    node.map_max_y = 1.0
    node.max_abs_yaw = 6.284
    node.telemetry_period_seconds = 0.5
    node.pose_stale_timeout_seconds = 5.0
    node.max_position_variance = 0.5
    node.max_yaw_variance = 0.35
    node.manual_command_timeout_seconds = 1.0
    node.chassis_status_timeout_seconds = 2.5
    node.mqtt_tls_enabled = True
    node.mqtt_ca_file = "/tmp/ca.crt"
    node.mqtt_client_cert_file = ""
    node.mqtt_client_key_file = ""
    node.mqtt_protocol_version = "3.1.1"
    node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)

    with pytest.raises(RuntimeError, match="client certificate"):
        node._validate_configuration()


def test_book_workflow_confirmation_rejects_an_outdated_task_id() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    active_command = GatewayCommand("CMD-BOOK", "LR-01", "book_task", {}, datetime.now(UTC) + timedelta(minutes=1))
    stale_command = GatewayCommand(
        "CMD-CONTINUE", "LR-01", "book_task_continue", {"taskId": "TSK-OLD"}, datetime.now(UTC) + timedelta(minutes=1)
    )
    node._active_book_task = BookTask(
        "TSK-ACTIVE", "return", "putback", "CPY-001", "测试图书", object(), None, object()
    )
    node._book_task_command = active_command
    node._bookarm_workflow_id = "workflow-001"
    node._bookarm_client = SimpleNamespace(workflow=lambda _workflow_id: pytest.fail("workflow must not be read"))
    events = []
    node._publish_event = lambda event, command=None, details=None: events.append((event, command, details))

    node._continue_book_task(stale_command)

    assert events == [
        ("command_rejected", stale_command, {"reason": "book taskId does not match the active task"})
    ]


def test_book_task_cancel_rejects_an_outdated_task_id() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    active_command = GatewayCommand("CMD-BOOK", "LR-01", "book_task", {}, datetime.now(UTC) + timedelta(minutes=1))
    stale_command = GatewayCommand(
        "CMD-CANCEL", "LR-01", "book_task_cancel", {"taskId": "TSK-OLD"}, datetime.now(UTC) + timedelta(minutes=1)
    )
    node._active_book_task = BookTask(
        "TSK-ACTIVE", "return", "putback", "CPY-001", "测试图书", object(), None, object()
    )
    node._book_task_command = active_command
    node._bookarm_workflow_id = None
    events = []
    node._publish_event = lambda event, command=None, details=None: events.append((event, command, details))

    node._cancel_book_task(stale_command, "operator cancelled book task")

    assert events == [
        ("command_rejected", stale_command, {"reason": "book taskId does not match the active task"})
    ]


def test_bookarm_maintenance_test_starts_in_manual_step_mode() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    node._manual_lease = None
    node._active_book_task = None
    node._bookarm_test_command = None
    node._active_command = None
    node._pending_navigation_command = None
    node._bookarm_workflow_id = None
    node._bookarm_test_last_state = None
    health = SimpleNamespace(
        lift_ready=True,
        camera_ready=True,
        rod_ready=True,
        arm_ready=True,
        message="ready",
    )
    calls = []
    node._bookarm_client = SimpleNamespace(
        health=lambda: health,
        prepare_arm=lambda: health,
        start_workflow=lambda mode, auto: calls.append((mode, auto))
        or {
            "id": "workflow-001",
            "active": True,
            "waiting": False,
            "step_index": 0,
            "step_count": 16,
            "step_name": "Capture RGB-D point cloud",
            "message": "ready",
        },
    )
    events = []
    node._publish_event = lambda event, command=None, details=None: events.append((event, command, details))
    command = GatewayCommand(
        "CMD-MNT",
        "LR-01",
        "bookarm_test_start",
        {
            "sessionId": "MNT-001",
            "workflowMode": "takeout",
            "stepMode": "manual",
            "acknowledgeSafety": True,
        },
        datetime.now(UTC) + timedelta(minutes=1),
    )

    node._start_bookarm_test(command)

    assert calls == [("takeout", False)]
    assert node._bookarm_test_session.session_id == "MNT-001"
    assert events[0][0] == "bookarm_test_started"
    assert events[1][0] == "bookarm_test_workflow"
    assert events[1][2]["stepIndex"] == 0


def test_lift_command_reports_driver_acknowledgement_not_physical_completion() -> None:
    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    node._active_book_task = None
    node._bookarm_test_command = None
    node._active_command = None
    node._pending_navigation_command = None
    node._manual_lease = None
    health = SimpleNamespace(lift_ready=True, message="ready")
    node._bookarm_client = SimpleNamespace(
        health=lambda: health,
        move_lift=lambda _distance: {
            "accepted": True,
            "completion": "driver_acknowledged",
            "physical_completion_verified": False,
        },
    )
    events = []
    node._publish_event = lambda event, command=None, details=None: events.append((event, command, details))
    command = GatewayCommand(
        "CMD-LIFT",
        "LR-01",
        "lift_move",
        {"distanceCm": 1.0, "acknowledgeSafety": True},
        datetime.now(UTC) + timedelta(minutes=1),
    )

    node._move_lift(command)

    assert events == [
        (
            "lift_move_driver_acknowledged",
            command,
            {
                "distanceCm": 1.0,
                "result": {
                    "accepted": True,
                    "completion": "driver_acknowledged",
                    "physical_completion_verified": False,
                },
                "physicalCompletionVerified": False,
                "warning": "升降台驱动器已接收命令，但没有位置反馈，未确认物理到位",
            },
        )
    ]


def test_bookarm_maintenance_state_forwards_bounded_recognition_result() -> None:
    from robot_decision.edge_gateway_protocol import BookArmTestSession

    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    command = GatewayCommand(
        "CMD-MNT",
        "LR-01",
        "bookarm_test_start",
        {},
        datetime.now(UTC) + timedelta(minutes=1),
    )
    node._bookarm_test_session = BookArmTestSession("MNT-001", "takeout")
    node._bookarm_test_command = command
    node._bookarm_workflow_id = "workflow-001"
    node._bookarm_test_last_state = None
    events = []
    node._publish_event = lambda event, command=None, details=None: events.append((event, command, details))

    node._publish_bookarm_test_state(
        command,
        {
            "active": True,
            "waiting": False,
            "step_index": 2,
            "step_count": 16,
            "step_name": "Solve target point",
            "message": "book detected",
            "recognition_result": {
                "confidence": 0.91,
                "good_count": 24,
                "inlier_count": 17,
                "pixel_uv": [640, 360],
                "target_point_m": [0.2, 0.1, 0.3],
            },
        },
    )

    assert events[0][0] == "bookarm_test_workflow"
    assert events[0][2]["recognitionResult"]["confidence"] == 0.91


def test_bookarm_maintenance_state_forwards_takeout_debug_preflight() -> None:
    from robot_decision.edge_gateway_protocol import BookArmTestSession

    node = EdgeGatewayNode.__new__(EdgeGatewayNode)
    command = GatewayCommand(
        "CMD-MNT",
        "LR-01",
        "bookarm_test_start",
        {},
        datetime.now(UTC) + timedelta(minutes=1),
    )
    node._bookarm_test_session = BookArmTestSession("MNT-001", "takeout")
    node._bookarm_test_command = command
    node._bookarm_workflow_id = "workflow-001"
    node._bookarm_test_last_state = None
    events = []
    node._publish_event = lambda event, command=None, details=None: events.append((event, command, details))

    node._publish_bookarm_test_state(
        command,
        {
            "active": True,
            "waiting": False,
            "step_index": 0,
            "step_count": 16,
            "step_name": "Capture RGB-D point cloud",
            "message": "startup preparation required",
            "preflight_required": True,
            "preflight_complete": False,
            "preflight_state": "required",
            "preflight_name": "MoveJ to book debug pose",
        },
    )

    details = events[0][2]
    assert details["preflightRequired"] is True
    assert details["preflightComplete"] is False
    assert details["preflightState"] == "required"
    assert details["preflightName"] == "MoveJ to book debug pose"
