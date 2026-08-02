"""Validation shared by the MQTT callback and the ROS 2 Edge Gateway node."""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import Any


_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9_-]{1,80}$")
_SUPPORTED_COMMANDS = frozenset(
    {
        "navigate",
        "cruise",
        "set_initial_pose",
        "start_manual_control",
        "manual_jog",
        "stop_manual_control",
        "book_task",
        "book_task_continue",
        "book_task_cancel",
        "bookarm_test_start",
        "bookarm_test_continue",
        "bookarm_test_cancel",
        "lift_move",
        "cancel",
        "stop",
        "emergency_stop",
    }
)


@dataclass(frozen=True)
class GatewayCommand:
    command_id: str
    robot_id: str
    command: str
    payload: dict[str, Any]
    expires_at: datetime


@dataclass(frozen=True)
class NavigationGoal:
    map_id: str
    annotation_id: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class InitialPose:
    """An operator-confirmed pose used to initialize AMCL on the active map."""

    map_id: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CruiseWaypoint:
    annotation_id: str
    label: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CruiseRoute:
    map_id: str
    loop_count: int
    waypoints: tuple[CruiseWaypoint, ...]


@dataclass(frozen=True)
class ManualControlStart:
    lease_id: str
    expires_at: datetime
    max_linear_speed: float
    max_angular_speed: float
    max_jog_duration_ms: int


@dataclass(frozen=True)
class ManualJog:
    lease_id: str
    linear: float
    angular: float
    duration_ms: int


@dataclass(frozen=True)
class BookTask:
    """A center-approved circulation task executed only on the robot."""

    task_id: str
    operation: str
    workflow_mode: str
    book_id: str
    book_title: str
    shelf: NavigationGoal
    delivery: NavigationGoal | None
    shelf_level: "ShelfLevelTarget"


@dataclass(frozen=True)
class ShelfLevelTarget:
    """A calibrated target accepted only by a gateway with absolute lift feedback."""

    level_id: str
    level_no: int
    slot_no: int
    lift_target_mm: float
    lift_tolerance_mm: float


@dataclass(frozen=True)
class LiftMove:
    """A bounded maintenance lift adjustment in centimetres."""

    distance_cm: float


@dataclass(frozen=True)
class BookArmTestSession:
    """A maintenance-only, step-confirmed MotorStudio workflow."""

    session_id: str
    workflow_mode: str


def parse_gateway_command(raw_payload: bytes, expected_robot_id: str, now: datetime | None = None) -> GatewayCommand:
    """Parse a platform command without trusting the MQTT payload."""
    if len(raw_payload) > 64 * 1024:
        raise ValueError("command payload exceeds 64 KiB")
    try:
        document = json.loads(raw_payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("command payload is not valid UTF-8 JSON") from error
    if not isinstance(document, dict):
        raise ValueError("command payload must be an object")

    command_id = _read_identifier(document, "commandId")
    robot_id = _read_robot_id(document, "robotId")
    if robot_id != expected_robot_id:
        raise ValueError("command robotId does not match this gateway")
    command = document.get("command")
    if command not in _SUPPORTED_COMMANDS:
        raise ValueError("command is not permitted on the Edge Gateway")
    payload = document.get("payload")
    if not isinstance(payload, dict):
        raise ValueError("command payload field must be an object")
    if document.get("safetyLevel") != "high":
        raise ValueError("command safetyLevel must be high")

    expires_at = _parse_timestamp(document.get("expiresAt"), "expiresAt")
    current_time = now or datetime.now(UTC)
    if expires_at <= current_time:
        raise ValueError("command has expired")
    return GatewayCommand(
        command_id=command_id,
        robot_id=robot_id,
        command=command,
        payload=payload,
        expires_at=expires_at,
    )


def parse_navigation_goal(
    payload: dict[str, Any],
    allowed_map_ids: set[str],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    max_abs_yaw: float,
) -> NavigationGoal:
    """Validate a map-frame target against the map explicitly enabled on the robot."""
    if not allowed_map_ids:
        raise ValueError("gateway has no allowed map IDs configured")
    map_id = _read_identifier(payload, "mapId")
    if map_id not in allowed_map_ids:
        raise ValueError("mapId is not enabled on this robot")
    annotation_id = _read_identifier(payload, "annotationId")
    if payload.get("frameId") != "map":
        raise ValueError("navigation frameId must be map")
    x = _read_finite_number(payload, "x")
    y = _read_finite_number(payload, "y")
    yaw = _read_finite_number(payload, "yaw")
    if not min_x <= x <= max_x or not min_y <= y <= max_y:
        raise ValueError("navigation target is outside configured map bounds")
    if abs(yaw) > max_abs_yaw:
        raise ValueError("navigation yaw is outside the permitted range")
    return NavigationGoal(map_id=map_id, annotation_id=annotation_id, x=x, y=y, yaw=yaw)


def parse_initial_pose(
    payload: dict[str, Any],
    allowed_map_ids: set[str],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    max_abs_yaw: float,
) -> InitialPose:
    """Validate a map-frame AMCL initialization pose from a trusted operator."""
    if not allowed_map_ids:
        raise ValueError("gateway has no allowed map IDs configured")
    map_id = _read_identifier(payload, "mapId")
    if map_id not in allowed_map_ids:
        raise ValueError("mapId is not enabled on this robot")
    if payload.get("frameId") != "map":
        raise ValueError("initial pose frameId must be map")
    x = _read_finite_number(payload, "x")
    y = _read_finite_number(payload, "y")
    yaw = _read_finite_number(payload, "yaw")
    if not min_x <= x <= max_x or not min_y <= y <= max_y:
        raise ValueError("initial pose is outside configured map bounds")
    if abs(yaw) > max_abs_yaw:
        raise ValueError("initial pose yaw is outside the permitted range")
    return InitialPose(map_id=map_id, x=x, y=y, yaw=yaw)


def parse_cruise_route(
    payload: dict[str, Any],
    allowed_map_ids: set[str],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    max_abs_yaw: float,
) -> CruiseRoute:
    """Validate the ordered local route that the gateway will execute."""
    if payload.get("frameId") != "map":
        raise ValueError("cruise frameId must be map")
    map_id = _read_identifier(payload, "mapId")
    if map_id not in allowed_map_ids:
        raise ValueError("mapId is not enabled on this robot")
    loop_count = payload.get("loopCount")
    if isinstance(loop_count, bool) or not isinstance(loop_count, int) or not 1 <= loop_count <= 10:
        raise ValueError("cruise loopCount must be an integer from 1 to 10")
    raw_waypoints = payload.get("waypoints")
    if not isinstance(raw_waypoints, list) or not 2 <= len(raw_waypoints) <= 50:
        raise ValueError("cruise must contain 2 to 50 waypoints")

    waypoints: list[CruiseWaypoint] = []
    for raw_waypoint in raw_waypoints:
        if not isinstance(raw_waypoint, dict):
            raise ValueError("cruise waypoint must be an object")
        annotation_id = _read_identifier(raw_waypoint, "annotationId")
        label = raw_waypoint.get("label")
        if not isinstance(label, str) or not label.strip() or len(label) > 120:
            raise ValueError("cruise waypoint label must be a non-empty string")
        x = _read_finite_number(raw_waypoint, "x")
        y = _read_finite_number(raw_waypoint, "y")
        yaw = _read_finite_number(raw_waypoint, "yaw")
        if not min_x <= x <= max_x or not min_y <= y <= max_y:
            raise ValueError("cruise waypoint is outside configured map bounds")
        if abs(yaw) > max_abs_yaw:
            raise ValueError("cruise waypoint yaw is outside the permitted range")
        waypoints.append(CruiseWaypoint(annotation_id, label.strip(), x, y, yaw))
    return CruiseRoute(map_id=map_id, loop_count=loop_count, waypoints=tuple(waypoints))


def parse_manual_control_start(
    payload: dict[str, Any], now: datetime | None = None
) -> ManualControlStart:
    """Validate a short-lived, server-issued manual-control lease."""
    lease_id = _read_identifier(payload, "leaseId")
    expires_at = _parse_timestamp(payload.get("expiresAt"), "expiresAt")
    if expires_at <= (now or datetime.now(UTC)):
        raise ValueError("manual control lease has expired")
    max_linear_speed = _read_finite_number(payload, "maxLinearSpeed")
    max_angular_speed = _read_finite_number(payload, "maxAngularSpeed")
    max_jog_duration_ms = payload.get("maxJogDurationMs")
    if not 0.0 < max_linear_speed <= 0.15 or not 0.0 < max_angular_speed <= 0.3:
        raise ValueError("manual control speed limits exceed gateway safety limits")
    if (
        isinstance(max_jog_duration_ms, bool)
        or not isinstance(max_jog_duration_ms, int)
        or not 100 <= max_jog_duration_ms <= 750
    ):
        raise ValueError("manual control jog duration limit is invalid")
    return ManualControlStart(
        lease_id=lease_id,
        expires_at=expires_at,
        max_linear_speed=max_linear_speed,
        max_angular_speed=max_angular_speed,
        max_jog_duration_ms=max_jog_duration_ms,
    )


def parse_manual_jog(payload: dict[str, Any]) -> ManualJog:
    """Validate one bounded manual velocity request; authorization is node state."""
    lease_id = _read_identifier(payload, "leaseId")
    linear = _read_finite_number(payload, "linear")
    angular = _read_finite_number(payload, "angular")
    duration_ms = payload.get("durationMs")
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int) or not 100 <= duration_ms <= 750:
        raise ValueError("manual jog duration must be 100 to 750 ms")
    if payload.get("stopOnExpiry") is not True:
        raise ValueError("manual jog must require a stop on expiry")
    return ManualJog(lease_id=lease_id, linear=linear, angular=angular, duration_ms=duration_ms)


def parse_book_task(
    payload: dict[str, Any],
    allowed_map_ids: set[str],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    max_abs_yaw: float,
) -> BookTask:
    """Validate a book task without accepting any client-supplied arm pose."""
    task_id = _read_identifier(payload, "taskId")
    operation = payload.get("operation")
    if operation not in {"borrow", "return"}:
        raise ValueError("book task operation must be borrow or return")
    workflow_mode = payload.get("workflowMode")
    expected_modes = {"borrow": {"takeout"}, "return": {"putback", "putback2", "tail_putback"}}
    if workflow_mode not in expected_modes[operation]:
        raise ValueError("book task workflow mode is not permitted for this operation")
    book = payload.get("book")
    if not isinstance(book, dict):
        raise ValueError("book task book must be an object")
    book_id = _read_identifier(book, "id")
    book_title = book.get("title")
    if not isinstance(book_title, str) or not book_title.strip() or len(book_title) > 160:
        raise ValueError("book task title must be a non-empty string up to 160 characters")
    shelf_payload = payload.get("shelf")
    if not isinstance(shelf_payload, dict):
        raise ValueError("book task shelf must be an object")
    shelf = parse_navigation_goal(
        shelf_payload, allowed_map_ids, min_x, max_x, min_y, max_y, max_abs_yaw
    )
    shelf_level = parse_shelf_level_target(payload.get("shelfLevel"))
    delivery_payload = payload.get("delivery")
    if delivery_payload is None:
        delivery = None
    elif isinstance(delivery_payload, dict):
        delivery = parse_navigation_goal(
            delivery_payload, allowed_map_ids, min_x, max_x, min_y, max_y, max_abs_yaw
        )
    else:
        raise ValueError("book task delivery must be an object when provided")
    if operation == "borrow" and delivery is None:
        raise ValueError("borrow task requires a delivery point")
    if operation == "return" and delivery is not None:
        raise ValueError("return task must not include a delivery point")
    return BookTask(
        task_id=task_id,
        operation=operation,
        workflow_mode=workflow_mode,
        book_id=book_id,
        book_title=book_title.strip(),
        shelf=shelf,
        delivery=delivery,
        shelf_level=shelf_level,
    )


def parse_shelf_level_target(value: Any) -> ShelfLevelTarget:
    """Validate an absolute lift target but never move hardware from this parser."""
    if not isinstance(value, dict):
        raise ValueError("book task shelfLevel must be an object")
    level_id = _read_identifier(value, "id")
    level_no = value.get("levelNo")
    slot_no = value.get("slotNo")
    if isinstance(level_no, bool) or not isinstance(level_no, int) or not 1 <= level_no <= 99:
        raise ValueError("book task shelfLevel levelNo must be 1 to 99")
    if isinstance(slot_no, bool) or not isinstance(slot_no, int) or not 1 <= slot_no <= 500:
        raise ValueError("book task shelfLevel slotNo must be 1 to 500")
    lift_target_mm = _read_finite_number(value, "liftTargetMm")
    lift_tolerance_mm = _read_finite_number(value, "liftToleranceMm")
    if not 0.0 <= lift_target_mm <= 10_000.0:
        raise ValueError("book task shelfLevel liftTargetMm is outside the permitted range")
    if not 0.0 < lift_tolerance_mm <= 100.0:
        raise ValueError("book task shelfLevel liftToleranceMm is outside the permitted range")
    return ShelfLevelTarget(level_id, level_no, slot_no, lift_target_mm, lift_tolerance_mm)


def parse_lift_move(payload: dict[str, Any]) -> LiftMove:
    """Bound direct maintenance moves; the local worker remains authoritative."""
    if payload.get("acknowledgeSafety") is not True:
        raise ValueError("lift move requires acknowledgeSafety=true")
    distance_cm = _read_finite_number(payload, "distanceCm")
    if distance_cm == 0.0 or abs(distance_cm) > 10.0:
        raise ValueError("lift move distance must be greater than 0 and no more than 10 cm")
    return LiftMove(distance_cm=distance_cm)


def parse_bookarm_test_start(payload: dict[str, Any]) -> BookArmTestSession:
    if payload.get("acknowledgeSafety") is not True:
        raise ValueError("book-arm maintenance test requires acknowledgeSafety=true")
    session_id = _read_identifier(payload, "sessionId")
    workflow_mode = payload.get("workflowMode")
    if workflow_mode not in {"takeout", "putback", "putback2", "tail_putback"}:
        raise ValueError("book-arm maintenance workflow mode is not permitted")
    if payload.get("stepMode") != "manual":
        raise ValueError("book-arm maintenance test must use manual step mode")
    return BookArmTestSession(session_id=session_id, workflow_mode=workflow_mode)


def parse_bookarm_test_session_id(payload: dict[str, Any]) -> str:
    return _read_identifier(payload, "sessionId")


def _read_identifier(document: dict[str, Any], key: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not _IDENTIFIER_PATTERN.fullmatch(value):
        raise ValueError(f"{key} must be a 1-80 character identifier")
    return value


def _read_robot_id(document: dict[str, Any], key: str) -> str:
    value = _read_identifier(document, key)
    if not value.startswith("LR-"):
        raise ValueError(f"{key} must start with LR-")
    return value


def _read_finite_number(document: dict[str, Any], key: str) -> float:
    value = document.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{key} must be finite")
    return number


def _parse_timestamp(value: Any, key: str) -> datetime:
    if not isinstance(value, str):
        raise ValueError(f"{key} must be an ISO-8601 timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError(f"{key} must be an ISO-8601 timestamp") from error
    if parsed.tzinfo is None:
        raise ValueError(f"{key} must include a timezone")
    return parsed.astimezone(UTC)
