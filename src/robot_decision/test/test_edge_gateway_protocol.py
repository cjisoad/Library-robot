from datetime import UTC, datetime, timedelta

import pytest

from robot_decision.edge_gateway_protocol import (
    parse_cruise_route,
    parse_gateway_command,
    parse_initial_pose,
    parse_manual_control_start,
    parse_manual_jog,
    parse_navigation_goal,
    parse_book_task,
    parse_bookarm_test_session_id,
    parse_bookarm_test_start,
    parse_lift_move,
)


def test_parse_navigation_command() -> None:
    expires_at = datetime.now(UTC) + timedelta(minutes=1)
    command = parse_gateway_command(
        (
            '{"commandId":"CMD-001","robotId":"LR-01","command":"navigate",'
            '"payload":{"mapId":"MAP-01","annotationId":"ANN-01","frameId":"map",'
            '"x":1.2,"y":-0.8,"yaw":1.57},"expiresAt":"'
            + expires_at.isoformat()
            + '","safetyLevel":"high"}'
        ).encode(),
        "LR-01",
    )

    goal = parse_navigation_goal(
        command.payload,
        {"MAP-01"},
        min_x=-2.0,
        max_x=2.0,
        min_y=-2.0,
        max_y=2.0,
        max_abs_yaw=6.284,
    )

    assert goal.map_id == "MAP-01"
    assert goal.x == 1.2


@pytest.mark.parametrize(
    ("payload", "reason"),
    [
        (b"not json", "not valid UTF-8 JSON"),
        (
            b'{"commandId":"CMD-001","robotId":"LR-02","command":"navigate",'
            b'"payload":{},"expiresAt":"2099-01-01T00:00:00+00:00","safetyLevel":"high"}',
            "does not match",
        ),
        (
            b'{"commandId":"CMD-001","robotId":"LR-01","command":"navigate",'
            b'"payload":{},"expiresAt":"2000-01-01T00:00:00+00:00","safetyLevel":"high"}',
            "expired",
        ),
    ],
)
def test_rejects_unsafe_commands(payload: bytes, reason: str) -> None:
    with pytest.raises(ValueError, match=reason):
        parse_gateway_command(payload, "LR-01")


def test_rejects_target_on_other_map_or_outside_bounds() -> None:
    with pytest.raises(ValueError, match="not enabled"):
        parse_navigation_goal(
            {"mapId": "MAP-OTHER", "annotationId": "ANN-01", "frameId": "map", "x": 0, "y": 0, "yaw": 0},
            {"MAP-01"},
            min_x=-1,
            max_x=1,
            min_y=-1,
            max_y=1,
            max_abs_yaw=6.284,
        )
    with pytest.raises(ValueError, match="outside configured"):
        parse_navigation_goal(
            {"mapId": "MAP-01", "annotationId": "ANN-01", "frameId": "map", "x": 2, "y": 0, "yaw": 0},
            {"MAP-01"},
            min_x=-1,
            max_x=1,
            min_y=-1,
            max_y=1,
            max_abs_yaw=6.284,
        )


def test_parse_web_confirmed_initial_pose() -> None:
    pose = parse_initial_pose(
        {"mapId": "MAP-01", "frameId": "map", "x": 0.5, "y": -0.5, "yaw": 1.57},
        {"MAP-01"},
        min_x=-1,
        max_x=1,
        min_y=-1,
        max_y=1,
        max_abs_yaw=6.284,
    )
    assert pose.map_id == "MAP-01"
    assert pose.yaw == 1.57


def test_parse_cruise_route_preserves_operator_selected_order() -> None:
    route = parse_cruise_route(
        {
            "mapId": "MAP-01",
            "frameId": "map",
            "loopCount": 2,
            "waypoints": [
                {"annotationId": "ANN-SECOND", "label": "第二点", "x": 0.5, "y": 0.2, "yaw": 0},
                {"annotationId": "ANN-FIRST", "label": "第一点", "x": -0.5, "y": -0.2, "yaw": 1.57},
            ],
        },
        {"MAP-01"},
        min_x=-1,
        max_x=1,
        min_y=-1,
        max_y=1,
        max_abs_yaw=6.284,
    )

    assert route.loop_count == 2
    assert [waypoint.annotation_id for waypoint in route.waypoints] == ["ANN-SECOND", "ANN-FIRST"]


def test_manual_protocol_rejects_unsafe_limits_and_missing_stop_requirement() -> None:
    expires_at = datetime.now(UTC) + timedelta(minutes=1)
    with pytest.raises(ValueError, match="speed limits"):
        parse_manual_control_start(
            {
                "leaseId": "lease-001",
                "expiresAt": expires_at.isoformat(),
                "maxLinearSpeed": 0.2,
                "maxAngularSpeed": 0.3,
                "maxJogDurationMs": 750,
            }
        )
    with pytest.raises(ValueError, match="stop on expiry"):
        parse_manual_jog(
            {"leaseId": "lease-001", "linear": 0.1, "angular": 0, "durationMs": 500, "stopOnExpiry": False}
        )


def test_book_task_only_accepts_server_bounded_navigation_and_workflow_modes() -> None:
    task = parse_book_task(
        {
            "taskId": "TSK-001",
            "operation": "borrow",
            "workflowMode": "takeout",
            "book": {"id": "CPY-001", "title": "机器人学导论"},
            "shelf": {"mapId": "MAP-01", "annotationId": "ANN-SHELF", "frameId": "map", "x": 0, "y": 0, "yaw": 0},
            "shelfLevel": {"id": "SLV-001", "levelNo": 2, "slotNo": 11, "liftTargetMm": 680, "liftToleranceMm": 5},
            "delivery": {"mapId": "MAP-01", "annotationId": "ANN-DELIVERY", "frameId": "map", "x": 0.5, "y": 0.2, "yaw": 0},
        },
        {"MAP-01"},
        min_x=-1,
        max_x=1,
        min_y=-1,
        max_y=1,
        max_abs_yaw=6.284,
    )
    assert task.operation == "borrow"
    assert task.delivery is not None
    assert task.shelf_level.lift_target_mm == 680

    with pytest.raises(ValueError, match="workflow mode"):
        parse_book_task(
            {
                "taskId": "TSK-001",
                "operation": "return",
                "workflowMode": "takeout",
                "book": {"id": "CPY-001", "title": "机器人学导论"},
                "shelf": {"mapId": "MAP-01", "annotationId": "ANN-SHELF", "frameId": "map", "x": 0, "y": 0, "yaw": 0},
                "shelfLevel": {"id": "SLV-001", "levelNo": 2, "slotNo": 11, "liftTargetMm": 680, "liftToleranceMm": 5},
            },
            {"MAP-01"},
            min_x=-1,
            max_x=1,
            min_y=-1,
            max_y=1,
            max_abs_yaw=6.284,
        )


def test_lift_move_requires_explicit_acknowledgement_and_has_a_small_bound() -> None:
    assert parse_lift_move({"distanceCm": -2.5, "acknowledgeSafety": True}).distance_cm == -2.5
    with pytest.raises(ValueError, match="acknowledgeSafety"):
        parse_lift_move({"distanceCm": 1.0})
    with pytest.raises(ValueError, match="no more than 10"):
        parse_lift_move({"distanceCm": 10.1, "acknowledgeSafety": True})


def test_bookarm_maintenance_test_requires_manual_mode_and_safety_acknowledgement() -> None:
    session = parse_bookarm_test_start(
        {
            "sessionId": "MNT-001",
            "workflowMode": "takeout",
            "stepMode": "manual",
            "acknowledgeSafety": True,
        }
    )
    assert session.session_id == "MNT-001"
    assert session.workflow_mode == "takeout"
    assert parse_bookarm_test_session_id({"sessionId": "MNT-001"}) == "MNT-001"

    with pytest.raises(ValueError, match="acknowledgeSafety"):
        parse_bookarm_test_start(
            {"sessionId": "MNT-001", "workflowMode": "takeout", "stepMode": "manual"}
        )
    with pytest.raises(ValueError, match="manual step mode"):
        parse_bookarm_test_start(
            {
                "sessionId": "MNT-001",
                "workflowMode": "takeout",
                "stepMode": "auto",
                "acknowledgeSafety": True,
            }
        )
