from datetime import UTC, datetime, timedelta
import json
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped

from robot_decision.auto_localization_node import AutoLocalizationNode


def test_read_saved_pose_requires_current_map_and_fresh_timestamp(tmp_path) -> None:
    node = AutoLocalizationNode.__new__(AutoLocalizationNode)
    node.state_file = tmp_path / "pose.json"
    node.active_map_id = "MAP-01"
    node.max_saved_pose_age_seconds = 60.0
    node.state_file.write_text(
        json.dumps(
            {
                "mapId": "MAP-01",
                "x": 1.2,
                "y": -0.3,
                "yaw": 0.5,
                "savedAt": datetime.now(UTC).isoformat(),
            }
        ),
        encoding="utf-8",
    )
    assert node._read_saved_pose() == (1.2, -0.3, 0.0, 0.5)

    node.state_file.write_text(
        json.dumps(
            {
                "mapId": "MAP-01",
                "x": 1.2,
                "y": -0.3,
                "yaw": 0.5,
                "savedAt": (datetime.now(UTC) - timedelta(seconds=61)).isoformat(),
            }
        ),
        encoding="utf-8",
    )
    assert node._read_saved_pose() is None


def test_mark_localized_saves_a_confirmed_manual_pose() -> None:
    node = AutoLocalizationNode.__new__(AutoLocalizationNode)
    saved: list[tuple[float, float, float]] = []
    statuses: list[str] = []
    node._save_pose = lambda x, y, yaw: saved.append((x, y, yaw))
    node._set_status = statuses.append
    node._publish_initial_pose = lambda *_args: (_ for _ in ()).throw(AssertionError("must not republish"))

    node._mark_localized(1.2, -0.3, 0.5)

    assert saved == [(1.2, -0.3, 0.5)]
    assert statuses == ["localized"]


def test_later_stable_amcl_pose_recovers_after_manual_confirmation_prompt() -> None:
    node = AutoLocalizationNode.__new__(AutoLocalizationNode)
    node._status = "manual_required"
    node.max_position_variance = 0.5
    node.max_yaw_variance = 0.35
    node.min_stable_samples = 1
    node._stable_samples = 0
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    localized: list[tuple[float, float, float]] = []
    node._mark_localized = lambda x, y, yaw, **_kwargs: localized.append((x, y, yaw))

    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.position.x = 1.2
    message.pose.pose.position.y = -0.3
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.1
    message.pose.covariance[7] = 0.1
    message.pose.covariance[35] = 0.1

    node._handle_amcl_pose(message)

    assert localized == [(1.2, -0.3, 0.0)]


def test_localized_node_becomes_manual_required_when_amcl_covariance_is_unreliable() -> None:
    node = AutoLocalizationNode.__new__(AutoLocalizationNode)
    node._status = "localized"
    node.max_position_variance = 0.5
    node.max_yaw_variance = 0.35
    node._stable_samples = 3
    node._last_stable_pose_at = 1.0
    node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)
    statuses: list[str] = []
    node._set_status = statuses.append

    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.6
    message.pose.covariance[7] = 0.1
    message.pose.covariance[35] = 0.1

    node._handle_amcl_pose(message)

    assert statuses == ["manual_required"]
