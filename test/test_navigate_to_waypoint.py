import importlib.util
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path("/home/unitree/ros_ws")
NAVIGATE_TO_WAYPOINT_PATH = (
    REPO_ROOT / "src/go2w_real/scripts/navigate_to_waypoint.py"
)


def _load_navigate_to_waypoint_module():
    spec = importlib.util.spec_from_file_location(
        "go2w_navigate_to_waypoint_script",
        NAVIGATE_TO_WAYPOINT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


nav_waypoint = _load_navigate_to_waypoint_module()


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        self.infos.append(str(message))

    def warn(self, message):
        self.warnings.append(str(message))

    def error(self, message):
        self.errors.append(str(message))


class FakeNode:
    def __init__(self):
        self._logger = FakeLogger()

    def get_logger(self):
        return self._logger


def _make_navigator(abort_replan_retries=1):
    navigator = nav_waypoint.WaypointNavigator.__new__(nav_waypoint.WaypointNavigator)
    navigator.node = FakeNode()
    navigator.server_timeout = 10.0
    navigator.abort_replan_retries = abort_replan_retries
    navigator.near_goal_distance = nav_waypoint.DEFAULT_NEAR_GOAL_DISTANCE
    navigator.ignore_waypoint_yaw = True
    navigator._active_goal_handle = None
    navigator._last_feedback_log_time = 0.0
    navigator._near_goal_seen = False
    navigator._near_goal_cancel_future = None
    return navigator


def test_parse_cli_args_supports_abort_replan_retries():
    config, ros_args = nav_waypoint._parse_cli_args(
        ["--abort-replan-retries", "3", "--ros-args", "-r", "__node:=foo"]
    )

    assert config["abort_replan_retries"] == 3
    assert config["ignore_waypoint_yaw"] is True
    assert ros_args == ["--ros-args", "-r", "__node:=foo"]


def test_parse_cli_args_supports_near_goal_distance():
    config, ros_args = nav_waypoint._parse_cli_args(
        ["--near-goal-distance", "0.55"]
    )

    assert config["near_goal_distance"] == 0.55
    assert ros_args == []


def test_parse_cli_args_can_use_waypoint_yaw():
    config, ros_args = nav_waypoint._parse_cli_args(["--use-waypoint-yaw"])

    assert config["ignore_waypoint_yaw"] is False
    assert ros_args == []


def test_apply_waypoint_to_pose_stamped_ignores_yaw_by_default():
    pose_stamped = SimpleNamespace(
        header=SimpleNamespace(frame_id=None, stamp=None),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=None, y=None, z=None),
            orientation=SimpleNamespace(x=None, y=None, z=None, w=None),
        ),
    )
    waypoint = {
        "frame_id": "map",
        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
        "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.4},
    }
    stamp = object()
    fake_node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: stamp)
        )
    )

    nav_waypoint._apply_waypoint_to_pose_stamped(
        fake_node,
        pose_stamped,
        waypoint,
    )

    assert pose_stamped.header.frame_id == "map"
    assert pose_stamped.header.stamp is stamp
    assert pose_stamped.pose.position.x == 1.0
    assert pose_stamped.pose.position.y == 2.0
    assert pose_stamped.pose.position.z == 0.0
    assert pose_stamped.pose.orientation.x == 0.0
    assert pose_stamped.pose.orientation.y == 0.0
    assert pose_stamped.pose.orientation.z == 0.0
    assert pose_stamped.pose.orientation.w == 1.0


def test_aborted_navigation_retries_when_planner_still_has_path(monkeypatch):
    navigator = _make_navigator(abort_replan_retries=2)
    waypoint = {"name": "wp_01"}
    statuses = iter(["ABORTED", "SUCCEEDED"])
    planner_checks = []

    monkeypatch.setattr(
        navigator,
        "_navigate_once",
        lambda target: next(statuses),
    )
    monkeypatch.setattr(
        navigator,
        "planner_has_path",
        lambda target: planner_checks.append(target) or True,
    )

    result = navigator.navigate_to(waypoint)

    assert result is True
    assert planner_checks == [waypoint]
    assert any("retrying current goal" in line for line in navigator.node.get_logger().infos)


def test_aborted_navigation_stops_when_planner_has_no_path(monkeypatch):
    navigator = _make_navigator(abort_replan_retries=2)
    waypoint = {"name": "wp_02"}

    monkeypatch.setattr(navigator, "_navigate_once", lambda target: "ABORTED")
    monkeypatch.setattr(navigator, "planner_has_path", lambda target: False)

    result = navigator.navigate_to(waypoint)

    assert result is False
    assert any(
        "planner could not find a path" in line
        for line in navigator.node.get_logger().errors
    )


def test_aborted_navigation_stops_after_retry_budget_exhausted(monkeypatch):
    navigator = _make_navigator(abort_replan_retries=1)
    waypoint = {"name": "wp_03"}
    planner_checks = []

    monkeypatch.setattr(navigator, "_navigate_once", lambda target: "ABORTED")
    monkeypatch.setattr(
        navigator,
        "planner_has_path",
        lambda target: planner_checks.append(target) or True,
    )

    result = navigator.navigate_to(waypoint)

    assert result is False
    assert planner_checks == [waypoint]
    assert any(
        "retry budget exhausted" in line
        for line in navigator.node.get_logger().errors
    )


def test_near_goal_aborted_status_is_treated_as_success():
    navigator = _make_navigator()
    navigator._near_goal_seen = True

    assert navigator._should_accept_near_goal_status("ABORTED") is True
    assert navigator._should_accept_near_goal_status("CANCELED") is True


def test_near_goal_status_not_accepted_when_waypoint_yaw_is_required():
    navigator = _make_navigator()
    navigator._near_goal_seen = True
    navigator.ignore_waypoint_yaw = False

    assert navigator._should_accept_near_goal_status("ABORTED") is False


def test_feedback_requests_cancel_when_within_near_goal_distance():
    navigator = _make_navigator()
    cancel_calls = []

    class FakeGoalHandle:
        def cancel_goal_async(self):
            cancel_calls.append(True)
            return "cancel-future"

    navigator._active_goal_handle = FakeGoalHandle()

    navigator._maybe_request_near_goal_cancel(0.35)

    assert navigator._near_goal_seen is True
    assert navigator._near_goal_cancel_future == "cancel-future"
    assert cancel_calls == [True]
