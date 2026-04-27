import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


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
    navigator._last_feedback_pose = None
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


def test_parse_cli_args_supports_batch_and_success_doublecheck_options():
    config, ros_args = nav_waypoint._parse_cli_args(
        [
            "--all-waypoints",
            "--waypoint-sequence",
            "POI_019,POI_020",
            "--continue-on-failure",
            "--batch-inter-goal-delay",
            "0.8",
            "--success-check-distance",
            "0.9",
            "--success-check-timeout",
            "1.5",
            "--map-frame",
            "world",
            "--base-frame",
            "base_link",
        ]
    )

    assert config["all_waypoints"] is True
    assert config["waypoint_sequence"] == "POI_019,POI_020"
    assert config["continue_on_failure"] is True
    assert config["batch_inter_goal_delay"] == 0.8
    assert config["success_check_distance"] == 0.9
    assert config["success_check_timeout"] == 1.5
    assert config["map_frame"] == "world"
    assert config["base_frame"] == "base_link"
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


def test_doublecheck_failure_retries_when_planner_still_has_path(monkeypatch):
    navigator = _make_navigator(abort_replan_retries=2)
    waypoint = {"name": "wp_04"}
    statuses = iter(["DOUBLECHECK_FAILED", "SUCCEEDED"])
    planner_checks = []
    spin_delays = []

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
    monkeypatch.setattr(
        nav_waypoint,
        "_spin_delay",
        lambda node, delay_sec: spin_delays.append(delay_sec),
    )

    result = navigator.navigate_to(waypoint)

    assert result is True
    assert planner_checks == [waypoint]
    assert spin_delays
    assert any(
        "reported success before the robot reached the target" in line
        for line in navigator.node.get_logger().warnings
    )


def test_doublecheck_failure_stops_when_planner_has_no_path(monkeypatch):
    navigator = _make_navigator(abort_replan_retries=2)
    waypoint = {"name": "wp_05"}

    monkeypatch.setattr(navigator, "_navigate_once", lambda target: "DOUBLECHECK_FAILED")
    monkeypatch.setattr(navigator, "planner_has_path", lambda target: False)
    monkeypatch.setattr(nav_waypoint, "_spin_delay", lambda node, delay_sec: None)

    result = navigator.navigate_to(waypoint)

    assert result is False
    assert any(
        "failed the final TF doublecheck" in line
        for line in navigator.node.get_logger().errors
    )


def test_resolve_waypoint_batch_can_expand_all_waypoints():
    waypoints = [
        {"index": 1, "id": "POI_019", "name": "wp_07"},
        {"index": 2, "id": "POI_020", "name": "wp_08"},
    ]

    selected = nav_waypoint._resolve_waypoint_batch(
        {
            "all_waypoints": True,
            "waypoint_sequence": None,
        },
        waypoints,
    )

    assert selected == waypoints


def test_resolve_waypoint_batch_can_expand_sequence():
    waypoints = [
        {"index": 1, "id": "POI_019", "name": "wp_07"},
        {"index": 2, "id": "POI_020", "name": "wp_08"},
    ]

    selected = nav_waypoint._resolve_waypoint_batch(
        {
            "all_waypoints": False,
            "waypoint_sequence": "POI_020,1",
        },
        waypoints,
    )

    assert [item["id"] for item in selected] == ["POI_020", "POI_019"]


def test_xy_distance_to_waypoint_uses_planar_distance():
    waypoint = {"position": {"x": 3.0, "y": 4.0}}

    distance = nav_waypoint._xy_distance_to_waypoint(
        waypoint,
        {"x": 0.0, "y": 0.0},
    )

    assert distance == 5.0


def test_doublecheck_failed_status_is_treated_as_failure(monkeypatch):
    navigator = _make_navigator(abort_replan_retries=2)
    waypoint = {"name": "wp_08"}

    monkeypatch.setattr(
        navigator,
        "_navigate_once",
        lambda target: "DOUBLECHECK_FAILED",
    )
    monkeypatch.setattr(navigator, "planner_has_path", lambda target: False)
    monkeypatch.setattr(nav_waypoint, "_spin_delay", lambda node, delay_sec: None)

    result = navigator.navigate_to(waypoint)

    assert result is False
    assert any(
        "failed the final TF doublecheck" in line
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


def test_doublecheck_waits_for_transform_before_lookup(monkeypatch):
    navigator = _make_navigator()
    navigator.map_frame = "map"
    navigator.base_frame = "base"
    navigator.success_check_distance = 0.75
    navigator.success_check_timeout = 0.3

    events = []
    state = {"ready": False}

    class FakeBuffer:
        def can_transform(self, target_frame, source_frame, time, timeout=None):
            events.append(("can", target_frame, source_frame))
            return state["ready"]

        def lookup_transform(self, target_frame, source_frame, time, timeout=None):
            events.append(("lookup", target_frame, source_frame))
            if not state["ready"]:
                raise RuntimeError("transform not ready")
            return SimpleNamespace(
                transform=SimpleNamespace(
                    translation=SimpleNamespace(x=0.0, y=0.0, z=0.0)
                )
            )

    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.spin_once = lambda node, timeout_sec=None: state.__setitem__("ready", True)
    fake_rclpy.time = SimpleNamespace(Time=lambda: None)
    fake_rclpy_duration = ModuleType("rclpy.duration")
    fake_rclpy_duration.Duration = lambda seconds=0.0: seconds

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.duration", fake_rclpy_duration)

    navigator._tf_buffer = FakeBuffer()
    waypoint = {"name": "wp_00", "position": {"x": 0.0, "y": 0.0}}

    assert navigator._doublecheck_waypoint_reached(waypoint) is True
    event_names = [name for name, _, _ in events]
    assert event_names[0] == "can"
    assert "lookup" in event_names
    assert all(name == "can" for name in event_names[:event_names.index("lookup")])


def test_doublecheck_can_pass_with_nav2_feedback_pose_when_tf_is_unavailable(monkeypatch):
    navigator = _make_navigator()
    navigator.map_frame = "map"
    navigator.base_frame = "base"
    navigator.success_check_distance = 0.75
    navigator.success_check_timeout = 0.3
    navigator._last_feedback_pose = {
        "frame_id": "map",
        "x": 0.25,
        "y": 0.20,
        "z": 0.0,
    }

    class FakeBuffer:
        def can_transform(self, target_frame, source_frame, time, timeout=None):
            return False

        def lookup_transform(self, target_frame, source_frame, time, timeout=None):
            raise AssertionError("lookup_transform should not be required")

    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.spin_once = lambda node, timeout_sec=None: None
    fake_rclpy.time = SimpleNamespace(Time=lambda: None)
    fake_rclpy_duration = ModuleType("rclpy.duration")
    fake_rclpy_duration.Duration = lambda seconds=0.0: seconds

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.duration", fake_rclpy_duration)

    navigator._tf_buffer = FakeBuffer()
    waypoint = {"name": "wp_00", "position": {"x": 0.0, "y": 0.0}}

    assert navigator._doublecheck_waypoint_reached(waypoint) is True
    assert any(
        "using Nav2 feedback pose" in line
        for line in navigator.node.get_logger().infos
    )


def test_doublecheck_uses_latest_feedback_pose_during_wait(monkeypatch):
    navigator = _make_navigator()
    navigator.map_frame = "map"
    navigator.base_frame = "base"
    navigator.success_check_distance = 0.75
    navigator.success_check_timeout = 0.3
    navigator._last_feedback_pose = {
        "frame_id": "map",
        "x": 2.0,
        "y": 0.0,
        "z": 0.0,
    }

    class FakeBuffer:
        def can_transform(self, target_frame, source_frame, time, timeout=None):
            return False

        def lookup_transform(self, target_frame, source_frame, time, timeout=None):
            raise AssertionError("lookup_transform should not be required")

    state = {"updated": False}
    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.ok = lambda: True

    def _spin_once(node, timeout_sec=None):
        if not state["updated"]:
            navigator._last_feedback_pose = {
                "frame_id": "map",
                "x": 0.3,
                "y": 0.2,
                "z": 0.0,
            }
            state["updated"] = True

    fake_rclpy.spin_once = _spin_once
    fake_rclpy.time = SimpleNamespace(Time=lambda: None)
    fake_rclpy_duration = ModuleType("rclpy.duration")
    fake_rclpy_duration.Duration = lambda seconds=0.0: seconds

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.duration", fake_rclpy_duration)

    navigator._tf_buffer = FakeBuffer()
    waypoint = {"name": "wp_01", "position": {"x": 0.0, "y": 0.0}}

    assert navigator._doublecheck_waypoint_reached(waypoint) is True


def test_spin_delay_spins_until_deadline(monkeypatch):
    state = {"now": 10.0}
    calls = []

    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.ok = lambda: True

    def _spin_once(node, timeout_sec=None):
        calls.append(timeout_sec)
        state["now"] += float(timeout_sec or 0.0)

    fake_rclpy.spin_once = _spin_once
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setattr(nav_waypoint.time, "monotonic", lambda: state["now"])

    nav_waypoint._spin_delay(object(), 0.25)

    assert calls
    assert max(calls) <= 0.1
    assert sum(calls) >= 0.25
