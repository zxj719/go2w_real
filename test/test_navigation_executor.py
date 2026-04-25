import asyncio
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / "src/go2w_real"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

NAV_EXECUTOR_PATH = REPO_ROOT / "src/go2w_real/scripts/navigation_executor.py"


def _load_navigation_executor_module():
    spec = importlib.util.spec_from_file_location(
        "go2w_navigation_executor_script",
        NAV_EXECUTOR_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


nav_exec = _load_navigation_executor_module()


class FakeBridge:
    def __init__(self, has_path):
        self.has_path = has_path
        self.planner_checks = 0
        self.started_tasks = []

    def planner_has_path(self, waypoint):
        self.planner_checks += 1
        self.last_waypoint = waypoint
        return self.has_path

    def start_navigation(self, task):
        self.started_tasks.append(task.request_id)
        return True


def test_parse_cli_args_uses_more_tolerant_default_abort_retry_budget():
    config, ros_args = nav_exec._parse_cli_args([])

    assert config["abort_replan_retries"] == 3
    assert config["ignore_waypoint_yaw"] is True
    assert nav_exec.DEFAULT_ABORT_REPLAN_RETRIES == 3
    assert ros_args == []


def test_parse_cli_args_can_use_waypoint_yaw():
    config, ros_args = nav_exec._parse_cli_args(["--use-waypoint-yaw"])

    assert config["ignore_waypoint_yaw"] is False
    assert ros_args == []


def test_parse_cli_args_loads_executor_defaults_from_headless_config(tmp_path):
    config_path = tmp_path / "navigation_headless.yaml"
    config_path.write_text(
        """
default_profile: zt_0
profiles:
  zt_0:
    waypoint_file: /tmp/zt_0.yaml
    slam_map_file: /tmp/zt_0
executor:
  server_uri: ws://10.0.0.2:8100/ws/navigation/executor
  server_timeout: 7.0
  heartbeat_interval: 9.0
  reconnect_delay: 11.0
  nav_event_timeout: 13.0
  show_logs: false
""".strip(),
        encoding="utf-8",
    )

    config, ros_args = nav_exec._parse_cli_args(["--config", str(config_path)])

    assert config["server_uri"] == "ws://10.0.0.2:8100/ws/navigation/executor"
    assert config["server_timeout"] == 7.0
    assert config["heartbeat_interval"] == 9.0
    assert config["reconnect_delay"] == 11.0
    assert config["nav_event_timeout"] == 13.0
    assert ros_args == []


def test_parse_cli_args_explicit_flags_override_headless_config(tmp_path):
    config_path = tmp_path / "navigation_headless.yaml"
    config_path.write_text(
        """
default_profile: zt_0
profiles:
  zt_0:
    waypoint_file: /tmp/zt_0.yaml
    slam_map_file: /tmp/zt_0
executor:
  server_uri: ws://10.0.0.2:8100/ws/navigation/executor
""".strip(),
        encoding="utf-8",
    )

    config, _ = nav_exec._parse_cli_args(
        [
            "--config",
            str(config_path),
            "--server-uri",
            "ws://127.0.0.1:8100/ws/navigation/executor",
        ]
    )

    assert config["server_uri"] == "ws://127.0.0.1:8100/ws/navigation/executor"


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

    nav_exec._apply_waypoint_to_pose_stamped(pose_stamped, waypoint, stamp)

    assert pose_stamped.header.frame_id == "map"
    assert pose_stamped.header.stamp is stamp
    assert pose_stamped.pose.position.x == 1.0
    assert pose_stamped.pose.position.y == 2.0
    assert pose_stamped.pose.position.z == 0.0
    assert pose_stamped.pose.orientation.x == 0.0
    assert pose_stamped.pose.orientation.y == 0.0
    assert pose_stamped.pose.orientation.z == 0.0
    assert pose_stamped.pose.orientation.w == 1.0


def _make_executor(bridge=None, abort_replan_retries=1):
    executor = nav_exec.NavigationExecutor(
        {
            "server_uri": "ws://test/ws/navigation/executor",
            "waypoint_file": "/tmp/waypoints.yaml",
            "action_name": "/navigate_to_pose",
            "server_timeout": 10.0,
            "heartbeat_interval": 30.0,
            "reconnect_delay": 3.0,
            "nav_event_timeout": 15.0,
            "abort_replan_retries": abort_replan_retries,
        },
        ros_args=[],
    )
    executor._bridge = bridge or FakeBridge(has_path=True)
    sent_messages = []

    async def _send_json(payload):
        sent_messages.append(payload)
        return True

    async def _run_blocking(func, *args):
        return func(*args)

    executor._send_json = _send_json
    executor._run_blocking = _run_blocking
    executor._sent_messages = sent_messages
    return executor


def _make_task(loop):
    return nav_exec.NavigationTask(
        request_id="req-1",
        sub_id=7,
        target_id="POI_001",
        waypoint={
            "frame_id": "map",
            "position": {"x": 1.0, "y": 2.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        done_future=loop.create_future(),
        token=42,
    )


def test_aborted_navigation_retries_when_planner_still_has_path():
    from action_msgs.msg import GoalStatus

    async def scenario():
        bridge = FakeBridge(has_path=True)
        executor = _make_executor(bridge=bridge, abort_replan_retries=1)
        task = _make_task(asyncio.get_running_loop())
        executor.current_task = task

        await executor._handle_bridge_event(
            {
                "kind": "error",
                "token": task.token,
                "request_id": task.request_id,
                "sub_id": task.sub_id,
                "nav_status": GoalStatus.STATUS_ABORTED,
                "error_message": "Nav2 finished with status 6 (aborted)",
            }
        )

        assert bridge.planner_checks == 1
        assert bridge.started_tasks == [task.request_id]
        assert task.nav_abort_retries_used == 1
        assert executor.current_task is task
        assert not task.done_future.done()
        assert executor.state == nav_exec.ExecutorState.STARTING
        assert executor._sent_messages == []

    asyncio.run(scenario())


def test_aborted_navigation_reports_error_when_planner_has_no_path():
    from action_msgs.msg import GoalStatus

    async def scenario():
        bridge = FakeBridge(has_path=False)
        executor = _make_executor(bridge=bridge, abort_replan_retries=1)
        task = _make_task(asyncio.get_running_loop())
        executor.current_task = task

        await executor._handle_bridge_event(
            {
                "kind": "error",
                "token": task.token,
                "request_id": task.request_id,
                "sub_id": task.sub_id,
                "nav_status": GoalStatus.STATUS_ABORTED,
                "error_message": "Nav2 finished with status 6 (aborted)",
            }
        )

        assert bridge.planner_checks == 1
        assert bridge.started_tasks == []
        assert task.done_future.done()
        assert executor.current_task is None
        assert executor.state == nav_exec.ExecutorState.ERROR
        assert executor._sent_messages == [
            {
                "event_type": "on_error",
                "request_id": task.request_id,
                "sub_id": task.sub_id,
                "error_message": (
                    "Nav2 aborted and planner could not find a path to the target"
                ),
            }
        ]

    asyncio.run(scenario())


def test_aborted_navigation_falls_back_to_error_after_retry_budget_is_exhausted():
    from action_msgs.msg import GoalStatus

    async def scenario():
        bridge = FakeBridge(has_path=True)
        executor = _make_executor(bridge=bridge, abort_replan_retries=1)
        task = _make_task(asyncio.get_running_loop())
        task.nav_abort_retries_used = 1
        executor.current_task = task

        await executor._handle_bridge_event(
            {
                "kind": "error",
                "token": task.token,
                "request_id": task.request_id,
                "sub_id": task.sub_id,
                "nav_status": GoalStatus.STATUS_ABORTED,
                "error_message": "Nav2 finished with status 6 (aborted)",
            }
        )

        assert bridge.planner_checks == 0
        assert bridge.started_tasks == []
        assert task.done_future.done()
        assert executor.current_task is None
        assert executor.state == nav_exec.ExecutorState.ERROR
        assert executor._sent_messages == [
            {
                "event_type": "on_error",
                "request_id": task.request_id,
                "sub_id": task.sub_id,
                "error_message": "Nav2 finished with status 6 (aborted)",
            }
        ]

    asyncio.run(scenario())
