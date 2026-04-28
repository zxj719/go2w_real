from pathlib import Path
import os
import stat
import subprocess
import time


REPO_ROOT = Path(__file__).resolve().parents[3]
CONFIG_CLI = REPO_ROOT / "src/go2w_real/scripts/navigation_headless_config.py"
START_SCRIPT = REPO_ROOT / "src/go2w_real/scripts/start_navigation_headless.sh"


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content)
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def test_headless_config_cli_prints_shell_assignments():
    output = subprocess.check_output(
        ["python3", str(CONFIG_CLI), "--format", "shell"],
        text=True,
    )

    assert "HEADLESS_PROFILE=zt_0" in output
    assert "HEADLESS_CLEANUP_MATCHERS=(" in output
    assert (
        "HEADLESS_SERVER_URI=ws://192.168.123.186:8100/ws/navigation/executor"
        in output
    )
    assert "HEADLESS_USE_ODOM_FUSION=1" in output
    assert "HEADLESS_RECORD_BAG=1" in output


def test_headless_start_script_help_prefers_config_and_profile():
    help_text = subprocess.check_output(
        ["bash", str(START_SCRIPT), "--help"],
        text=True,
    )

    assert "--config PATH" in help_text
    assert "--profile NAME" in help_text
    assert "--server-uri" not in help_text
    assert "--heartbeat-interval" not in help_text


def test_headless_start_script_declares_navigation_readiness_waits():
    script_text = START_SCRIPT.read_text()

    assert "GO2W_WAIT_FOR_TF_BIN" in script_text
    assert "GO2W_WAIT_FOR_SCAN_HEALTH_BIN" in script_text
    assert "GO2W_XT16_POINT_TOPIC" in script_text
    assert '"topic:=${XT16_POINT_TOPIC}"' in script_text
    assert '"topic:=/scan"' in script_text
    assert '"target_frame:=${TF_TARGET_FRAME}"' in script_text
    assert '"source_frame:=${TF_SOURCE_FRAME}"' in script_text


def test_headless_start_script_waits_for_xt16_tf_and_scan_before_executor(tmp_path):
    config_file = tmp_path / "navigation_headless.yaml"
    waypoint_file = tmp_path / "waypoints.yaml"
    map_prefix = tmp_path / "zt_0"
    nav2_params = tmp_path / "nav2_params.yaml"
    slam_params = tmp_path / "slam_params.yaml"
    ros_setup = tmp_path / "ros_setup.bash"
    install_setup = tmp_path / "install_setup.bash"
    xt16_bin = tmp_path / "fake_xt16.py"
    ros2_bin = tmp_path / "fake_ros2.py"
    executor_bin = tmp_path / "fake_executor.py"
    wait_for_tf_bin = tmp_path / "fake_wait_for_transform.py"
    wait_for_topic_bin = tmp_path / "fake_wait_for_topic_rate.py"
    events_file = tmp_path / "events.log"
    ready_file = tmp_path / "bt_navigator.ready"

    config_file.write_text(
        f"""
default_profile: demo
profiles:
  demo:
    waypoint_file: {waypoint_file}
    slam_map_file: {map_prefix}
executor:
  server_uri: ws://127.0.0.1:8100/ws/navigation/executor
  server_timeout: 1.0
  heartbeat_interval: 1.0
  reconnect_delay: 1.0
  nav_event_timeout: 1.0
  show_logs: false
  record_bag: false
launch:
  network_interface: eth0
  slam_mode: localization
  use_rviz: false
  use_odom_fusion: true
  nav2_wait_timeout_sec: 5
cleanup:
  process_matchers: []
  term_timeout_sec: 1
  kill_timeout_sec: 1
""".strip()
        + "\n"
    )
    waypoint_file.write_text("waypoints: []\n")
    (tmp_path / "zt_0.data").write_text("data")
    (tmp_path / "zt_0.posegraph").write_text("posegraph")
    nav2_params.write_text("nav2: {}\n")
    slam_params.write_text("slam: {}\n")
    ros_setup.write_text("")
    install_setup.write_text("")

    _write_executable(
        xt16_bin,
        """#!/usr/bin/env python3
import signal
import time

signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))
signal.signal(signal.SIGINT, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))

while True:
    time.sleep(0.1)
""",
    )
    _write_executable(
        ros2_bin,
        """#!/usr/bin/env python3
import os
import signal
import sys
import time

events_file = os.environ["EVENTS_FILE"]
ready_file = os.environ["READY_FILE"]
args = sys.argv[1:]

if args[:2] == ["node", "list"]:
    if os.path.exists(ready_file):
        print("/bt_navigator")
    raise SystemExit(0)

if args and args[0] == "launch":
    with open(events_file, "a", encoding="utf-8") as handle:
        handle.write("nav_launch_started\\n")
    with open(ready_file, "w", encoding="utf-8") as handle:
        handle.write("ready\\n")

    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))
    signal.signal(signal.SIGINT, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))

    while True:
        time.sleep(0.1)

if args and args[0] == "bag":
    raise SystemExit(0)

raise SystemExit(0)
""",
    )
    _write_executable(
        wait_for_topic_bin,
        """#!/usr/bin/env python3
import os
import sys

with open(os.environ["EVENTS_FILE"], "a", encoding="utf-8") as handle:
    handle.write(f"topic_wait:{' '.join(sys.argv[1:])}\\n")
""",
    )
    _write_executable(
        wait_for_tf_bin,
        """#!/usr/bin/env python3
import os
import sys

with open(os.environ["EVENTS_FILE"], "a", encoding="utf-8") as handle:
    handle.write(f"tf_wait:{' '.join(sys.argv[1:])}\\n")
""",
    )
    _write_executable(
        executor_bin,
        """#!/usr/bin/env python3
import os

with open(os.environ["EVENTS_FILE"], "a", encoding="utf-8") as handle:
    handle.write("executor_started\\n")
""",
    )

    env = os.environ.copy()
    env["EVENTS_FILE"] = str(events_file)
    env["READY_FILE"] = str(ready_file)
    env["GO2W_ROS2_BIN"] = str(ros2_bin)
    env["GO2W_XT16_BIN"] = str(xt16_bin)
    env["GO2W_EXECUTOR_BIN"] = str(executor_bin)
    env["GO2W_WAIT_FOR_TF_BIN"] = str(wait_for_tf_bin)
    env["GO2W_WAIT_FOR_SCAN_HEALTH_BIN"] = str(wait_for_topic_bin)
    env["GO2W_ROS_SETUP_FILE"] = str(ros_setup)
    env["GO2W_UNITREE_SETUP_FILE"] = ""
    env["GO2W_NAV2_PARAMS_FILE"] = str(nav2_params)
    env["GO2W_SLAM_LOCALIZATION_PARAMS_FILE"] = str(slam_params)
    env["GO2W_INSTALL_SETUP_FILE"] = str(install_setup)

    proc = subprocess.run(
        ["bash", str(START_SCRIPT), "--config", str(config_file)],
        env=env,
        text=True,
        capture_output=True,
        timeout=20,
    )

    assert proc.returncode == 1
    event_lines = events_file.read_text().splitlines()
    xt16_wait = next(
        index
        for index, line in enumerate(event_lines)
        if line.startswith("topic_wait:") and "topic:=/unitree/slam_lidar/points" in line
    )
    nav_launch = event_lines.index("nav_launch_started")
    tf_wait = next(
        index
        for index, line in enumerate(event_lines)
        if line.startswith("tf_wait:")
        and "target_frame:=map" in line
        and "source_frame:=base" in line
    )
    scan_wait = next(
        index
        for index, line in enumerate(event_lines)
        if line.startswith("topic_wait:") and "topic:=/scan" in line
    )
    executor_started = event_lines.index("executor_started")

    assert xt16_wait < nav_launch
    assert nav_launch < tf_wait < scan_wait < executor_started


def test_headless_start_script_kills_launch_process_group(tmp_path):
    config_file = tmp_path / "navigation_headless.yaml"
    waypoint_file = tmp_path / "waypoints.yaml"
    map_prefix = tmp_path / "zt_0"
    nav2_params = tmp_path / "nav2_params.yaml"
    slam_params = tmp_path / "slam_params.yaml"
    ros_setup = tmp_path / "ros_setup.bash"
    install_setup = tmp_path / "install_setup.bash"
    xt16_bin = tmp_path / "fake_xt16.py"
    ros2_bin = tmp_path / "fake_ros2.py"
    executor_bin = tmp_path / "fake_executor.py"
    launch_child_bin = tmp_path / "fake_launch_child.py"
    events_file = tmp_path / "events.log"
    ready_file = tmp_path / "bt_navigator.ready"
    child_pid_file = tmp_path / "launch_child.pid"

    config_file.write_text(
        f"""
default_profile: demo
profiles:
  demo:
    waypoint_file: {waypoint_file}
    slam_map_file: {map_prefix}
executor:
  server_uri: ws://127.0.0.1:8100/ws/navigation/executor
  server_timeout: 1.0
  heartbeat_interval: 1.0
  reconnect_delay: 1.0
  nav_event_timeout: 1.0
  show_logs: false
  record_bag: false
launch:
  network_interface: eth0
  slam_mode: localization
  use_rviz: false
  use_odom_fusion: false
  nav2_wait_timeout_sec: 5
cleanup:
  process_matchers:
    - fake_launch_child.py
  term_timeout_sec: 1
  kill_timeout_sec: 1
""".strip()
        + "\n"
    )
    waypoint_file.write_text("waypoints: []\n")
    (tmp_path / "zt_0.data").write_text("data")
    (tmp_path / "zt_0.posegraph").write_text("posegraph")
    nav2_params.write_text("nav2: {}\n")
    slam_params.write_text("slam: {}\n")
    ros_setup.write_text("")
    install_setup.write_text("")

    _write_executable(
        xt16_bin,
        """#!/usr/bin/env python3
import signal
import time

signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))
signal.signal(signal.SIGINT, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))

while True:
    time.sleep(0.1)
""",
    )
    _write_executable(
        launch_child_bin,
        """#!/usr/bin/env python3
import os
import signal
import time

events_file = os.environ["EVENTS_FILE"]

def _ignore(signum, frame):
    signal_name = signal.Signals(signum).name
    with open(events_file, "a", encoding="utf-8") as handle:
        handle.write(f"launch_child_ignored:{signal_name}\\n")

signal.signal(signal.SIGTERM, _ignore)
signal.signal(signal.SIGINT, _ignore)

with open(events_file, "a", encoding="utf-8") as handle:
    handle.write("launch_child_started\\n")

while True:
    time.sleep(0.1)
""",
    )
    _write_executable(
        ros2_bin,
        """#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time

events_file = os.environ["EVENTS_FILE"]
ready_file = os.environ["READY_FILE"]
child_pid_file = os.environ["LAUNCH_CHILD_PID_FILE"]
launch_child_bin = os.environ["LAUNCH_CHILD_BIN"]
args = sys.argv[1:]

if args[:2] == ["node", "list"]:
    if os.path.exists(ready_file):
        print("/bt_navigator")
    raise SystemExit(0)

if args and args[0] == "launch":
    child = subprocess.Popen([launch_child_bin], env=os.environ.copy())
    with open(child_pid_file, "w", encoding="utf-8") as handle:
        handle.write(f"{child.pid}\\n")
    with open(events_file, "a", encoding="utf-8") as handle:
        handle.write("nav_launch_started\\n")
    with open(ready_file, "w", encoding="utf-8") as handle:
        handle.write("ready\\n")

    def _handle_stop(signum, frame):
        signal_name = signal.Signals(signum).name
        with open(events_file, "a", encoding="utf-8") as handle:
            handle.write(f"nav_launch_stopped:{signal_name}\\n")
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _handle_stop)
    signal.signal(signal.SIGINT, _handle_stop)

    while True:
        time.sleep(0.1)

if args and args[0] == "bag":
    raise SystemExit(0)

raise SystemExit(0)
""",
    )
    _write_executable(
        executor_bin,
        """#!/usr/bin/env python3
import os

with open(os.environ["EVENTS_FILE"], "a", encoding="utf-8") as handle:
    handle.write("executor_started\\n")
""",
    )

    env = os.environ.copy()
    env["EVENTS_FILE"] = str(events_file)
    env["READY_FILE"] = str(ready_file)
    env["LAUNCH_CHILD_PID_FILE"] = str(child_pid_file)
    env["LAUNCH_CHILD_BIN"] = str(launch_child_bin)
    env["GO2W_ROS2_BIN"] = str(ros2_bin)
    env["GO2W_XT16_BIN"] = str(xt16_bin)
    env["GO2W_EXECUTOR_BIN"] = str(executor_bin)
    env["GO2W_ROS_SETUP_FILE"] = str(ros_setup)
    env["GO2W_UNITREE_SETUP_FILE"] = ""
    env["GO2W_NAV2_PARAMS_FILE"] = str(nav2_params)
    env["GO2W_SLAM_LOCALIZATION_PARAMS_FILE"] = str(slam_params)
    env["GO2W_INSTALL_SETUP_FILE"] = str(install_setup)
    env["GO2W_SKIP_TF_READY_WAIT"] = "1"
    env["GO2W_SKIP_SCAN_READY_WAIT"] = "1"
    env["GO2W_SKIP_XT16_READY_WAIT"] = "1"

    proc = subprocess.run(
        ["bash", str(START_SCRIPT), "--config", str(config_file)],
        env=env,
        text=True,
        capture_output=True,
        timeout=20,
    )

    assert proc.returncode == 1

    child_pid = int(child_pid_file.read_text().strip())
    deadline = time.time() + 3.0
    while time.time() < deadline and _pid_exists(child_pid):
        time.sleep(0.1)

    event_lines = events_file.read_text().splitlines()

    assert "nav_launch_started" in event_lines
    assert "executor_started" in event_lines
    assert any(line.startswith("nav_launch_stopped:SIGINT") for line in event_lines)
    assert "launch_child_started" in event_lines
    assert not _pid_exists(child_pid)
