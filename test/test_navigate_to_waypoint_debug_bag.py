from pathlib import Path
import os
import pty
import select
import errno
import stat
import subprocess
import time


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / "src/go2w_real"
WRAPPER_SCRIPT = (
    PACKAGE_ROOT / "scripts/navigate_to_waypoint_with_debug_bag.sh"
)
TOPICS_HELPER = PACKAGE_ROOT / "scripts/navigation_debug_bag_topics.sh"
START_HEADLESS_SCRIPT = PACKAGE_ROOT / "scripts/start_navigation_headless.sh"


def _read_pty_text(fd: int) -> str:
    try:
        return os.read(fd, 4096).decode(errors="replace")
    except OSError as exc:
        if exc.errno == errno.EIO:
            return ""
        raise


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content)
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def test_debug_bag_wrapper_help_mentions_recording_lifecycle():
    help_text = subprocess.check_output(
        ["bash", str(WRAPPER_SCRIPT), "--help"],
        text=True,
    )

    assert "Starts rosbag recording immediately before navigate_to_waypoint.py" in help_text
    assert "Stops recording automatically when navigation exits" in help_text
    assert "start xt16_driver" in help_text
    assert "launch slam_rf2o localization" in help_text


def test_debug_bag_topics_are_shared_between_headless_and_wrapper_scripts():
    topics_text = TOPICS_HELPER.read_text()
    wrapper_text = WRAPPER_SCRIPT.read_text()
    start_headless_text = START_HEADLESS_SCRIPT.read_text()

    assert "GO2W_NAVIGATION_DEBUG_BAG_TOPICS=(" in topics_text
    assert "navigation_debug_bag_topics.sh" in wrapper_text
    assert "navigation_debug_bag_topics.sh" in start_headless_text
    assert '"${GO2W_NAVIGATION_DEBUG_BAG_TOPICS[@]}"' in wrapper_text
    assert '"${GO2W_NAVIGATION_DEBUG_BAG_TOPICS[@]}"' in start_headless_text


def test_debug_bag_wrapper_records_around_navigation_process(tmp_path):
    events_file = tmp_path / "events.log"
    ros_setup = tmp_path / "ros_setup.bash"
    install_setup = tmp_path / "install_setup.bash"
    navigate_bin = tmp_path / "fake_navigate_to_waypoint.sh"
    ros2_bin = tmp_path / "fake_ros2.sh"
    bag_output_dir = tmp_path / "bag_output"

    ros_setup.write_text("")
    install_setup.write_text("")
    _write_executable(
        navigate_bin,
        """#!/usr/bin/env bash
set -euo pipefail
printf 'navigate_started:%s\\n' "$*" >> "${EVENTS_FILE}"
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
args = sys.argv[1:]
with open(events_file, "a", encoding="utf-8") as handle:
    handle.write(f"ros2_invoked:{' '.join(args)}\\n")

output_dir = ""
for index, arg in enumerate(args[:-1]):
    if arg == "-o":
        output_dir = args[index + 1]
        break

if output_dir:
    os.makedirs(output_dir, exist_ok=True)

def _handle_stop(signum, frame):
    with open(events_file, "a", encoding="utf-8") as handle:
        handle.write("rosbag_stopped\\n")
    raise SystemExit(0)

signal.signal(signal.SIGINT, _handle_stop)
signal.signal(signal.SIGTERM, _handle_stop)

with open(events_file, "a", encoding="utf-8") as handle:
    handle.write("rosbag_started\\n")

while True:
    time.sleep(0.1)
""",
    )

    env = os.environ.copy()
    env["EVENTS_FILE"] = str(events_file)
    env["GO2W_ROS_SETUP_FILE"] = str(ros_setup)
    env["GO2W_INSTALL_SETUP_FILE"] = str(install_setup)
    env["GO2W_UNITREE_SETUP_FILE"] = ""
    env["GO2W_NAVIGATE_BIN"] = str(navigate_bin)
    env["GO2W_ROS2_BIN"] = str(ros2_bin)
    env["GO2W_DEBUG_BAG_OUTPUT_DIR"] = str(bag_output_dir)
    env["GO2W_SKIP_BRINGUP"] = "1"

    subprocess.check_call(
        [
            "bash",
            str(WRAPPER_SCRIPT),
            "--waypoint",
            "wp_07",
        ],
        env=env,
    )

    event_lines = events_file.read_text().splitlines()

    assert "rosbag_started" in event_lines
    assert "rosbag_stopped" in event_lines
    assert any(line == "navigate_started:--waypoint wp_07" for line in event_lines)
    assert event_lines.index("rosbag_started") < event_lines.index(
        "navigate_started:--waypoint wp_07"
    )
    assert event_lines.index("navigate_started:--waypoint wp_07") < event_lines.index(
        "rosbag_stopped"
    )
    assert any(
        line.startswith("ros2_invoked:bag record -o ")
        and "--include-hidden-topics" in line
        for line in event_lines
    )
    assert (bag_output_dir / "bag").is_dir()
    assert (bag_output_dir / "topics.txt").is_file()


def test_debug_bag_wrapper_preserves_interactive_stdin_for_navigation(tmp_path):
    events_file = tmp_path / "events.log"
    ros_setup = tmp_path / "ros_setup.bash"
    install_setup = tmp_path / "install_setup.bash"
    navigate_bin = tmp_path / "fake_interactive_navigate.py"
    ros2_bin = tmp_path / "fake_ros2.py"

    ros_setup.write_text("")
    install_setup.write_text("")
    _write_executable(
        navigate_bin,
        """#!/usr/bin/env python3
import sys

print(f"stdin_isatty={sys.stdin.isatty()}", flush=True)
try:
    choice = input("choose: ")
except EOFError:
    print("eof", flush=True)
    raise SystemExit(23)

print(f"choice={choice}", flush=True)
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

def _handle_stop(signum, frame):
    raise SystemExit(0)

signal.signal(signal.SIGINT, _handle_stop)
signal.signal(signal.SIGTERM, _handle_stop)

with open(events_file, "a", encoding="utf-8") as handle:
    handle.write(f"ros2_invoked:{' '.join(sys.argv[1:])}\\n")

while True:
    time.sleep(0.1)
""",
    )

    env = os.environ.copy()
    env["EVENTS_FILE"] = str(events_file)
    env["GO2W_ROS_SETUP_FILE"] = str(ros_setup)
    env["GO2W_INSTALL_SETUP_FILE"] = str(install_setup)
    env["GO2W_UNITREE_SETUP_FILE"] = ""
    env["GO2W_NAVIGATE_BIN"] = str(navigate_bin)
    env["GO2W_ROS2_BIN"] = str(ros2_bin)
    env["GO2W_SKIP_BRINGUP"] = "1"

    master_fd, slave_fd = pty.openpty()
    try:
        proc = subprocess.Popen(
            ["bash", str(WRAPPER_SCRIPT)],
            env=env,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            close_fds=True,
        )
    finally:
        os.close(slave_fd)

    output = ""
    sent_input = False
    deadline = time.time() + 10.0

    while time.time() < deadline:
        if proc.poll() is not None:
            break

        ready, _, _ = select.select([master_fd], [], [], 0.2)
        if ready:
            chunk = _read_pty_text(master_fd)
            if not chunk:
                break
            output += chunk
            if not sent_input and "choose:" in output:
                os.write(master_fd, b"wp_07\n")
                sent_input = True

    while True:
        ready, _, _ = select.select([master_fd], [], [], 0.1)
        if not ready:
            break
        chunk = _read_pty_text(master_fd)
        if not chunk:
            break
        output += chunk

    return_code = proc.wait(timeout=5)
    os.close(master_fd)

    assert return_code == 0
    assert "stdin_isatty=True" in output
    assert "choice=wp_07" in output


def test_debug_bag_wrapper_can_bring_up_xt16_and_localization(tmp_path):
    events_file = tmp_path / "events.log"
    ready_file = tmp_path / "bt_navigator.ready"
    ros_setup = tmp_path / "ros_setup.bash"
    install_setup = tmp_path / "install_setup.bash"
    navigate_bin = tmp_path / "fake_navigate_to_waypoint.sh"
    ros2_bin = tmp_path / "fake_ros2.py"
    xt16_bin = tmp_path / "fake_xt16.py"
    map_prefix = tmp_path / "zt_0"
    bag_output_dir = tmp_path / "bag_output"

    ros_setup.write_text("")
    install_setup.write_text("")
    (tmp_path / "zt_0.data").write_text("data")
    (tmp_path / "zt_0.posegraph").write_text("posegraph")
    _write_executable(
        navigate_bin,
        """#!/usr/bin/env bash
set -euo pipefail
printf 'navigate_started:%s\\n' "$*" >> "${EVENTS_FILE}"
""",
    )
    _write_executable(
        xt16_bin,
        """#!/usr/bin/env python3
import os
import signal
import time

events_file = os.environ["EVENTS_FILE"]

def _handle_stop(signum, frame):
    with open(events_file, "a", encoding="utf-8") as handle:
        handle.write("xt16_stopped\\n")
    raise SystemExit(0)

signal.signal(signal.SIGTERM, _handle_stop)
signal.signal(signal.SIGINT, _handle_stop)

with open(events_file, "a", encoding="utf-8") as handle:
    handle.write("xt16_started\\n")

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
        handle.write(f"ros2_launch_invoked:{' '.join(args)}\\n")
        handle.write("nav_launch_started\\n")
    with open(ready_file, "w", encoding="utf-8") as handle:
        handle.write("ready\\n")

    def _handle_stop(signum, frame):
        with open(events_file, "a", encoding="utf-8") as handle:
            handle.write("nav_launch_stopped\\n")
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _handle_stop)
    signal.signal(signal.SIGINT, _handle_stop)

    while True:
        time.sleep(0.1)

if args and args[0] == "bag":
    with open(events_file, "a", encoding="utf-8") as handle:
        handle.write(f"ros2_bag_invoked:{' '.join(args)}\\n")

    output_dir = ""
    for index, arg in enumerate(args[:-1]):
        if arg == "-o":
            output_dir = args[index + 1]
            break

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    def _handle_stop(signum, frame):
        with open(events_file, "a", encoding="utf-8") as handle:
            handle.write("rosbag_stopped\\n")
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _handle_stop)
    signal.signal(signal.SIGINT, _handle_stop)

    with open(events_file, "a", encoding="utf-8") as handle:
        handle.write("rosbag_started\\n")

    while True:
        time.sleep(0.1)

raise SystemExit(0)
""",
    )

    env = os.environ.copy()
    env["EVENTS_FILE"] = str(events_file)
    env["READY_FILE"] = str(ready_file)
    env["GO2W_ROS_SETUP_FILE"] = str(ros_setup)
    env["GO2W_INSTALL_SETUP_FILE"] = str(install_setup)
    env["GO2W_UNITREE_SETUP_FILE"] = ""
    env["GO2W_NAVIGATE_BIN"] = str(navigate_bin)
    env["GO2W_ROS2_BIN"] = str(ros2_bin)
    env["GO2W_XT16_BIN"] = str(xt16_bin)
    env["GO2W_SLAM_MAP_FILE"] = str(map_prefix)
    env["GO2W_DEBUG_BAG_OUTPUT_DIR"] = str(bag_output_dir)
    env["GO2W_XT16_WARMUP_SEC"] = "0"
    env["GO2W_NAV_READY_TIMEOUT_SEC"] = "5"
    env["GO2W_SKIP_TF_READY_WAIT"] = "1"

    subprocess.check_call(
        [
            "bash",
            str(WRAPPER_SCRIPT),
            "--waypoint",
            "wp_07",
        ],
        env=env,
    )

    event_lines = events_file.read_text().splitlines()

    assert "xt16_started" in event_lines
    assert "nav_launch_started" in event_lines
    assert "rosbag_started" in event_lines
    assert "rosbag_stopped" in event_lines
    assert "xt16_stopped" in event_lines
    assert "nav_launch_stopped" in event_lines
    assert any(line == "navigate_started:--waypoint wp_07" for line in event_lines)
    assert event_lines.index("nav_launch_started") < event_lines.index("rosbag_started")
    assert event_lines.index("rosbag_started") < event_lines.index(
        "navigate_started:--waypoint wp_07"
    )
    assert any(
        line.startswith("ros2_launch_invoked:launch go2w_real slam_rf2o.launch.py ")
        and "slam_mode:=localization" in line
        and f"slam_map_file:={map_prefix}" in line
        for line in event_lines
    )
    assert (bag_output_dir / "bag").is_dir()


def test_debug_bag_wrapper_waits_for_tf_ready_before_navigation(tmp_path):
    events_file = tmp_path / "events.log"
    ready_file = tmp_path / "bt_navigator.ready"
    tf_ready_file = tmp_path / "map_base.ready"
    ros_setup = tmp_path / "ros_setup.bash"
    install_setup = tmp_path / "install_setup.bash"
    navigate_bin = tmp_path / "fake_navigate_to_waypoint.sh"
    ros2_bin = tmp_path / "fake_ros2.py"
    xt16_bin = tmp_path / "fake_xt16.py"
    wait_for_tf_bin = tmp_path / "fake_wait_for_transform.py"
    map_prefix = tmp_path / "zt_0"
    bag_output_dir = tmp_path / "bag_output"

    ros_setup.write_text("")
    install_setup.write_text("")
    (tmp_path / "zt_0.data").write_text("data")
    (tmp_path / "zt_0.posegraph").write_text("posegraph")
    _write_executable(
        navigate_bin,
        """#!/usr/bin/env bash
set -euo pipefail
printf 'navigate_started:%s\\n' "$*" >> "${EVENTS_FILE}"
""",
    )
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

ready_file = os.environ["READY_FILE"]
args = sys.argv[1:]

if args[:2] == ["node", "list"]:
    if os.path.exists(ready_file):
        print("/bt_navigator")
    raise SystemExit(0)

if args and args[0] == "launch":
    with open(ready_file, "w", encoding="utf-8") as handle:
        handle.write("ready\\n")

    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))
    signal.signal(signal.SIGINT, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))

    while True:
        time.sleep(0.1)

if args and args[0] == "bag":
    output_dir = ""
    for index, arg in enumerate(args[:-1]):
        if arg == "-o":
            output_dir = args[index + 1]
            break

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))
    signal.signal(signal.SIGINT, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))

    while True:
        time.sleep(0.1)

raise SystemExit(0)
""",
    )
    _write_executable(
        wait_for_tf_bin,
        """#!/usr/bin/env python3
import os
import time

events_file = os.environ["EVENTS_FILE"]
tf_ready_file = os.environ["TF_READY_FILE"]

with open(events_file, "a", encoding="utf-8") as handle:
    handle.write("tf_wait_started\\n")

deadline = time.time() + 3.0
while time.time() < deadline:
    if os.path.exists(tf_ready_file):
        with open(events_file, "a", encoding="utf-8") as handle:
            handle.write("tf_wait_succeeded\\n")
        raise SystemExit(0)
    time.sleep(0.1)

with open(events_file, "a", encoding="utf-8") as handle:
    handle.write("tf_wait_failed\\n")
raise SystemExit(1)
""",
    )

    env = os.environ.copy()
    env["EVENTS_FILE"] = str(events_file)
    env["READY_FILE"] = str(ready_file)
    env["TF_READY_FILE"] = str(tf_ready_file)
    env["GO2W_ROS_SETUP_FILE"] = str(ros_setup)
    env["GO2W_INSTALL_SETUP_FILE"] = str(install_setup)
    env["GO2W_UNITREE_SETUP_FILE"] = ""
    env["GO2W_NAVIGATE_BIN"] = str(navigate_bin)
    env["GO2W_ROS2_BIN"] = str(ros2_bin)
    env["GO2W_XT16_BIN"] = str(xt16_bin)
    env["GO2W_SLAM_MAP_FILE"] = str(map_prefix)
    env["GO2W_DEBUG_BAG_OUTPUT_DIR"] = str(bag_output_dir)
    env["GO2W_XT16_WARMUP_SEC"] = "0"
    env["GO2W_NAV_READY_TIMEOUT_SEC"] = "5"
    env["GO2W_WAIT_FOR_TF_BIN"] = str(wait_for_tf_bin)

    proc = subprocess.Popen(
        [
            "bash",
            str(WRAPPER_SCRIPT),
            "--waypoint",
            "wp_07",
        ],
        env=env,
    )

    time.sleep(1.0)
    assert proc.poll() is None
    deadline = time.time() + 3.0
    while time.time() < deadline and not events_file.exists():
        time.sleep(0.05)
    event_lines = events_file.read_text().splitlines()
    assert "tf_wait_started" in event_lines
    assert "navigate_started:--waypoint wp_07" not in event_lines

    tf_ready_file.write_text("ready\n")
    return_code = proc.wait(timeout=10)
    event_lines = events_file.read_text().splitlines()

    assert return_code == 0
    assert "tf_wait_succeeded" in event_lines
    assert "navigate_started:--waypoint wp_07" in event_lines
    assert event_lines.index("tf_wait_succeeded") < event_lines.index(
        "navigate_started:--waypoint wp_07"
    )


def test_cmakelists_installs_debug_bag_wrapper_and_helper():
    cmake_text = (PACKAGE_ROOT / "CMakeLists.txt").read_text()

    assert "navigate_to_waypoint_with_debug_bag.sh" in cmake_text
    assert "navigation_debug_bag_topics.sh" in cmake_text


def test_debug_bag_wrapper_kills_entire_launch_process_group(tmp_path):
    events_file = tmp_path / "events.log"
    ready_file = tmp_path / "bt_navigator.ready"
    child_pid_file = tmp_path / "launch_child.pid"
    ros_setup = tmp_path / "ros_setup.bash"
    install_setup = tmp_path / "install_setup.bash"
    navigate_bin = tmp_path / "fake_navigate_to_waypoint.sh"
    ros2_bin = tmp_path / "fake_ros2.py"
    xt16_bin = tmp_path / "fake_xt16.py"
    launch_child_bin = tmp_path / "fake_launch_child.py"
    map_prefix = tmp_path / "zt_0"
    bag_output_dir = tmp_path / "bag_output"

    ros_setup.write_text("")
    install_setup.write_text("")
    (tmp_path / "zt_0.data").write_text("data")
    (tmp_path / "zt_0.posegraph").write_text("posegraph")
    _write_executable(
        navigate_bin,
        """#!/usr/bin/env bash
set -euo pipefail
printf 'navigate_started:%s\\n' "$*" >> "${EVENTS_FILE}"
""",
    )
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

signal.signal(signal.SIGINT, _ignore)
signal.signal(signal.SIGTERM, _ignore)

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
    output_dir = ""
    for index, arg in enumerate(args[:-1]):
        if arg == "-o":
            output_dir = args[index + 1]
            break

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))
    signal.signal(signal.SIGINT, lambda signum, frame: (_ for _ in ()).throw(SystemExit(0)))

    while True:
        time.sleep(0.1)

raise SystemExit(0)
""",
    )

    env = os.environ.copy()
    env["EVENTS_FILE"] = str(events_file)
    env["READY_FILE"] = str(ready_file)
    env["LAUNCH_CHILD_PID_FILE"] = str(child_pid_file)
    env["LAUNCH_CHILD_BIN"] = str(launch_child_bin)
    env["GO2W_ROS_SETUP_FILE"] = str(ros_setup)
    env["GO2W_INSTALL_SETUP_FILE"] = str(install_setup)
    env["GO2W_UNITREE_SETUP_FILE"] = ""
    env["GO2W_NAVIGATE_BIN"] = str(navigate_bin)
    env["GO2W_ROS2_BIN"] = str(ros2_bin)
    env["GO2W_XT16_BIN"] = str(xt16_bin)
    env["GO2W_SLAM_MAP_FILE"] = str(map_prefix)
    env["GO2W_DEBUG_BAG_OUTPUT_DIR"] = str(bag_output_dir)
    env["GO2W_XT16_WARMUP_SEC"] = "0"
    env["GO2W_NAV_READY_TIMEOUT_SEC"] = "5"
    env["GO2W_SKIP_TF_READY_WAIT"] = "1"

    subprocess.check_call(
        [
            "bash",
            str(WRAPPER_SCRIPT),
            "--waypoint",
            "wp_07",
        ],
        env=env,
    )

    child_pid = int(child_pid_file.read_text().strip())
    deadline = time.time() + 3.0
    while time.time() < deadline and _pid_exists(child_pid):
        time.sleep(0.1)

    event_lines = events_file.read_text().splitlines()

    assert "launch_child_started" in event_lines
    assert any(line.startswith("nav_launch_stopped:SIGINT") for line in event_lines)
    assert any(line.startswith("launch_child_ignored:SIGINT") for line in event_lines)
    assert not _pid_exists(child_pid)
