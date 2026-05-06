#!/usr/bin/env python3
"""
GO2W Navigation Launch Manager — HTTP API for the phone operator console.

Responsibilities:
  * Start / stop the GO2W navigation stack via a whitelist of profiles
    (profiles.yaml).
  * List POIs loaded from a waypoint YAML (default: go2w_map_waypoints.yaml).
  * Dispatch a navigate_to_waypoint.py subprocess per POI selection, with
    strict single-instance tracking (a new goto preempts the previous one).
  * Serve a static web UI for the phone control panel.

No outbound WebSocket connection is opened — the phone talks to this server
over the robot's LAN, and Nav2 goals are sent locally via the ROS 2 action
client inside navigate_to_waypoint.py.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import json
import logging
import math
import os
import signal
import subprocess
import sys
import threading
import time
import shlex
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from typing import Optional

import yaml

logger = logging.getLogger("launch_manager")


DEFAULT_PROFILES_FILE = Path(__file__).parent / "profiles.yaml"
LOG_BUFFER_MAX = 2000
NAV_LOG_BUFFER_MAX = 500
DEFAULT_NAVIGATE_COMMAND_TEMPLATE = (
    "source /opt/ros/foxy/setup.bash && "
    "if [ -f /home/unitree/unitree_ros2/setup.sh ]; then "
    "source /home/unitree/unitree_ros2/setup.sh; fi && "
    "source /home/unitree/ros_ws/install/setup.bash && "
    "exec ros2 run go2w_real navigate_to_waypoint.py "
    '--waypoint-file "$GO2W_WAYPOINT_FILE" '
    '--waypoint "{{waypoint}}"'
)
DEFAULT_ESTOP_COMMAND = (
    "source /opt/ros/foxy/setup.bash && "
    "if [ -f /home/unitree/unitree_ros2/setup.sh ]; then "
    "source /home/unitree/unitree_ros2/setup.sh; fi && "
    "source /home/unitree/ros_ws/install/setup.bash && "
    "exec ros2 run go2w_real emergency_stop.py"
)
WAYPOINT_TEMPLATE_TOKEN = "{{waypoint}}"
ESTOP_TIMEOUT_SECONDS = 6.0
DEFAULT_DEBUG_BAG_COMMAND = (
    "/home/unitree/ros_ws/src/go2w_real/scripts/"
    "navigate_to_waypoint_with_debug_bag.sh"
)
DEFAULT_DEBUG_PROFILES = ["zt_0", "14_1"]
DEFAULT_FOXGLOVE_PORT = 8765
DEFAULT_FOXGLOVE_URL = (
    "https://app.foxglove.dev/~/view"
    "?ds=foxglove-websocket&ds.url=ws://{{host}}:{{foxglove_port}}"
)
DEFAULT_RVIZ_URL = "http://{{host}}:6080/vnc.html?autoconnect=true&resize=remote"
SAFE_TOKEN_CHARS = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-.:/")


# ---------------------------------------------------------------------------
# Profile + navigation config loader
# ---------------------------------------------------------------------------


def load_profiles(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    profiles: dict[str, dict] = {}
    for name, entry in (data.get("profiles") or {}).items():
        profiles[name] = {
            "command": entry["command"],
            "description": entry.get("description", ""),
            "env": entry.get("env", {}) or {},
        }

    nav_cfg = data.get("navigation") or {}
    waypoint_file_raw = str(nav_cfg.get("waypoint_file", "")).strip()
    navigate_command = str(
        nav_cfg.get("navigate_command", DEFAULT_NAVIGATE_COMMAND_TEMPLATE)
    ).strip()
    estop_command = str(
        nav_cfg.get("estop_command", DEFAULT_ESTOP_COMMAND)
    ).strip()
    if WAYPOINT_TEMPLATE_TOKEN not in navigate_command:
        raise ValueError(
            "navigation.navigate_command must contain the "
            f"'{WAYPOINT_TEMPLATE_TOKEN}' placeholder"
        )

    return {
        "profiles": profiles,
        "navigation": {
            "waypoint_file": os.path.expanduser(waypoint_file_raw)
            if waypoint_file_raw
            else "",
            "navigate_command": navigate_command,
            "estop_command": estop_command,
        },
        "debug_navigation": _load_debug_navigation_config(data),
        "visualization": _load_visualization_config(data),
    }


def _load_debug_navigation_config(data: dict) -> dict:
    raw_cfg = data.get("debug_navigation") or {}
    allowed_profiles = raw_cfg.get("allowed_profiles", DEFAULT_DEBUG_PROFILES)
    if not isinstance(allowed_profiles, list) or not allowed_profiles:
        raise ValueError("debug_navigation.allowed_profiles must be a non-empty list")

    command = str(raw_cfg.get("command", DEFAULT_DEBUG_BAG_COMMAND)).strip()
    if not command:
        raise ValueError("debug_navigation.command is required")

    default_profile = str(
        raw_cfg.get("default_profile", allowed_profiles[0])
    ).strip()
    if default_profile not in allowed_profiles:
        raise ValueError("debug_navigation.default_profile must be allowed")

    defaults = raw_cfg.get("defaults") or {}
    if not isinstance(defaults, dict):
        raise ValueError("debug_navigation.defaults must be a mapping")

    waypoint_files = raw_cfg.get("waypoint_files") or {}
    if not isinstance(waypoint_files, dict):
        raise ValueError("debug_navigation.waypoint_files must be a mapping")
    resolved_waypoint_files = {
        str(profile).strip(): os.path.expanduser(str(path).strip())
        for profile, path in waypoint_files.items()
        if str(path).strip()
    }

    return {
        "command": os.path.expanduser(command),
        "allowed_profiles": [str(profile).strip() for profile in allowed_profiles],
        "default_profile": default_profile,
        "waypoint_files": resolved_waypoint_files,
        "defaults": defaults,
    }


def _load_visualization_config(data: dict) -> dict:
    raw_cfg = data.get("visualization") or {}
    foxglove_port = int(raw_cfg.get("foxglove_port", DEFAULT_FOXGLOVE_PORT))
    return {
        "foxglove_port": foxglove_port,
        "foxglove_url": str(
            raw_cfg.get("foxglove_url", DEFAULT_FOXGLOVE_URL)
        ).strip(),
        "rviz_url": str(raw_cfg.get("rviz_url", DEFAULT_RVIZ_URL)).strip(),
    }


# ---------------------------------------------------------------------------
# Waypoint loader (kept tolerant; mirrors navigate_to_waypoint.py)
# ---------------------------------------------------------------------------


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_waypoints(path: Path) -> list:
    waypoint_path = Path(os.path.expanduser(str(path)))
    with open(waypoint_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    raw_waypoints = data.get("waypoints", [])
    if not isinstance(raw_waypoints, list):
        raise ValueError(
            f"Invalid waypoint file: 'waypoints' is not a list in {waypoint_path}"
        )

    waypoints: list[dict] = []
    for idx, raw_wp in enumerate(raw_waypoints, start=1):
        if not isinstance(raw_wp, dict):
            continue
        position = raw_wp.get("position", {}) or {}
        orientation = raw_wp.get("orientation", {}) or {}

        yaw = raw_wp.get("yaw")
        if yaw is None and all(k in orientation for k in ("x", "y", "z", "w")):
            yaw = _yaw_from_quaternion(
                float(orientation["x"]),
                float(orientation["y"]),
                float(orientation["z"]),
                float(orientation["w"]),
            )
        elif yaw is None:
            yaw = 0.0
        else:
            yaw = float(yaw)

        name = str(raw_wp.get("name", f"wp_{idx:02d}"))
        waypoint_id = str(raw_wp.get("id", name))
        waypoints.append(
            {
                "index": idx,
                "id": waypoint_id,
                "name": name,
                "frame_id": str(raw_wp.get("frame_id", "map")),
                "position": {
                    "x": float(position.get("x", 0.0)),
                    "y": float(position.get("y", 0.0)),
                    "z": float(position.get("z", 0.0)),
                },
                "yaw": float(yaw),
            }
        )
    return waypoints


# ---------------------------------------------------------------------------
# Managed process (navigation stack)
# ---------------------------------------------------------------------------


class ManagedProcess:
    """Wraps a navigation-stack subprocess with log capture + lifecycle."""

    def __init__(self, profile_name: str, command: str, env: dict):
        self.profile_name = profile_name
        self.command = command
        self.env = env
        self.process: Optional[subprocess.Popen] = None
        self.started_at: Optional[datetime.datetime] = None
        self.stopped_at: Optional[datetime.datetime] = None
        self.log_buffer: collections.deque = collections.deque(maxlen=LOG_BUFFER_MAX)
        self._reader_thread: Optional[threading.Thread] = None
        self._state = "stopped"
        self._lock = threading.Lock()

    @property
    def state(self) -> str:
        with self._lock:
            if self._state in ("running", "starting"):
                if self.process and self.process.poll() is not None:
                    self._state = "stopped"
                    self.stopped_at = datetime.datetime.now(datetime.timezone.utc)
            return self._state

    @property
    def pid(self) -> Optional[int]:
        return self.process.pid if self.process else None

    @property
    def uptime_seconds(self) -> Optional[float]:
        if self.started_at and self.state in ("running", "starting"):
            return (
                datetime.datetime.now(datetime.timezone.utc) - self.started_at
            ).total_seconds()
        return None

    def start(self) -> dict:
        with self._lock:
            if self._state in ("running", "starting"):
                if self.process and self.process.poll() is None:
                    return {
                        "ok": False,
                        "error": (
                            f"Profile '{self.profile_name}' is already running "
                            f"(pid {self.process.pid})"
                        ),
                    }
            self._state = "starting"

        run_env = os.environ.copy()
        run_env.update({k: str(v) for k, v in self.env.items()})

        try:
            self.process = subprocess.Popen(
                ["bash", "-lc", self.command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=run_env,
                start_new_session=True,
            )
        except Exception as exc:
            with self._lock:
                self._state = "stopped"
            return {"ok": False, "error": str(exc)}

        self.started_at = datetime.datetime.now(datetime.timezone.utc)
        self.stopped_at = None
        self.log_buffer.clear()

        self._reader_thread = threading.Thread(
            target=self._read_output, daemon=True
        )
        self._reader_thread.start()

        with self._lock:
            self._state = "running"

        logger.info(
            "Started profile '%s' with pid %d", self.profile_name, self.process.pid
        )
        return {"ok": True, "pid": self.process.pid}

    def stop(self, grace_seconds: float = 10.0) -> dict:
        with self._lock:
            if self._state == "stopped":
                return {"ok": True, "message": "Already stopped"}
            if self._state == "stopping":
                return {"ok": False, "error": "Already stopping"}
            self._state = "stopping"

        if self.process and self.process.poll() is None:
            _terminate_process_group(self.process, grace_seconds)

        with self._lock:
            self._state = "stopped"
        self.stopped_at = datetime.datetime.now(datetime.timezone.utc)
        logger.info("Stopped profile '%s'", self.profile_name)
        return {"ok": True}

    def status(self) -> dict:
        s = self.state
        result: dict = {"profile": self.profile_name, "state": s}
        if self.pid is not None:
            result["pid"] = self.pid
        if self.started_at:
            result["started_at"] = self.started_at.isoformat()
        if s in ("running", "starting") and self.uptime_seconds is not None:
            result["uptime_seconds"] = round(self.uptime_seconds, 1)
        if self.process and self.process.poll() is not None:
            result["exit_code"] = self.process.returncode
        return result

    def logs(self, tail: int = 100) -> list:
        lines = list(self.log_buffer)
        return lines[-tail:]

    def _read_output(self):
        assert self.process and self.process.stdout
        try:
            for raw_line in self.process.stdout:
                try:
                    line = raw_line.decode("utf-8", errors="replace").rstrip("\n")
                except Exception:
                    line = repr(raw_line)
                self.log_buffer.append(line)
        except Exception:
            pass


def _terminate_process_group(process: subprocess.Popen, grace_seconds: float):
    """SIGTERM the process group, then SIGKILL on grace timeout.

    Works on Linux (operator robot) via killpg; falls back to process-level
    terminate()/kill() on platforms without getpgid (Windows dev).
    """
    if hasattr(os, "getpgid"):
        pgid = None
        try:
            pgid = os.getpgid(process.pid)
        except OSError:
            pass
        if pgid:
            try:
                os.killpg(pgid, signal.SIGTERM)
            except OSError:
                pass
        else:
            process.terminate()
    else:
        process.terminate()

    try:
        process.wait(timeout=grace_seconds)
        return
    except subprocess.TimeoutExpired:
        logger.warning("Grace period expired, sending SIGKILL")

    if hasattr(os, "killpg"):
        pgid = None
        try:
            pgid = os.getpgid(process.pid)
        except OSError:
            pass
        if pgid:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except OSError:
                pass
    try:
        process.kill()
        process.wait(timeout=5)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Process manager (navigation stack, multi-profile)
# ---------------------------------------------------------------------------


class ProcessManager:
    def __init__(self, profiles: dict):
        self.profiles = profiles
        self._managed: dict[str, ManagedProcess] = {}
        self._lock = threading.Lock()

    def get_or_create(self, profile_name: str) -> ManagedProcess:
        with self._lock:
            if profile_name not in self._managed:
                cfg = self.profiles[profile_name]
                self._managed[profile_name] = ManagedProcess(
                    profile_name=profile_name,
                    command=cfg["command"],
                    env=cfg.get("env", {}) or {},
                )
            return self._managed[profile_name]

    def stop_all(self):
        with self._lock:
            names = list(self._managed.keys())
        for name in names:
            proc = self._managed.get(name)
            if proc and proc.state in ("running", "starting"):
                proc.stop()

    def running_profiles(self) -> list:
        """Names of profiles that are currently running or starting."""
        with self._lock:
            names = list(self._managed.keys())
        live = []
        for name in names:
            proc = self._managed.get(name)
            if proc and proc.state in ("running", "starting"):
                live.append(name)
        return live

    def stop_running(self, grace_seconds: float = 5.0) -> list:
        """Stop every running profile. Returns the names actually stopped."""
        stopped: list = []
        for name in self.running_profiles():
            proc = self._managed.get(name)
            if proc is None:
                continue
            try:
                proc.stop(grace_seconds=grace_seconds)
                stopped.append(name)
            except Exception as exc:  # noqa: BLE001
                logger.warning("Failed to stop profile '%s': %s", name, exc)
        return stopped


# ---------------------------------------------------------------------------
# Active navigation (POI dispatcher)
# ---------------------------------------------------------------------------


_NAV_SUCCESS_MARKERS = ("reached successfully",)
_NAV_FAILURE_MARKERS = (
    "rejected the waypoint goal",
    "retry budget exhausted",
    "could not find a path",
)
_NAV_FINISHED_STATUS_MARKER = "finished with status"


class NavigationDispatcher:
    """Runs one navigate_to_waypoint.py at a time.

    Starting a new navigation preempts (SIGTERM + waits + SIGKILL) the
    previous one. Exposes a status summary for the UI and a recent-log tail.
    """

    def __init__(
        self,
        command_template: str,
        default_waypoint_file: str,
        estop_command: str = DEFAULT_ESTOP_COMMAND,
    ):
        self._command_template = command_template
        self._default_waypoint_file = default_waypoint_file
        self._estop_command = estop_command
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._reader: Optional[threading.Thread] = None
        self._monitor: Optional[threading.Thread] = None
        self._log_buffer: collections.deque = collections.deque(
            maxlen=NAV_LOG_BUFFER_MAX
        )
        self._target_id: Optional[str] = None
        self._started_at: Optional[datetime.datetime] = None
        self._finished_at: Optional[datetime.datetime] = None
        self._state = "idle"  # idle | starting | running | succeeded | failed | canceled | estopped
        self._exit_code: Optional[int] = None
        self._last_error: Optional[str] = None
        self._last_distance: Optional[float] = None
        self._observed_outcome: Optional[str] = None  # set from log heuristics
        self._last_estop_at: Optional[datetime.datetime] = None
        self._last_estop_ok: Optional[bool] = None
        self._last_estop_detail: Optional[str] = None
        self._estop_lock = threading.Lock()

    @property
    def default_waypoint_file(self) -> str:
        return self._default_waypoint_file

    def status(self) -> dict:
        with self._lock:
            result: dict = {
                "state": self._state,
                "target_id": self._target_id,
            }
            if self._started_at:
                result["started_at"] = self._started_at.isoformat()
            if self._finished_at:
                result["finished_at"] = self._finished_at.isoformat()
            if self._started_at and self._state in ("starting", "running"):
                result["uptime_seconds"] = round(
                    (
                        datetime.datetime.now(datetime.timezone.utc)
                        - self._started_at
                    ).total_seconds(),
                    1,
                )
            if self._exit_code is not None:
                result["exit_code"] = self._exit_code
            if self._last_error:
                result["error"] = self._last_error
            if self._last_distance is not None:
                result["remaining_distance"] = self._last_distance
            if self._process is not None and self._process.poll() is None:
                result["pid"] = self._process.pid
            if self._last_estop_at is not None:
                result["last_estop"] = {
                    "at": self._last_estop_at.isoformat(),
                    "ok": bool(self._last_estop_ok),
                    "detail": self._last_estop_detail or "",
                }
            return result

    def logs(self, tail: int = 100) -> list:
        with self._lock:
            return list(self._log_buffer)[-tail:]

    def start(self, target_id: str, waypoint_file: Optional[str] = None) -> dict:
        target_id = str(target_id).strip()
        if not target_id:
            return {"ok": False, "error": "target_id is required"}

        waypoint_file = waypoint_file or self._default_waypoint_file
        if not waypoint_file:
            return {
                "ok": False,
                "error": "no waypoint_file configured for navigation",
            }

        # Preempt any in-flight navigation before spinning up the new one.
        self._stop_current_locked_external(reason="preempted")

        if WAYPOINT_TEMPLATE_TOKEN not in self._command_template:
            return {
                "ok": False,
                "error": (
                    "navigate_command template is missing the "
                    f"'{WAYPOINT_TEMPLATE_TOKEN}' placeholder"
                ),
            }

        command = self._command_template.replace(
            WAYPOINT_TEMPLATE_TOKEN, _shell_escape(target_id)
        )

        run_env = os.environ.copy()
        run_env["GO2W_WAYPOINT_FILE"] = waypoint_file
        # navigate_to_waypoint.py reads input() when it runs out of the
        # explicit --waypoint; close stdin so EOFError fires and it exits
        # cleanly after one goal.
        try:
            process = subprocess.Popen(
                ["bash", "-lc", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                env=run_env,
                start_new_session=True,
            )
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

        with self._lock:
            self._process = process
            self._target_id = target_id
            self._started_at = datetime.datetime.now(datetime.timezone.utc)
            self._finished_at = None
            self._state = "running"
            self._exit_code = None
            self._last_error = None
            self._last_distance = None
            self._observed_outcome = None
            self._log_buffer.clear()
            self._reader = threading.Thread(
                target=self._read_output, args=(process,), daemon=True
            )
            self._monitor = threading.Thread(
                target=self._monitor_exit, args=(process,), daemon=True
            )
            self._reader.start()
            self._monitor.start()

        logger.info(
            "Dispatched navigation to '%s' (pid %d)", target_id, process.pid
        )
        return {"ok": True, "pid": process.pid, "target_id": target_id}

    def cancel(self, reason: str = "user cancel") -> dict:
        canceled = self._stop_current_locked_external(reason=reason)
        if not canceled:
            return {"ok": True, "message": "No active navigation"}
        return {"ok": True, "message": f"Canceled ({reason})"}

    def estop(
        self,
        stack_manager: Optional["ProcessManager"] = None,
        timeout_sec: float = ESTOP_TIMEOUT_SECONDS,
    ) -> dict:
        """Nuclear emergency stop.

        Fires in this order, with the zero-velocity stream and the stack
        teardown happening **in parallel** so the robot keeps receiving
        Twist(0) while Nav2/SLAM/xt16 are being killed:

          1. SIGKILL the in-flight navigate_to_waypoint.py subprocess (no
             grace, fastest-path).
          2. Spawn the whitelisted emergency_stop helper: it cancels every
             /navigate_to_pose goal via the action's cancel_goal service,
             then publishes Twist(0) to /cmd_vel at ~30 Hz for ~3 s.
          3. In parallel with (2), if a stack_manager is given, stop every
             running profile (SIGTERM → SIGKILL via killpg). Profile shell
             scripts (start_navigation_no_server.sh etc.) run their own
             cleanup traps so Nav2, slam_rf2o, and xt16_driver go down
             together.
          4. Wait for both the helper and the teardown thread to finish
             (capped at 2 × timeout_sec so a wedged profile shell never
             blocks the caller indefinitely).

        The estop_lock keeps concurrent button taps from fanning out into
        multiple helpers.
        """
        with self._estop_lock:
            t0 = time.monotonic()
            killed_pid = self._hard_kill_current(reason="E-STOP")

            if not self._estop_command:
                detail = "no estop_command configured"
                self._record_estop(False, detail)
                return {"ok": False, "killed_process": killed_pid, "error": detail}

            # (2) helper — published zeros bracket the teardown.
            helper_result: dict = {"kind": "pending"}

            def _run_helper():
                try:
                    proc = subprocess.run(
                        ["bash", "-lc", self._estop_command],
                        stdin=subprocess.DEVNULL,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        timeout=timeout_sec,
                        start_new_session=True,
                    )
                    helper_result.update(
                        kind="done",
                        returncode=proc.returncode,
                        stdout=(proc.stdout or b"").decode("utf-8", errors="replace"),
                    )
                except subprocess.TimeoutExpired as exc:
                    helper_result.update(
                        kind="timeout",
                        stdout=(exc.stdout or b"").decode("utf-8", errors="replace")
                        if exc.stdout
                        else "",
                    )
                except Exception as exc:  # noqa: BLE001
                    helper_result.update(kind="error", error=str(exc))

            helper_thread = threading.Thread(
                target=_run_helper, name="estop-helper", daemon=True
            )
            helper_thread.start()

            # (3) in parallel, stop every running profile.
            stop_outcome: dict = {"stopped": [], "error": None}

            def _run_stop_all():
                if stack_manager is None:
                    return
                try:
                    stop_outcome["stopped"] = stack_manager.stop_running(
                        grace_seconds=5.0
                    )
                except Exception as exc:  # noqa: BLE001
                    stop_outcome["error"] = str(exc)

            stop_thread = threading.Thread(
                target=_run_stop_all, name="estop-stop-all", daemon=True
            )
            stop_thread.start()

            # (4) await both, cap so a hung profile script cannot wedge us.
            overall_deadline = t0 + max(timeout_sec * 2.0, timeout_sec + 5.0)

            helper_thread.join(timeout=max(overall_deadline - time.monotonic(), 0.1))
            stop_thread.join(timeout=max(overall_deadline - time.monotonic(), 0.1))

            elapsed = time.monotonic() - t0
            kind = helper_result.get("kind")
            returncode = helper_result.get("returncode")
            stdout_text = helper_result.get("stdout", "")

            if kind == "done":
                ok = returncode == 0
                detail = f"helper exit {returncode} in {elapsed:.2f}s"
            elif kind == "timeout":
                ok = False
                detail = f"helper timed out after {timeout_sec:.1f}s"
            elif kind == "error":
                ok = False
                detail = f"helper spawn failed: {helper_result.get('error')}"
            else:
                ok = False
                detail = "helper did not complete in time"

            self._append_nav_log_locked(f"[E-STOP] {detail}")
            if stdout_text.strip():
                self._append_nav_log_locked(stdout_text.rstrip("\n"))
            if stop_outcome["stopped"]:
                self._append_nav_log_locked(
                    "[E-STOP] stopped profiles: "
                    + ", ".join(stop_outcome["stopped"])
                )
            if stop_outcome["error"]:
                self._append_nav_log_locked(
                    "[E-STOP] stop_all error: " + stop_outcome["error"]
                )

            self._record_estop(ok, detail)

            with self._lock:
                if self._state in ("starting", "running"):
                    self._state = "estopped"
                    self._finished_at = datetime.datetime.now(datetime.timezone.utc)
                    self._last_error = "emergency stop"

            return {
                "ok": ok,
                "killed_process": killed_pid,
                "stopped_profiles": stop_outcome["stopped"],
                "stop_error": stop_outcome["error"],
                "exit_code": returncode,
                "elapsed_seconds": round(elapsed, 2),
                "stdout": stdout_text,
                "helper": kind,
            }

    def _hard_kill_current(self, reason: str) -> bool:
        """SIGKILL the active process group. Returns True if a process was
        killed."""
        with self._lock:
            proc = self._process
            target = self._target_id
            if proc is None or proc.poll() is not None:
                return False

        logger.warning(
            "Hard-killing navigation subprocess for '%s' (%s)", target, reason
        )
        if hasattr(os, "killpg"):
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except OSError:
                try:
                    proc.kill()
                except Exception:
                    pass
        else:
            try:
                proc.kill()
            except Exception:
                pass

        try:
            proc.wait(timeout=2.0)
        except Exception:
            pass

        with self._lock:
            if self._process is proc:
                self._state = "estopped"
                self._finished_at = datetime.datetime.now(datetime.timezone.utc)
                self._exit_code = proc.returncode
                self._last_error = f"hard-killed: {reason}"
        return True

    def _append_nav_log_locked(self, text: str):
        with self._lock:
            for line in str(text).splitlines() or [str(text)]:
                self._log_buffer.append(line)

    def _record_estop(self, ok: bool, detail: str):
        with self._lock:
            self._last_estop_at = datetime.datetime.now(datetime.timezone.utc)
            self._last_estop_ok = bool(ok)
            self._last_estop_detail = detail

    def _stop_current_locked_external(self, reason: str) -> bool:
        """Terminate the in-flight process and record outcome."""
        with self._lock:
            proc = self._process
            if proc is None or proc.poll() is not None:
                return False
            if self._state in ("idle", "succeeded", "failed", "canceled"):
                return False
            target_id = self._target_id

        logger.info("Canceling navigation to '%s' (%s)", target_id, reason)
        _terminate_process_group(proc, grace_seconds=4.0)

        with self._lock:
            if self._process is proc and self._state in ("starting", "running"):
                self._state = "canceled"
                self._finished_at = datetime.datetime.now(datetime.timezone.utc)
                self._exit_code = proc.returncode
                self._last_error = f"canceled: {reason}"
        return True

    def _read_output(self, process: subprocess.Popen):
        assert process.stdout is not None
        try:
            for raw_line in process.stdout:
                try:
                    line = raw_line.decode("utf-8", errors="replace").rstrip("\n")
                except Exception:
                    line = repr(raw_line)
                with self._lock:
                    self._log_buffer.append(line)
                    self._scrape_line_locked(line)
        except Exception:
            pass

    def _scrape_line_locked(self, line: str):
        # distance scraping: "Nav2 feedback: dist=1.234 m, ..."
        marker = "dist="
        if marker in line:
            try:
                frag = line.split(marker, 1)[1]
                number = frag.split()[0]
                number = number.rstrip(",")
                self._last_distance = float(number)
            except (ValueError, IndexError):
                pass

        lower = line.lower()
        if any(m in lower for m in _NAV_SUCCESS_MARKERS):
            self._observed_outcome = "succeeded"
            return
        if any(m in lower for m in _NAV_FAILURE_MARKERS):
            self._observed_outcome = "failed"
            if not self._last_error:
                self._last_error = line.strip()
            return
        if _NAV_FINISHED_STATUS_MARKER in lower and "succeeded" not in lower:
            self._observed_outcome = "failed"
            if not self._last_error:
                self._last_error = line.strip()

    def _monitor_exit(self, process: subprocess.Popen):
        returncode = process.wait()
        with self._lock:
            if self._process is not process:
                return
            self._finished_at = datetime.datetime.now(datetime.timezone.utc)
            self._exit_code = returncode

            if self._state == "canceled":
                return
            if self._observed_outcome == "succeeded":
                self._state = "succeeded"
            elif self._observed_outcome == "failed":
                self._state = "failed"
            elif returncode == 0:
                # Exit 0 without an explicit "reached" — treat as succeeded.
                self._state = "succeeded"
            else:
                self._state = "failed"
                if not self._last_error:
                    self._last_error = f"process exited with code {returncode}"

    def shutdown(self):
        self._stop_current_locked_external(reason="launch_manager shutdown")


def _is_safe_token(value: str) -> bool:
    return bool(value) and all(char in SAFE_TOKEN_CHARS for char in value)


def _optional_float(
    request: dict,
    key: str,
    flag: str,
    *,
    minimum: float = 0.0,
    maximum: float = 10000.0,
) -> list[str]:
    if key not in request or request.get(key) in (None, ""):
        return []
    try:
        value = float(request[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be numeric") from exc
    if value < minimum or value > maximum:
        raise ValueError(f"{key} out of range")
    return [flag, str(request[key])]


def _optional_int(
    request: dict,
    key: str,
    flag: str,
    *,
    minimum: int = 0,
    maximum: int = 10000,
) -> list[str]:
    if key not in request or request.get(key) in (None, ""):
        return []
    try:
        value = int(request[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be an integer") from exc
    if value < minimum or value > maximum:
        raise ValueError(f"{key} out of range")
    return [flag, str(value)]


def build_debug_bag_args(
    command: str,
    *,
    allowed_profiles: list[str],
    profile_waypoint_files: dict[str, str],
    request: dict,
) -> list[str]:
    profile = str(request.get("profile", allowed_profiles[0])).strip()
    if profile not in allowed_profiles:
        raise ValueError(f"unknown debug profile: {profile}")

    args = [command, "--profile", profile]
    waypoint_file = str(profile_waypoint_files.get(profile, "")).strip()
    if waypoint_file:
        args += ["--waypoint-file", waypoint_file]

    if bool(request.get("no_bringup", False)):
        args.append("--no-bringup")

    args += _optional_float(
        request, "nav_ready_timeout", "--nav-ready-timeout", minimum=1.0
    )
    args += _optional_float(
        request, "tf_ready_timeout", "--tf-ready-timeout", minimum=0.1
    )
    args += _optional_float(
        request, "scan_ready_timeout", "--scan-ready-timeout", minimum=0.1
    )
    args += _optional_float(
        request, "scan_ready_min_hz", "--scan-ready-min-hz", minimum=0.1
    )
    args += _optional_float(
        request, "xt16_ready_timeout", "--xt16-ready-timeout", minimum=0.1
    )
    args += _optional_float(
        request, "xt16_ready_min_hz", "--xt16-ready-min-hz", minimum=0.1
    )
    args += _optional_int(
        request, "xt16_ready_restarts", "--xt16-ready-restarts", minimum=0
    )

    navigate_args: list[str] = []
    target_id = str(request.get("target_id", "")).strip()
    sequence = str(request.get("waypoint_sequence", "")).strip()
    all_waypoints = bool(request.get("all_waypoints", False))

    selected_modes = sum(bool(value) for value in (target_id, sequence, all_waypoints))
    if selected_modes != 1:
        raise ValueError(
            "select exactly one debug navigation target: target_id, "
            "waypoint_sequence, or all_waypoints"
        )

    if target_id:
        if not _is_safe_token(target_id):
            raise ValueError("target_id contains unsupported characters")
        navigate_args += ["--waypoint", target_id]
    elif sequence:
        if any(not _is_safe_token(item.strip()) for item in sequence.split(",")):
            raise ValueError("waypoint_sequence contains unsupported characters")
        navigate_args += ["--waypoint-sequence", sequence]
    elif all_waypoints:
        navigate_args.append("--all-waypoints")

    navigate_args += _optional_float(
        request, "near_goal_distance", "--near-goal-distance", minimum=0.01
    )
    navigate_args += _optional_float(
        request,
        "success_check_distance",
        "--success-check-distance",
        minimum=0.01,
    )
    navigate_args += _optional_float(
        request,
        "success_check_timeout",
        "--success-check-timeout",
        minimum=0.0,
    )
    navigate_args += _optional_int(
        request,
        "abort_replan_retries",
        "--abort-replan-retries",
        minimum=0,
    )
    navigate_args += _optional_float(
        request,
        "batch_inter_goal_delay",
        "--batch-inter-goal-delay",
        minimum=0.0,
    )
    if bool(request.get("use_waypoint_yaw", False)):
        navigate_args.append("--use-waypoint-yaw")
    if bool(request.get("continue_on_failure", False)):
        navigate_args.append("--continue-on-failure")

    return args + ["--"] + navigate_args


class DebugBagDispatcher:
    """Runs one navigate_to_waypoint_with_debug_bag.sh task at a time."""

    def __init__(self, config: dict):
        self.config = config
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._reader: Optional[threading.Thread] = None
        self._monitor: Optional[threading.Thread] = None
        self._log_buffer: collections.deque = collections.deque(
            maxlen=NAV_LOG_BUFFER_MAX
        )
        self._state = "idle"
        self._started_at: Optional[datetime.datetime] = None
        self._finished_at: Optional[datetime.datetime] = None
        self._exit_code: Optional[int] = None
        self._last_error: Optional[str] = None
        self._last_request: dict = {}
        self._last_command: list[str] = []

    def start(self, request: dict) -> dict:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return {"ok": False, "error": "debug navigation is already running"}

        merged_request = dict(self.config.get("defaults", {}))
        merged_request.update(request or {})
        if "profile" not in merged_request:
            merged_request["profile"] = self.config["default_profile"]

        try:
            argv = build_debug_bag_args(
                self.config["command"],
                allowed_profiles=self.config["allowed_profiles"],
                profile_waypoint_files=self.config.get("waypoint_files", {}),
                request=merged_request,
            )
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}

        run_env = os.environ.copy()
        run_env.update(
            {
                key: str(value)
                for key, value in (self.config.get("env") or {}).items()
            }
        )

        try:
            process = subprocess.Popen(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                env=run_env,
                start_new_session=True,
            )
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

        with self._lock:
            self._process = process
            self._state = "running"
            self._started_at = datetime.datetime.now(datetime.timezone.utc)
            self._finished_at = None
            self._exit_code = None
            self._last_error = None
            self._last_request = merged_request
            self._last_command = argv
            self._log_buffer.clear()
            self._reader = threading.Thread(
                target=self._read_output,
                args=(process,),
                name="debug-bag-reader",
                daemon=True,
            )
            self._monitor = threading.Thread(
                target=self._monitor_exit,
                args=(process,),
                name="debug-bag-monitor",
                daemon=True,
            )
            self._reader.start()
            self._monitor.start()

        return {"ok": True, "pid": process.pid, "argv": argv}

    def cancel(self, reason: str = "user cancel") -> dict:
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                return {"ok": True, "message": "No active debug navigation"}

        _terminate_process_group(process, grace_seconds=5.0)
        with self._lock:
            if self._process is process:
                self._state = "canceled"
                self._finished_at = datetime.datetime.now(datetime.timezone.utc)
                self._exit_code = process.returncode
                self._last_error = f"canceled: {reason}"
        return {"ok": True, "message": f"Canceled debug navigation ({reason})"}

    def status(self) -> dict:
        with self._lock:
            if (
                self._state in ("running", "starting")
                and self._process is not None
                and self._process.poll() is not None
            ):
                self._state = "succeeded" if self._process.returncode == 0 else "failed"
                self._finished_at = datetime.datetime.now(datetime.timezone.utc)
                self._exit_code = self._process.returncode

            result: dict = {
                "state": self._state,
                "request": self._last_request,
            }
            if self._process is not None and self._process.poll() is None:
                result["pid"] = self._process.pid
            if self._started_at:
                result["started_at"] = self._started_at.isoformat()
            if self._finished_at:
                result["finished_at"] = self._finished_at.isoformat()
            if self._started_at and self._state in ("starting", "running"):
                result["uptime_seconds"] = round(
                    (
                        datetime.datetime.now(datetime.timezone.utc)
                        - self._started_at
                    ).total_seconds(),
                    1,
                )
            if self._exit_code is not None:
                result["exit_code"] = self._exit_code
            if self._last_error:
                result["error"] = self._last_error
            if self._last_command:
                result["command"] = shlex.join(self._last_command)
            return result

    def logs(self, tail: int = 100) -> list:
        with self._lock:
            return list(self._log_buffer)[-tail:]

    def shutdown(self):
        self.cancel(reason="launch_manager shutdown")

    def _read_output(self, process: subprocess.Popen):
        assert process.stdout is not None
        try:
            for raw_line in process.stdout:
                line = raw_line.decode("utf-8", errors="replace").rstrip("\n")
                with self._lock:
                    self._log_buffer.append(line)
        except Exception:
            pass

    def _monitor_exit(self, process: subprocess.Popen):
        returncode = process.wait()
        with self._lock:
            if self._process is not process:
                return
            if self._state == "canceled":
                return
            self._finished_at = datetime.datetime.now(datetime.timezone.utc)
            self._exit_code = returncode
            self._state = "succeeded" if returncode == 0 else "failed"
            if returncode != 0:
                self._last_error = f"process exited with code {returncode}"


def _shell_escape(value: str) -> str:
    safe = all(
        c.isalnum() or c in ("_", "-", ".", "/", ":", "+", "=")
        for c in value
    )
    if safe and value:
        return value
    return "'" + value.replace("'", "'\\''") + "'"


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------


class LaunchManagerHandler(BaseHTTPRequestHandler):
    manager: ProcessManager
    profiles: dict
    navigator: NavigationDispatcher
    debug_navigator: DebugBagDispatcher
    visualization: dict

    def log_message(self, fmt, *args):
        logger.debug(fmt, *args)

    # --- helpers ---------------------------------------------------------

    def _send_json(self, data: dict, status: int = 200):
        body = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {}

    def _parse_query(self) -> dict:
        if "?" not in self.path:
            return {}
        qs = self.path.split("?", 1)[1]
        result = {}
        for part in qs.split("&"):
            if "=" in part:
                k, v = part.split("=", 1)
                result[k] = v
        return result

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    # --- GET routes ------------------------------------------------------

    def do_GET(self):
        path = self.path.split("?")[0].rstrip("/")
        query = self._parse_query()

        if path == "/api/health":
            services = {}
            for name in self.profiles:
                services[name] = self.manager.get_or_create(name).state
            services["_navigation"] = self.navigator.status().get("state", "idle")
            services["_debug_navigation"] = self.debug_navigator.status().get(
                "state", "idle"
            )
            self._send_json({"ok": True, "services": services})
            return

        if path == "/api/config":
            self._send_json(
                {
                    "ok": True,
                    "visualization": self.visualization,
                    "debug_navigation": {
                        "allowed_profiles": self.debug_navigator.config[
                            "allowed_profiles"
                        ],
                        "default_profile": self.debug_navigator.config[
                            "default_profile"
                        ],
                        "waypoint_files": self.debug_navigator.config.get(
                            "waypoint_files", {}
                        ),
                        "defaults": self.debug_navigator.config.get("defaults", {}),
                    },
                }
            )
            return

        if path == "/api/profiles":
            result = []
            for name, cfg in self.profiles.items():
                proc = self.manager.get_or_create(name)
                result.append(
                    {
                        "name": name,
                        "description": cfg.get("description", ""),
                        "state": proc.state,
                    }
                )
            self._send_json({"profiles": result})
            return

        parts = path.split("/")

        if (
            len(parts) == 5
            and parts[1] == "api"
            and parts[2] == "profiles"
            and parts[4] == "status"
        ):
            name = parts[3]
            if name not in self.profiles:
                self._send_json(
                    {"ok": False, "error": f"Unknown profile: {name}"}, 404
                )
                return
            self._send_json(self.manager.get_or_create(name).status())
            return

        if (
            len(parts) == 5
            and parts[1] == "api"
            and parts[2] == "profiles"
            and parts[4] == "logs"
        ):
            name = parts[3]
            if name not in self.profiles:
                self._send_json(
                    {"ok": False, "error": f"Unknown profile: {name}"}, 404
                )
                return
            tail = int(query.get("tail", "100"))
            self._send_json({"lines": self.manager.get_or_create(name).logs(tail=tail)})
            return

        if path == "/api/waypoints":
            requested_profile = query.get("profile", "")
            wp_file = ""
            if requested_profile:
                wp_file = self.debug_navigator.config.get(
                    "waypoint_files", {}
                ).get(requested_profile, "")
            if not wp_file:
                wp_file = self.navigator.default_waypoint_file
            if not wp_file:
                self._send_json(
                    {"ok": False, "error": "no waypoint_file configured"}, 500
                )
                return
            try:
                waypoints = load_waypoints(Path(wp_file))
            except FileNotFoundError:
                self._send_json(
                    {"ok": False, "error": f"waypoint file not found: {wp_file}"},
                    404,
                )
                return
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, 500)
                return

            trimmed = [
                {
                    "id": wp["id"],
                    "name": wp["name"],
                    "frame_id": wp["frame_id"],
                    "x": wp["position"]["x"],
                    "y": wp["position"]["y"],
                    "yaw_deg": round(math.degrees(wp["yaw"]), 1),
                }
                for wp in waypoints
            ]
            self._send_json({"ok": True, "waypoint_file": wp_file, "waypoints": trimmed})
            return

        if path == "/api/nav/status":
            self._send_json({"ok": True, **self.navigator.status()})
            return

        if path == "/api/nav/logs":
            tail = int(query.get("tail", "100"))
            self._send_json({"ok": True, "lines": self.navigator.logs(tail=tail)})
            return

        if path == "/api/debug/status":
            self._send_json({"ok": True, **self.debug_navigator.status()})
            return

        if path == "/api/debug/logs":
            tail = int(query.get("tail", "100"))
            self._send_json(
                {"ok": True, "lines": self.debug_navigator.logs(tail=tail)}
            )
            return

        self._send_json({"ok": False, "error": "Not found"}, 404)

    # --- POST routes -----------------------------------------------------

    def do_POST(self):
        path = self.path.split("?")[0].rstrip("/")
        parts = path.split("/")

        if (
            len(parts) == 5
            and parts[1] == "api"
            and parts[2] == "profiles"
            and parts[4] == "start"
        ):
            name = parts[3]
            if name not in self.profiles:
                self._send_json(
                    {"ok": False, "error": f"Unknown profile: {name}"}, 404
                )
                return
            proc = self.manager.get_or_create(name)
            result = proc.start()
            status_code = 200 if result.get("ok") else 409
            self._send_json(result, status_code)
            return

        if (
            len(parts) == 5
            and parts[1] == "api"
            and parts[2] == "profiles"
            and parts[4] == "stop"
        ):
            name = parts[3]
            if name not in self.profiles:
                self._send_json(
                    {"ok": False, "error": f"Unknown profile: {name}"}, 404
                )
                return
            # Cancel any in-flight POI navigation first so we never outlive
            # the navigation stack we're about to tear down.
            self.navigator.cancel(reason=f"stack '{name}' stopping")
            result = self.manager.get_or_create(name).stop()
            self._send_json(result)
            return

        if path == "/api/nav/goto":
            body = self._read_body()
            target_id = str(body.get("target_id", "")).strip()
            waypoint_file = body.get("waypoint_file")
            if not target_id:
                self._send_json(
                    {"ok": False, "error": "target_id is required"}, 400
                )
                return
            result = self.navigator.start(
                target_id, waypoint_file=waypoint_file
            )
            status_code = 200 if result.get("ok") else 409
            self._send_json(result, status_code)
            return

        if path == "/api/nav/cancel":
            result = self.navigator.cancel(reason="user cancel")
            self._send_json(result)
            return

        if path == "/api/nav/estop":
            # Nuclear E-STOP: kills every running profile AND publishes
            # Twist(0) to /cmd_vel in parallel. See NavigationDispatcher.estop.
            self.debug_navigator.cancel(reason="E-STOP")
            result = self.navigator.estop(stack_manager=self.manager)
            status_code = 200 if result.get("ok") else 500
            self._send_json(result, status_code)
            return

        if path == "/api/debug/start":
            body = self._read_body()
            result = self.debug_navigator.start(body)
            status_code = 200 if result.get("ok") else 409
            self._send_json(result, status_code)
            return

        if path == "/api/debug/cancel":
            result = self.debug_navigator.cancel(reason="user cancel")
            self._send_json(result)
            return

        self._send_json({"ok": False, "error": "Not found"}, 404)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="GO2W Navigation Launch Manager")
    parser.add_argument(
        "--host", default="0.0.0.0", help="Listen address (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8080, help="Listen port (default: 8080)"
    )
    parser.add_argument(
        "--profiles",
        type=Path,
        default=DEFAULT_PROFILES_FILE,
        help="Path to profiles.yaml",
    )
    parser.add_argument(
        "--web-dir",
        type=Path,
        default=Path(__file__).parent / "web",
        help="Directory containing static web files",
    )
    parser.add_argument(
        "--waypoint-file",
        type=Path,
        default=None,
        help="Override the default waypoint YAML used for POI dispatch",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="[launch_manager] %(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    loaded = load_profiles(args.profiles)
    profiles = loaded["profiles"]
    nav_cfg = loaded["navigation"]
    debug_nav_cfg = loaded["debug_navigation"]
    visualization_cfg = loaded["visualization"]
    logger.info(
        "Loaded %d profiles: %s", len(profiles), ", ".join(profiles.keys())
    )

    waypoint_file = (
        str(args.waypoint_file) if args.waypoint_file else nav_cfg["waypoint_file"]
    )
    if not waypoint_file:
        logger.warning(
            "No waypoint_file configured; /api/waypoints and POI dispatch "
            "will return errors until one is set."
        )
    else:
        logger.info("Default waypoint_file: %s", waypoint_file)

    manager = ProcessManager(profiles)
    navigator = NavigationDispatcher(
        command_template=nav_cfg["navigate_command"],
        default_waypoint_file=waypoint_file,
        estop_command=nav_cfg.get("estop_command", DEFAULT_ESTOP_COMMAND),
    )
    debug_navigator = DebugBagDispatcher(debug_nav_cfg)
    web_dir = args.web_dir

    class Handler(LaunchManagerHandler):
        pass

    Handler.manager = manager
    Handler.profiles = profiles
    Handler.navigator = navigator
    Handler.debug_navigator = debug_navigator
    Handler.visualization = visualization_cfg

    _orig_do_GET = Handler.do_GET

    def do_GET_with_static(self):
        path = self.path.split("?")[0]
        if path.startswith("/api/"):
            return _orig_do_GET(self)

        if path == "/" or path == "":
            path = "/index.html"

        file_path = web_dir / path.lstrip("/")
        if file_path.is_file():
            content_type = "text/html; charset=utf-8"
            if path.endswith(".js"):
                content_type = "application/javascript; charset=utf-8"
            elif path.endswith(".css"):
                content_type = "text/css; charset=utf-8"
            elif path.endswith(".json"):
                content_type = "application/json; charset=utf-8"
            elif path.endswith(".png"):
                content_type = "image/png"
            elif path.endswith(".svg"):
                content_type = "image/svg+xml"

            data = file_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(data)
        else:
            self._send_json({"ok": False, "error": "Not found"}, 404)

    Handler.do_GET = do_GET_with_static

    server = HTTPServer((args.host, args.port), Handler)
    logger.info("Listening on http://%s:%d", args.host, args.port)
    logger.info("Web UI:   http://%s:%d/", args.host, args.port)
    logger.info("Health:   http://%s:%d/api/health", args.host, args.port)
    logger.info("POIs:     http://%s:%d/api/waypoints", args.host, args.port)

    def shutdown_handler(signum, frame):
        logger.info("Shutting down...")
        debug_navigator.shutdown()
        navigator.shutdown()
        manager.stop_all()
        threading.Thread(
            target=server.shutdown,
            name="launch-manager-shutdown",
            daemon=True,
        ).start()

    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        debug_navigator.shutdown()
        navigator.shutdown()
        manager.stop_all()
        server.server_close()


if __name__ == "__main__":
    main()
