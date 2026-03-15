#!/usr/bin/env python3
"""
GO2W Bridge Node: bridges ROS2 cmd_vel to Unitree SDK2 SportClient,
and publishes odometry + TF from SportModeState.

Architecture: Uses Python multiprocessing to isolate SDK2 (CycloneDDS domain)
from rclpy (CycloneDDS domain) — they CANNOT coexist in the same process.

  Child process:  SDK2 ChannelFactory + SportClient + SportModeState subscriber
  Parent process: rclpy Node (cmd_vel sub, odom/tf/imu pub)
  IPC:            multiprocessing.Queue for odom state (child→parent)
                  multiprocessing.Queue for cmd_vel/control (parent→child)

Subscribes:
  /cmd_vel (geometry_msgs/Twist) - velocity commands from Nav2 or teleop
  /cmd_control (std_msgs/String) - mode commands: stand_up, stand_down, damp, recovery

Publishes:
  /odom (nav_msgs/Odometry) - robot odometry from SportModeState
  /tf (odom -> base) - odometry transform
  /imu/data (sensor_msgs/Imu) - IMU data from SportModeState
"""

import math
import multiprocessing
import os
import signal
import sys
import time

LOCAL_UNITREE_SDK2_PYTHON = os.path.expanduser("~/ros2_ws/src/unitree_sdk2_python")
if os.path.isdir(LOCAL_UNITREE_SDK2_PYTHON) and LOCAL_UNITREE_SDK2_PYTHON not in sys.path:
    sys.path.insert(0, LOCAL_UNITREE_SDK2_PYTHON)


DEFAULT_STATE_TOPICS = (
    "rt/lf/sportmodestate",  # GO2W
    "rt/sportmodestate",     # GO2 / older examples
)
NO_STATE_WARN_PERIOD = 5.0


def _normalize_network_interface(net_iface: str):
    """Return None when SDK2 should auto-select the network interface."""
    if net_iface is None:
        return None

    normalized = net_iface.strip()
    if not normalized or normalized.lower() == "auto":
        return None

    return normalized


# ═══════════════════════════════════════════════════════════════════════
# SDK2 child process — runs in its own CycloneDDS domain
# ═══════════════════════════════════════════════════════════════════════

def _sdk2_worker(
    net_iface: str,
    state_queue: multiprocessing.Queue,
    cmd_queue: multiprocessing.Queue,
    shutdown_event: multiprocessing.Event,
):
    """
    Runs in a child process. Initializes SDK2, subscribes to SportModeState,
    and forwards commands from cmd_queue to SportClient.
    """
    from unitree_sdk2py.core.channel import (
        ChannelSubscriber,
        ChannelFactoryInitialize,
    )
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    # Ignore SIGINT in child — parent handles it
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    resolved_iface = _normalize_network_interface(net_iface)
    iface_label = resolved_iface or "auto"
    print(
        f"[go2w_bridge] SDK2 worker initializing on interface: {iface_label}",
        flush=True,
    )

    try:
        ChannelFactoryInitialize(0, resolved_iface)
    except Exception as exc:
        print(
            f"[go2w_bridge] SDK2 ChannelFactory init failed on interface "
            f"{iface_label}: {exc}",
            flush=True,
        )
        return

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    latest_state = None
    first_state_topic = None
    state_lock = multiprocessing.Lock()

    def _state_cb(msg: SportModeState_, topic_name: str):
        nonlocal first_state_topic, latest_state
        with state_lock:
            latest_state = msg
            if first_state_topic is None:
                first_state_topic = topic_name
                print(
                    f"[go2w_bridge] Received first SportModeState on "
                    f"{topic_name}",
                    flush=True,
                )

    print(
        "[go2w_bridge] Subscribing SportModeState topics: "
        + ", ".join(DEFAULT_STATE_TOPICS),
        flush=True,
    )
    state_subs = []
    for topic_name in DEFAULT_STATE_TOPICS:
        sub = ChannelSubscriber(topic_name, SportModeState_)
        sub.Init(lambda msg, topic_name=topic_name: _state_cb(msg, topic_name), 10)
        state_subs.append(sub)

    last_cmd_time = time.time()
    CMD_TIMEOUT = 0.5
    next_no_state_warn = time.time() + NO_STATE_WARN_PERIOD

    while not shutdown_event.is_set():
        # Forward odom state to parent (non-blocking, drop old if full)
        with state_lock:
            st = latest_state

        if st is not None:
            odom_data = {
                "pos": [float(st.position[0]), float(st.position[1]), float(st.position[2])],
                "rpy": [float(st.imu_state.rpy[0]), float(st.imu_state.rpy[1]), float(st.imu_state.rpy[2])],
                "vel": [float(st.velocity[0]), float(st.velocity[1]), float(st.velocity[2])],
                "yaw_speed": float(st.yaw_speed),
                "gyro": [float(st.imu_state.gyroscope[0]), float(st.imu_state.gyroscope[1]), float(st.imu_state.gyroscope[2])],
                "accel": [float(st.imu_state.accelerometer[0]), float(st.imu_state.accelerometer[1]), float(st.imu_state.accelerometer[2])],
            }
            try:
                # Drop stale data — keep only latest
                while not state_queue.empty():
                    try:
                        state_queue.get_nowait()
                    except Exception:
                        break
                state_queue.put_nowait(odom_data)
            except Exception:
                pass
        elif time.time() >= next_no_state_warn:
            print(
                "[go2w_bridge] Waiting for SportModeState; /odom and odom->base "
                "TF will not be published until a state packet arrives. "
                "Checked topics: "
                + ", ".join(DEFAULT_STATE_TOPICS),
                flush=True,
            )
            next_no_state_warn = time.time() + NO_STATE_WARN_PERIOD

        # Process incoming commands from parent
        while True:
            try:
                cmd = cmd_queue.get_nowait()
            except Exception:
                break

            cmd_type = cmd.get("type")
            if cmd_type == "move":
                sport_client.Move(cmd["vx"], cmd["vy"], cmd["vyaw"])
                last_cmd_time = time.time()
            elif cmd_type == "control":
                action = cmd["action"]
                if action == "stand_up":
                    sport_client.StandUp()
                elif action == "stand_down":
                    sport_client.StandDown()
                elif action == "damp":
                    sport_client.Damp()
                elif action == "recovery":
                    sport_client.RecoveryStand()
                elif action == "balance":
                    sport_client.BalanceStand()
                elif action == "stop":
                    sport_client.StopMove()

        # Safety: stop if no cmd_vel received within timeout
        if time.time() - last_cmd_time > CMD_TIMEOUT:
            sport_client.Move(0.0, 0.0, 0.0)

        time.sleep(0.02)  # ~50 Hz loop

    # Clean shutdown
    sport_client.StopMove()


# ═══════════════════════════════════════════════════════════════════════
# ROS2 parent process
# ═══════════════════════════════════════════════════════════════════════

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert euler angles to geometry_msgs Quaternion."""
    from geometry_msgs.msg import Quaternion
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor

    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    import tf2_ros

    # ── Parse network_interface from args BEFORE rclpy.init ───────────
    # We need to fork SDK2 worker before rclpy touches CycloneDDS.
    # Default to auto-detect; override via --ros-args -p network_interface:=xxx
    net_iface = ""
    if args is None:
        import sys as _sys
        _argv = _sys.argv[1:]
    else:
        _argv = list(args)
    for i, a in enumerate(_argv):
        if "network_interface:=" in a:
            net_iface = a.split(":=", 1)[1]
            break

    # IPC queues
    state_queue = multiprocessing.Queue(maxsize=2)
    cmd_queue = multiprocessing.Queue(maxsize=50)
    shutdown_event = multiprocessing.Event()

    # Start SDK2 child process BEFORE rclpy.init() to avoid
    # CycloneDDS domain conflict (fork inherits parent's DDS state)
    iface_label = _normalize_network_interface(net_iface) or "auto"
    print(
        f"[go2w_bridge] Starting SDK2 worker on interface: {iface_label}",
        flush=True,
    )
    sdk_proc = multiprocessing.Process(
        target=_sdk2_worker,
        args=(net_iface, state_queue, cmd_queue, shutdown_event),
        daemon=True,
    )
    sdk_proc.start()
    print(f"[go2w_bridge] SDK2 worker started (PID {sdk_proc.pid})", flush=True)

    # Now safe to initialize ROS2 (creates its own CycloneDDS domain)
    rclpy.init(args=args)

    node = Node("go2w_bridge")

    # Parameters (for logging; net_iface already parsed above)
    node.declare_parameter("network_interface", net_iface)
    node.declare_parameter("cmd_vel_timeout", 0.5)
    node.declare_parameter("odom_frame", "odom")
    node.declare_parameter("base_frame", "base")
    node.declare_parameter("publish_odom_tf", True)

    odom_frame = node.get_parameter("odom_frame").value
    base_frame = node.get_parameter("base_frame").value
    publish_odom_tf = node.get_parameter("publish_odom_tf").value

    node.get_logger().info(
        f"SDK2 worker PID {sdk_proc.pid}, interface: {iface_label}"
    )

    # Publishers
    odom_pub = node.create_publisher(Odometry, "odom", 10)
    imu_pub = node.create_publisher(Imu, "imu/data", 10)
    tf_broadcaster = tf2_ros.TransformBroadcaster(node)

    # Callback groups
    cmd_cb_group = MutuallyExclusiveCallbackGroup()
    odom_cb_group = MutuallyExclusiveCallbackGroup()

    # cmd_vel subscriber
    def _cmd_vel_cb(msg: Twist):
        try:
            cmd_queue.put_nowait({
                "type": "move",
                "vx": msg.linear.x,
                "vy": msg.linear.y,
                "vyaw": msg.angular.z,
            })
        except Exception:
            pass

    node.create_subscription(
        Twist, "cmd_vel", _cmd_vel_cb, 10, callback_group=cmd_cb_group
    )

    # cmd_control subscriber
    def _cmd_control_cb(msg: String):
        action = msg.data.strip().lower()
        node.get_logger().info(f"Control command: {action}")
        try:
            cmd_queue.put_nowait({"type": "control", "action": action})
        except Exception:
            pass

    node.create_subscription(
        String, "cmd_control", _cmd_control_cb, 10, callback_group=cmd_cb_group
    )

    # Odom timer: read state from SDK2 child and publish
    next_no_state_warn = time.monotonic() + NO_STATE_WARN_PERIOD
    sdk_proc_dead_logged = False

    def _odom_timer_cb():
        nonlocal next_no_state_warn, sdk_proc_dead_logged
        if not sdk_proc.is_alive():
            if not sdk_proc_dead_logged:
                node.get_logger().error(
                    "SDK2 worker exited; /odom and odom->base TF are unavailable."
                )
                sdk_proc_dead_logged = True
            return

        try:
            odom_data = state_queue.get_nowait()
        except Exception:
            if time.monotonic() >= next_no_state_warn:
                node.get_logger().warn(
                    "Still waiting for SportModeState; /odom and odom->base TF "
                    "are not being published yet."
                )
                next_no_state_warn = time.monotonic() + NO_STATE_WARN_PERIOD
            return

        next_no_state_warn = time.monotonic() + NO_STATE_WARN_PERIOD

        now = node.get_clock().now().to_msg()

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame

        odom.pose.pose.position.x = odom_data["pos"][0]
        odom.pose.pose.position.y = odom_data["pos"][1]
        odom.pose.pose.position.z = odom_data["pos"][2]

        roll, pitch, yaw = odom_data["rpy"]
        odom.pose.pose.orientation = euler_to_quaternion(roll, pitch, yaw)

        odom.twist.twist.linear.x = odom_data["vel"][0]
        odom.twist.twist.linear.y = odom_data["vel"][1]
        odom.twist.twist.linear.z = odom_data["vel"][2]
        odom.twist.twist.angular.z = odom_data["yaw_speed"]

        odom_pub.publish(odom)

        # TF: odom -> base
        if publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = odom_frame
            t.child_frame_id = base_frame
            t.transform.translation.x = odom_data["pos"][0]
            t.transform.translation.y = odom_data["pos"][1]
            t.transform.translation.z = odom_data["pos"][2]
            t.transform.rotation = odom.pose.pose.orientation
            tf_broadcaster.sendTransform(t)

        # IMU
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation = euler_to_quaternion(roll, pitch, yaw)
        imu_msg.angular_velocity.x = odom_data["gyro"][0]
        imu_msg.angular_velocity.y = odom_data["gyro"][1]
        imu_msg.angular_velocity.z = odom_data["gyro"][2]
        imu_msg.linear_acceleration.x = odom_data["accel"][0]
        imu_msg.linear_acceleration.y = odom_data["accel"][1]
        imu_msg.linear_acceleration.z = odom_data["accel"][2]
        imu_pub.publish(imu_msg)

    node.create_timer(0.02, _odom_timer_cb, callback_group=odom_cb_group)

    node.get_logger().info("GO2W Bridge ready (multiprocessing). Waiting for cmd_vel...")

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down, stopping SDK2 worker...")
        shutdown_event.set()
        sdk_proc.join(timeout=3.0)
        if sdk_proc.is_alive():
            sdk_proc.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
