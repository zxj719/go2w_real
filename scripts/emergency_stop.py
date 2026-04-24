#!/usr/bin/env python3
"""Emergency stop helper for the GO2W operator console.

Does two things, both best-effort and each with its own timeout so the
script always exits quickly:

  1. Cancel every active goal on /navigate_to_pose by calling the action's
     CancelGoal service. Sending an empty CancelGoal.Request (goal_info
     zero-initialized) asks the server to cancel all goals.
  2. Publish geometry_msgs/Twist(0) to /cmd_vel for a short hold window at
     20 Hz so the base actually decelerates to zero even if Nav2's controller
     is stopped mid-cycle or not currently running.

Intended to be invoked as a one-shot subprocess (no ROS state is persisted).

Defaults can be overridden on the command line:
  --cmd-vel-topic      default: /cmd_vel
  --action-name        default: /navigate_to_pose
  --hold-seconds       default: 1.5
  --rate-hz            default: 20.0
  --cancel-timeout     default: 1.0
"""
from __future__ import annotations

import argparse
import sys
import time


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="GO2W emergency stop helper")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--cancel-timeout", type=float, default=1.0)
    return parser.parse_args(argv)


def _cancel_all_nav_goals(node, action_name, timeout_sec):
    """Best-effort: ask the action server to cancel every goal.

    Uses the low-level CancelGoal service (`<action>/_action/cancel_goal`)
    because rclpy's ActionClient does not expose a public "cancel all"
    method and we do not have handles for goals sent by other processes
    (navigate_to_waypoint.py, navigation_executor.py, ...).
    """
    from action_msgs.srv import CancelGoal

    service_name = f"{action_name}/_action/cancel_goal"
    try:
        client = node.create_client(CancelGoal, service_name)
    except Exception as exc:
        print(f"[emergency_stop] could not create cancel client: {exc}", flush=True)
        return False

    if not client.wait_for_service(timeout_sec=timeout_sec):
        print(
            f"[emergency_stop] cancel service not available within "
            f"{timeout_sec:.1f}s: {service_name}",
            flush=True,
        )
        return False

    # An all-zero Request asks the server to cancel every tracked goal.
    request = CancelGoal.Request()
    future = client.call_async(request)

    import rclpy

    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        print("[emergency_stop] cancel call timed out", flush=True)
        return False

    try:
        response = future.result()
    except Exception as exc:
        print(f"[emergency_stop] cancel call failed: {exc}", flush=True)
        return False

    canceled = len(getattr(response, "goals_canceling", []) or [])
    print(
        f"[emergency_stop] requested cancel on {action_name}; "
        f"server reports {canceled} goals canceling",
        flush=True,
    )
    return True


def _hold_zero_velocity(node, topic, hold_seconds, rate_hz):
    from geometry_msgs.msg import Twist

    publisher = node.create_publisher(Twist, topic, 10)
    zero = Twist()
    period = 1.0 / max(rate_hz, 1.0)

    # Pump a few messages out the door before we start the main loop so the
    # DDS discovery has time to find subscribers.
    for _ in range(3):
        publisher.publish(zero)
        time.sleep(0.02)

    deadline = time.monotonic() + hold_seconds
    count = 3
    while time.monotonic() < deadline:
        publisher.publish(zero)
        count += 1
        time.sleep(period)

    print(
        f"[emergency_stop] published {count} zero-velocity messages to {topic} "
        f"over {hold_seconds:.2f}s",
        flush=True,
    )


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])

    try:
        import rclpy
    except ImportError as exc:
        print(f"[emergency_stop] rclpy import failed: {exc}", flush=True)
        return 1

    try:
        rclpy.init()
    except Exception as exc:
        print(f"[emergency_stop] rclpy.init failed: {exc}", flush=True)
        return 1

    node = rclpy.create_node("emergency_stop")
    try:
        _cancel_all_nav_goals(node, args.action_name, args.cancel_timeout)
        _hold_zero_velocity(node, args.cmd_vel_topic, args.hold_seconds, args.rate_hz)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    print("[emergency_stop] done", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
