#!/usr/bin/env python3
"""
One-shot Unitree SDK2 motion command helper.

Runs as a standalone process so SDK2 does not share a process with rclpy.
"""

import argparse
import os
import sys
import time


SDK_SEARCH_PATHS = (
    os.path.expanduser("~/ros_ws/src/unitree_sdk2_python"),
    os.path.expanduser("~/ros2_ws/src/unitree_sdk2_python"),
)

for sdk_path in SDK_SEARCH_PATHS:
    if os.path.isdir(sdk_path) and sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)


def _normalize_network_interface(net_iface: str):
    if net_iface is None:
        return None

    normalized = net_iface.strip()
    if not normalized or normalized.lower() == "auto":
        return None

    return normalized


def _build_parser():
    parser = argparse.ArgumentParser(description="Send one Unitree SportClient command.")
    parser.add_argument(
        "--network-interface",
        default="eth0",
        help="Network interface connected to GO2/GO2W. Use 'auto' to let SDK2 choose.",
    )
    parser.add_argument(
        "--action",
        default="stand_up",
        choices=("stand_up", "stand_down", "damp", "recovery", "balance", "stop"),
        help="SportClient action to send.",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=1.0,
        help="Wait time after SDK init before sending the command.",
    )
    parser.add_argument(
        "--post-seconds",
        type=float,
        default=2.0,
        help="Wait time after sending the command before exit.",
    )
    return parser


def main():
    args = _build_parser().parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient
    except Exception as exc:
        print(
            "[go2_motion_command] Failed to import unitree_sdk2py. "
            "Make sure unitree_sdk2_python is installed or present in the workspace. "
            f"Additional search paths: {SDK_SEARCH_PATHS}. Error: {exc}",
            file=sys.stderr,
            flush=True,
        )
        return 1

    resolved_iface = _normalize_network_interface(args.network_interface)
    iface_label = resolved_iface or "auto"
    print(
        f"[go2_motion_command] Initializing SDK2 on interface: {iface_label}",
        flush=True,
    )

    ChannelFactoryInitialize(0, resolved_iface)

    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()

    if args.settle_seconds > 0.0:
        time.sleep(args.settle_seconds)

    action_map = {
        "stand_up": client.StandUp,
        "stand_down": client.StandDown,
        "damp": client.Damp,
        "recovery": client.RecoveryStand,
        "balance": client.BalanceStand,
        "stop": client.StopMove,
    }

    print(f"[go2_motion_command] Sending action: {args.action}", flush=True)
    action_map[args.action]()

    if args.post_seconds > 0.0:
        time.sleep(args.post_seconds)

    print("[go2_motion_command] Command finished.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
