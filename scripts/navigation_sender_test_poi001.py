#!/usr/bin/env python3
"""
Sender-side one-shot WebSocket test for GO2W navigation.

This script runs on another host in the same LAN, waits for the robot-side
navigation executor to connect, then automatically sends a NavigateTo command
to POI_001 and prints progress / arrived / error events.
"""

import argparse
import asyncio
import json
import sys
import time
import uuid

from websockets.legacy.server import serve


DEFAULT_BIND_HOST = "0.0.0.0"
DEFAULT_PORT = 8100
DEFAULT_PATH = "/ws/navigation/executor"
DEFAULT_TARGET_ID = "POI_001"
DEFAULT_SUB_ID = 1
DEFAULT_TIMEOUT = 180.0


def _print_help():
    print(
        "Usage: python3 navigation_sender_test_poi001.py [options]\n"
        "\n"
        "Options:\n"
        "  --bind-host HOST      Bind host, default: 0.0.0.0\n"
        "  --port PORT           Listen port, default: 8100\n"
        "  --path PATH           WebSocket path, default: /ws/navigation/executor\n"
        "  --target-id ID        Target POI id, default: POI_001\n"
        "  --sub-id N            Sub task id, default: 1\n"
        "  --timeout SEC         Total wait timeout, default: 180\n"
        "  --announce-ip IP      Sender host IP shown in instructions\n"
        "  -h, --help            Show this help message\n"
    )


def _parse_args(raw_args):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--bind-host", default=DEFAULT_BIND_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--path", default=DEFAULT_PATH)
    parser.add_argument("--target-id", default=DEFAULT_TARGET_ID)
    parser.add_argument("--sub-id", type=int, default=DEFAULT_SUB_ID)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--announce-ip", default=None)
    parser.add_argument("-h", "--help", action="store_true")
    args = parser.parse_args(raw_args)

    if args.help:
        _print_help()
        raise SystemExit(0)
    if args.port <= 0 or args.port > 65535:
        raise SystemExit("--port must be within 1..65535")
    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be > 0")
    if not args.path.startswith("/"):
        raise SystemExit("--path must start with '/'")

    return args


class Poi001TestServer:
    def __init__(self, config):
        self.config = config
        self.request_id = f"req_test_{uuid.uuid4().hex[:8]}"
        self.result_future = None
        self._active_client = None
        self._done = False

    async def run(self):
        self.result_future = asyncio.get_running_loop().create_future()
        announce_ip = self.config.announce_ip or "<sender_ip>"
        print(
            f"[sender_test] starting server at "
            f"ws://{self.config.bind_host}:{self.config.port}{self.config.path}",
            flush=True,
        )
        print(
            "[sender_test] on the robot, run:\n"
            f"  ros2 run go2w_real navigation_executor.py "
            f"--server-uri ws://{announce_ip}:{self.config.port}{self.config.path}",
            flush=True,
        )
        print(
            f"[sender_test] waiting for robot connection, will auto-send "
            f"{self.config.target_id} after connect",
            flush=True,
        )

        async with serve(
            self._handler,
            self.config.bind_host,
            self.config.port,
            ping_interval=None,
        ):
            try:
                status, payload = await asyncio.wait_for(
                    self.result_future,
                    timeout=self.config.timeout,
                )
            except asyncio.TimeoutError:
                print(
                    f"[sender_test] timeout after {self.config.timeout:.1f}s",
                    flush=True,
                )
                return 1

        if status == "arrived":
            print("[sender_test] test succeeded: robot arrived", flush=True)
            return 0

        if status == "error":
            print(
                f"[sender_test] test failed: {payload.get('error_message', 'unknown error')}",
                flush=True,
            )
            return 2

        if status == "disconnected":
            print("[sender_test] test failed: robot disconnected", flush=True)
            return 3

        print(f"[sender_test] finished with unexpected status: {status}", flush=True)
        return 4

    async def _handler(self, websocket, path):
        if path != self.config.path:
            print(f"[sender_test] rejected unexpected path: {path}", flush=True)
            await websocket.close(code=1008, reason="invalid path")
            return

        if self._active_client is not None:
            print("[sender_test] rejected extra client connection", flush=True)
            await websocket.close(code=1013, reason="busy")
            return

        self._active_client = websocket
        peer = websocket.remote_address
        print(f"[sender_test] robot connected from: {peer}", flush=True)

        try:
            await self._send_navigate_to(websocket)
            async for raw_message in websocket:
                try:
                    data = json.loads(raw_message)
                except json.JSONDecodeError:
                    print(
                        f"[sender_test] ignored invalid JSON: {raw_message}",
                        flush=True,
                    )
                    continue

                if data.get("type") == "ping":
                    await self._send_json(websocket, {"type": "pong"})
                    print("[sender_test] recv ping -> sent pong", flush=True)
                    continue
                if data.get("type") == "pong":
                    print("[sender_test] recv pong", flush=True)
                    continue

                await self._handle_event(data)
                if self._done:
                    await websocket.close()
                    return
        finally:
            self._active_client = None
            if not self._done and not self.result_future.done():
                self.result_future.set_result(("disconnected", {}))

    async def _send_navigate_to(self, websocket):
        payload = {
            "action": "navigate_to",
            "request_id": self.request_id,
            "sub_id": self.config.sub_id,
            "target_id": self.config.target_id,
        }
        await self._send_json(websocket, payload)
        print(f"[sender_test] sent navigate_to: {payload}", flush=True)

    async def _send_json(self, websocket, payload):
        await websocket.send(json.dumps(payload, ensure_ascii=False))

    async def _handle_event(self, data):
        event_type = data.get("event_type")

        if event_type == "on_progress":
            remaining = data.get("remaining_distance")
            status = data.get("status")
            print(
                f"[sender_test] progress: status={status}, "
                f"remaining_distance={remaining}",
                flush=True,
            )
            return

        if event_type == "on_arrived":
            print(f"[sender_test] arrived event: {data}", flush=True)
            self._done = True
            if not self.result_future.done():
                self.result_future.set_result(("arrived", data))
            return

        if event_type == "on_error":
            print(f"[sender_test] error event: {data}", flush=True)
            self._done = True
            if not self.result_future.done():
                self.result_future.set_result(("error", data))
            return

        print(f"[sender_test] recv event: {data}", flush=True)


def main(args=None):
    raw_args = sys.argv[1:] if args is None else list(args)
    config = _parse_args(raw_args)

    start_time = time.monotonic()
    server = Poi001TestServer(config)
    try:
        exit_code = asyncio.run(server.run())
    except KeyboardInterrupt:
        print("[sender_test] interrupted by user", flush=True)
        exit_code = 130

    elapsed = time.monotonic() - start_time
    print(f"[sender_test] total elapsed: {elapsed:.1f}s", flush=True)
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
