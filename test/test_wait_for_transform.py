import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


REPO_ROOT = Path("/home/unitree/ros_ws")
WAIT_FOR_TRANSFORM_PATH = (
    REPO_ROOT / "src/go2w_real/scripts/wait_for_transform.py"
)


class FakeParameter:
    def __init__(self, value):
        self.value = value


def _install_ros_stubs(monkeypatch):
    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.ok = lambda: True
    fake_rclpy.init = lambda args=None: None
    fake_rclpy.shutdown = lambda: None
    fake_rclpy.spin_once = lambda node, timeout_sec=None: None

    fake_rclpy_duration = ModuleType("rclpy.duration")
    fake_rclpy_duration.Duration = lambda seconds=0.0: seconds

    fake_rclpy_time = ModuleType("rclpy.time")
    fake_rclpy_time.Time = lambda: None

    class FakeNode:
        parameter_overrides = {}

        def __init__(self, name):
            self.name = name
            self._parameters = {}

        def declare_parameter(self, name, default_value):
            value = self.parameter_overrides.get(name, default_value)
            parameter = FakeParameter(value)
            self._parameters[name] = parameter
            return parameter

        def get_parameter(self, name):
            return self._parameters[name]

        def get_logger(self):
            return SimpleNamespace(info=lambda message: None, error=lambda message: None)

        def destroy_node(self):
            return None

    fake_rclpy_node = ModuleType("rclpy.node")
    fake_rclpy_node.Node = FakeNode

    fake_tf2_ros = ModuleType("tf2_ros")
    fake_tf2_ros.Buffer = lambda: SimpleNamespace(
        can_transform=lambda *args, **kwargs: False
    )
    fake_tf2_ros.TransformListener = lambda buffer, node: SimpleNamespace()

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.duration", fake_rclpy_duration)
    monkeypatch.setitem(sys.modules, "rclpy.time", fake_rclpy_time)
    monkeypatch.setitem(sys.modules, "rclpy.node", fake_rclpy_node)
    monkeypatch.setitem(sys.modules, "tf2_ros", fake_tf2_ros)

    return FakeNode


def _load_wait_for_transform_module(monkeypatch):
    fake_node_cls = _install_ros_stubs(monkeypatch)
    spec = importlib.util.spec_from_file_location(
        "go2w_wait_for_transform_script",
        WAIT_FOR_TRANSFORM_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module, fake_node_cls


def test_parameter_to_float_accepts_int(monkeypatch):
    wait_for_transform, _ = _load_wait_for_transform_module(monkeypatch)

    assert wait_for_transform._parameter_to_float(FakeParameter(30), "timeout_sec") == 30.0


def test_parameter_to_float_accepts_float(monkeypatch):
    wait_for_transform, _ = _load_wait_for_transform_module(monkeypatch)

    assert (
        wait_for_transform._parameter_to_float(FakeParameter(0.25), "poll_period")
        == 0.25
    )


def test_parameter_to_float_rejects_non_numeric(monkeypatch):
    wait_for_transform, _ = _load_wait_for_transform_module(monkeypatch)

    with pytest.raises(ValueError, match="must be numeric"):
        wait_for_transform._parameter_to_float(FakeParameter("later"), "timeout_sec")


def test_wait_for_transform_initializes_numeric_overrides_from_int_params(monkeypatch):
    wait_for_transform, fake_node_cls = _load_wait_for_transform_module(monkeypatch)
    fake_node_cls.parameter_overrides = {
        "target_frame": "map",
        "source_frame": "base",
        "timeout_sec": 30,
        "poll_period": 1,
        "log_period": 5,
    }

    try:
        node = wait_for_transform.WaitForTransform()
    finally:
        fake_node_cls.parameter_overrides = {}

    assert node.target_frame == "map"
    assert node.source_frame == "base"
    assert node.timeout_sec == 30.0
    assert node.poll_period == 1.0
    assert node.log_period == 5.0

