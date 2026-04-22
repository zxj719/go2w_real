import importlib.util
from pathlib import Path


REPO_ROOT = Path("/home/unitree/ros_ws")
BRIDGE_PATH = REPO_ROOT / "src/go2w_real/scripts/go2w_bridge.py"


def _load_bridge_module():
    spec = importlib.util.spec_from_file_location("go2w_bridge", BRIDGE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_apply_angular_deadband_zeroes_small_yaw_rate():
    bridge = _load_bridge_module()

    assert bridge.apply_angular_deadband(0.03, 0.05) == 0.0
    assert bridge.apply_angular_deadband(-0.03, 0.05) == 0.0


def test_apply_angular_deadband_keeps_threshold_and_larger_yaw_rate():
    bridge = _load_bridge_module()

    assert bridge.apply_angular_deadband(0.05, 0.05) == 0.05
    assert bridge.apply_angular_deadband(-0.08, 0.05) == -0.08


def test_apply_angular_deadband_is_disabled_when_threshold_is_zero():
    bridge = _load_bridge_module()

    assert bridge.apply_angular_deadband(0.01, 0.0) == 0.01
    assert bridge.apply_angular_deadband(-0.01, -1.0) == -0.01


def test_bridge_script_defaults_angular_deadband_to_01():
    bridge_source = BRIDGE_PATH.read_text()

    assert 'node.declare_parameter("angular_deadband", 0.1)' in bridge_source
