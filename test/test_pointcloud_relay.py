from pathlib import Path
import importlib.util

from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_pointcloud_relay_module():
    module_path = PACKAGE_ROOT / "scripts/pointcloud_relay.py"
    spec = importlib.util.spec_from_file_location("pointcloud_relay", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_prepare_pointcloud_for_relay_preserves_stamp_by_default():
    relay = _load_pointcloud_relay_module()
    msg = PointCloud2()
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = 20
    now = Time(sec=30, nanosec=40)

    output = relay.prepare_pointcloud_for_relay(
        msg,
        now_stamp=now,
        restamp_output=False,
    )

    assert output is msg
    assert output.header.stamp.sec == 10
    assert output.header.stamp.nanosec == 20


def test_prepare_pointcloud_for_relay_can_restamp_stale_clouds():
    relay = _load_pointcloud_relay_module()
    msg = PointCloud2()
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = 20
    now = Time(sec=30, nanosec=40)

    output = relay.prepare_pointcloud_for_relay(
        msg,
        now_stamp=now,
        restamp_output=True,
    )

    assert output is msg
    assert output.header.stamp.sec == 30
    assert output.header.stamp.nanosec == 40
