from pathlib import Path
import sys

import pytest
import yaml


REPO_ROOT = Path("/home/unitree/ros_ws")
PACKAGE_ROOT = REPO_ROOT / "src/go2w_real"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from go2w_real.imu_fusion_bridges import (  # noqa: E402
    adapt_utlidar_imu_sample,
    convert_lowstate_imu_sample,
)


def test_lowstate_conversion_reorders_quaternion_and_subtracts_gyro_bias():
    sample = convert_lowstate_imu_sample(
        quaternion_wxyz=[0.5, 0.1, 0.2, 0.3],
        gyroscope=[0.01, -0.02, -0.004],
        accelerometer=[1.0, 2.0, 3.0],
        gyro_bias=[0.001, -0.002, -0.009],
        frame_id="imu",
        orientation_covariance_yaw=0.05,
        angular_velocity_covariance_z=0.04,
        linear_acceleration_covariance=99.0,
    )

    assert sample["frame_id"] == "imu"
    assert sample["orientation_xyzw"] == [0.1, 0.2, 0.3, 0.5]
    assert sample["angular_velocity"] == pytest.approx([0.009, -0.018, 0.005])
    assert sample["linear_acceleration"] == [1.0, 2.0, 3.0]
    assert sample["orientation_covariance"][8] == 0.05
    assert sample["angular_velocity_covariance"][8] == 0.04
    assert sample["linear_acceleration_covariance"][0] == 99.0


def test_utlidar_adapter_flips_yaw_rate_into_base_frame():
    sample = adapt_utlidar_imu_sample(
        orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
        angular_velocity=[0.01, 0.02, -0.3],
        linear_acceleration=[0.0, 0.0, 9.8],
        axis_signs=[1.0, 1.0, -1.0],
        frame_id="imu",
        orientation_covariance_yaw=0.02,
        angular_velocity_covariance_z=0.03,
        linear_acceleration_covariance=100.0,
    )

    assert sample["frame_id"] == "imu"
    assert sample["angular_velocity"] == [0.01, 0.02, 0.3]
    assert sample["linear_acceleration"] == [0.0, 0.0, -9.8]
    assert sample["orientation_covariance"][8] == 0.02
    assert sample["angular_velocity_covariance"][8] == 0.03


def test_lowstate_rf2o_fusion_config_uses_rf2o_and_corrected_utlidar_imu():
    cfg = yaml.safe_load(
        (PACKAGE_ROOT / "config/odom_fusion_lowstate_rf2o.yaml").read_text()
    )
    params = cfg["ekf_filter_node_odom"]["ros__parameters"]

    assert params["odom0"] == "/rf2o/odom"
    assert params["imu0"] == "/lowstate_imu"
    assert params["imu1"] == "/utlidar_imu_base"
    assert params["two_d_mode"] is True
    assert params["publish_tf"] is True
    assert params["odom0_config"] == [
        True,
        True,
        False,
        False,
        False,
        True,
        True,
        True,
        False,
        False,
        False,
        True,
        False,
        False,
        False,
    ]
    assert params["imu0_config"] == [
        False,
        False,
        False,
        False,
        False,
        True,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
    ]
    assert params["imu1_config"] == [
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        True,
        False,
        False,
        False,
    ]


def test_slam_launch_runs_rf2o_as_fusion_input_and_starts_imu_adapters():
    launch_text = (PACKAGE_ROOT / "launch/slam_rf2o.launch.py").read_text()

    assert "rf2o_fusion_input_node" in launch_text
    assert '"odom_topic": "/rf2o/odom"' in launch_text
    assert '"publish_tf": False' in launch_text
    assert "utlidar_imu_adapter.py" in launch_text
    assert "lowstate_imu_bridge.py" in launch_text
    assert "odom_fusion_lowstate_rf2o.yaml" in launch_text


def test_cmakelists_installs_imu_bridge_scripts_and_registers_tests():
    cmake_text = (PACKAGE_ROOT / "CMakeLists.txt").read_text()

    assert "scripts/lowstate_imu_bridge.py" in cmake_text
    assert "scripts/utlidar_imu_adapter.py" in cmake_text
    assert "test_imu_fusion_bridges" in cmake_text
