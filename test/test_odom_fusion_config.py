from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


REPO_ROOT = Path("/home/unitree/ros_ws")
PACKAGE_ROOT = REPO_ROOT / "src/go2w_real"


def test_launch_defaults_to_robot_localization_odom_fusion():
    launch_text = (PACKAGE_ROOT / "launch/slam_rf2o.launch.py").read_text()
    match = re.search(
        r'DeclareLaunchArgument\(\s*"use_odom_fusion",(?P<body>.*?)\n\s*\)',
        launch_text,
        re.DOTALL,
    )

    assert match is not None
    assert 'default_value="true"' in match.group("body")
    assert "robot_localization" in launch_text
    assert 'executable="ekf_node"' in launch_text
    assert '("odometry/filtered", "/odom")' in launch_text
    assert "odom_fusion_params_file" in launch_text


def test_odom_fusion_config_prefers_sport_odom_and_imu():
    cfg = yaml.safe_load(
        (PACKAGE_ROOT / "config/odom_fusion_params.yaml").read_text()
    )

    params = cfg["ekf_filter_node_odom"]["ros__parameters"]

    assert params["two_d_mode"] is True
    assert params["world_frame"] == "odom"
    assert params["odom0"] == "/sport_odom"
    assert params["imu0"] == "/sport_imu"
    assert params["odom0_config"] == [
        True,
        True,
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
        True,
        False,
        False,
        False,
    ]


def test_package_declares_robot_localization_dependency():
    package_xml = ET.fromstring((PACKAGE_ROOT / "package.xml").read_text())
    exec_depends = {
        element.text for element in package_xml.findall("./exec_depend") if element.text
    }

    assert "robot_localization" in exec_depends


def test_cmakelists_registers_odom_fusion_pytest():
    cmake_text = (PACKAGE_ROOT / "CMakeLists.txt").read_text()

    assert "test_odom_fusion_config" in cmake_text


def test_headless_start_script_prefers_odom_fusion():
    script_text = (PACKAGE_ROOT / "scripts/start_navigation_headless.sh").read_text()

    assert "use_odom_fusion:=true" in script_text
