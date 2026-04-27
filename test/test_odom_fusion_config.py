from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

import yaml


REPO_ROOT = Path("/home/unitree/ros_ws")
PACKAGE_ROOT = REPO_ROOT / "src/go2w_real"


def test_launch_defaults_to_rf2o_odom_with_optional_ekf_fusion():
    launch_text = (PACKAGE_ROOT / "launch/slam_rf2o.launch.py").read_text()
    match = re.search(
        r'DeclareLaunchArgument\(\s*"use_odom_fusion",(?P<body>.*?)\n\s*\)',
        launch_text,
        re.DOTALL,
    )

    assert match is not None
    assert 'default_value="false"' in match.group("body")
    assert "robot_localization" in launch_text
    assert 'executable="ekf_node"' in launch_text
    assert '("odometry/filtered", "/odom")' in launch_text
    assert "odom_fusion_params_file" in launch_text
    assert "nav2_params_file" in launch_text
    assert "slam_localization_params_file" in launch_text
    assert "/odom defaults to RF2O odom only" in launch_text
    assert "Set use_odom_fusion:=true" in launch_text


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


def test_headless_config_pipeline_defaults_to_rf2o_odom():
    config_text = (PACKAGE_ROOT / "config/navigation_headless.yaml").read_text()
    helper_output = subprocess.check_output(
        [
            "python3",
            str(PACKAGE_ROOT / "scripts/navigation_headless_config.py"),
            "--format",
            "shell",
        ],
        text=True,
    )
    script_text = (PACKAGE_ROOT / "scripts/start_navigation_headless.sh").read_text()

    assert "use_odom_fusion: false" in config_text
    assert "HEADLESS_USE_ODOM_FUSION=0" in helper_output
    assert (
        'use_odom_fusion:=$([[ "${HEADLESS_USE_ODOM_FUSION}" == "1" ]] && echo true || echo false)'
        in script_text
    )
    assert 'nav2_params_file:="${NAV2_PARAMS_FILE}"' in script_text
    assert (
        'slam_localization_params_file:="${SLAM_LOCALIZATION_PARAMS_FILE}"'
        in script_text
    )


def test_headless_start_script_supports_debug_rosbag_recording():
    config_text = (PACKAGE_ROOT / "config/navigation_headless.yaml").read_text()
    helper_output = subprocess.check_output(
        [
            "python3",
            str(PACKAGE_ROOT / "scripts/navigation_headless_config.py"),
            "--format",
            "shell",
        ],
        text=True,
    )
    script_text = (PACKAGE_ROOT / "scripts/start_navigation_headless.sh").read_text()
    topics_helper_text = (
        PACKAGE_ROOT / "scripts/navigation_debug_bag_topics.sh"
    ).read_text()

    assert "record_bag: true" in config_text
    assert "HEADLESS_RECORD_BAG=1" in helper_output
    assert "ros2 bag record" in script_text
    assert "--include-hidden-topics" in script_text
    assert "navigation_debug_bag_topics.sh" in script_text
    assert "/navigate_to_pose/_action/feedback" in topics_helper_text
    assert "/tf" in topics_helper_text
