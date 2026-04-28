import importlib.util
import math
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "src/go2w_real/scripts/odom_imu_motion_test.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("odom_imu_motion_test", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_default_plan_exercises_turns_and_half_meter_translation():
    module = _load_module()

    plan = module.build_motion_plan(
        linear_distance_m=0.5,
        linear_speed_mps=0.15,
        turn_angle_deg=90.0,
        angular_speed_radps=0.35,
        settle_sec=1.0,
        initial_hold_sec=2.0,
        final_hold_sec=3.0,
    )

    active = [segment for segment in plan if not segment.is_hold]
    assert [segment.name for segment in active] == [
        "turn_left_90deg",
        "turn_right_90deg",
        "forward_0.50m",
        "reverse_0.50m",
    ]
    assert active[0].angular_z == pytest.approx(0.35)
    assert active[1].angular_z == pytest.approx(-0.35)
    assert active[2].linear_x == pytest.approx(0.15)
    assert active[3].linear_x == pytest.approx(-0.15)
    assert active[0].duration_sec == pytest.approx((math.pi / 2.0) / 0.35)
    assert active[2].duration_sec == pytest.approx(0.5 / 0.15)


def test_motion_limits_require_meaningful_linear_speed_and_safe_distance():
    module = _load_module()

    with pytest.raises(ValueError, match="linear speed"):
        module.validate_motion_limits(
            linear_distance_m=0.5,
            linear_speed_mps=0.1,
            turn_angle_deg=90.0,
            angular_speed_radps=0.35,
        )

    with pytest.raises(ValueError, match="linear distance"):
        module.validate_motion_limits(
            linear_distance_m=0.51,
            linear_speed_mps=0.15,
            turn_angle_deg=90.0,
            angular_speed_radps=0.35,
        )


def test_default_cli_is_dry_run_for_physical_safety():
    module = _load_module()

    args = module.parse_args([])

    assert args.execute is False
    assert args.linear_distance_m == pytest.approx(0.5)
    assert args.linear_speed_mps > 0.1
    assert args.turn_angle_deg == pytest.approx(90.0)


def test_cmakelists_installs_motion_test_script():
    cmake_text = (REPO_ROOT / "src/go2w_real/CMakeLists.txt").read_text()

    assert "scripts/odom_imu_motion_test.py" in cmake_text
    assert "test_odom_imu_motion_test" in cmake_text
