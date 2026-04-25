from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[3]
CONFIG_CLI = REPO_ROOT / "src/go2w_real/scripts/navigation_headless_config.py"
START_SCRIPT = REPO_ROOT / "src/go2w_real/scripts/start_navigation_headless.sh"


def test_headless_config_cli_prints_shell_assignments():
    output = subprocess.check_output(
        ["python3", str(CONFIG_CLI), "--format", "shell"],
        text=True,
    )

    assert "HEADLESS_PROFILE=zt_0" in output
    assert "HEADLESS_CLEANUP_MATCHERS=(" in output
    assert (
        "HEADLESS_SERVER_URI=ws://192.168.123.186:8100/ws/navigation/executor"
        in output
    )
    assert "HEADLESS_USE_ODOM_FUSION=0" in output
    assert "HEADLESS_RECORD_BAG=1" in output


def test_headless_start_script_help_prefers_config_and_profile():
    help_text = subprocess.check_output(
        ["bash", str(START_SCRIPT), "--help"],
        text=True,
    )

    assert "--config PATH" in help_text
    assert "--profile NAME" in help_text
    assert "--server-uri" not in help_text
    assert "--heartbeat-interval" not in help_text
