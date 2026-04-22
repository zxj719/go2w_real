import importlib.util
from pathlib import Path


REPO_ROOT = Path("/home/unitree/ros_ws")
SAVE_MAP_AND_WAYPOINTS_PATH = (
    REPO_ROOT / "src/go2w_real/scripts/save_map_and_waypoints.py"
)


def _load_save_map_and_waypoints_module():
    spec = importlib.util.spec_from_file_location(
        "go2w_save_map_and_waypoints_script",
        SAVE_MAP_AND_WAYPOINTS_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


save_waypoints = _load_save_map_and_waypoints_module()


def test_default_waypoint_identity_sequence_starts_from_poi_013():
    assert save_waypoints._default_waypoint_id(1) == "POI_013"
    assert save_waypoints._default_waypoint_id(2) == "POI_014"
    assert save_waypoints._default_waypoint_name(1) == "wp_01"
    assert save_waypoints._default_waypoint_name(2) == "wp_02"


def test_serialize_waypoints_yaml_preserves_map_header_and_overwrites_waypoints():
    existing_text = (
        "map:\n"
        "  serialized_prefix: '/home/unitree/ros_ws/src/map/zt_0'\n"
        "  data: '/home/unitree/ros_ws/src/map/zt_0.data'\n"
        "  posegraph: '/home/unitree/ros_ws/src/map/zt_0.posegraph'\n"
        "waypoints:\n"
        "  - id: 'POI_999'\n"
        "    name: 'old_wp'\n"
    )
    waypoints = [
        {
            "id": "POI_013",
            "name": "wp_01",
            "frame_id": "map",
            "position": {"x": 1.0, "y": 2.0, "z": 0.0},
            "yaw": 0.5,
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.247404, "w": 0.968912},
        }
    ]

    rendered = save_waypoints._serialize_waypoints_yaml(
        waypoints,
        existing_text=existing_text,
    )

    assert "serialized_prefix" in rendered
    assert "old_wp" not in rendered
    assert "POI_013" in rendered
    assert "wp_01" in rendered
    assert rendered.count("waypoints:") == 1
