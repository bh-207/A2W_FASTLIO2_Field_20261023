import csv
import math
from pathlib import Path

import yaml

from a2w_waypoint_tools.route_storage import (
    Waypoint,
    is_capture_key,
    quaternion_yaw,
    save_route_files,
)


def test_quaternion_yaw_returns_planar_heading():
    half_yaw = math.radians(45.0)

    yaw = quaternion_yaw(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1.0e-12)


def test_only_s_or_space_capture_a_waypoint():
    assert is_capture_key("s")
    assert is_capture_key("S")
    assert is_capture_key(" ")
    assert not is_capture_key("\n")
    assert not is_capture_key("q")


def test_save_route_files_preserves_map_pose_for_future_navigation(tmp_path: Path):
    output_prefix = tmp_path / "routes" / "route_01"
    waypoints = [
        Waypoint(
            index=0,
            stamp_sec=123,
            stamp_nanosec=456,
            x=1.25,
            y=-2.5,
            z=0.1,
            qx=0.0,
            qy=0.0,
            qz=0.5,
            qw=0.8660254038,
            yaw=math.pi / 3.0,
        ),
        Waypoint(
            index=1,
            stamp_sec=124,
            stamp_nanosec=789,
            x=2.0,
            y=-1.0,
            z=0.2,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            yaw=0.0,
        ),
    ]

    yaml_path, csv_path = save_route_files(
        output_prefix,
        frame_id="map",
        robot_frame="base_link",
        waypoints=waypoints,
        saved_at_utc="2026-09-22T10:00:00Z",
    )

    assert yaml_path == output_prefix.with_suffix(".yaml")
    assert csv_path == output_prefix.with_suffix(".csv")
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    assert document["schema"] == "a2w_waypoint_route_v1"
    assert document["frame_id"] == "map"
    assert document["robot_frame"] == "base_link"
    assert document["waypoint_count"] == 2
    assert document["waypoints"][0] == {
        "id": 0,
        "stamp": {"sec": 123, "nanosec": 456},
        "position": {"x": 1.25, "y": -2.5, "z": 0.1},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.5, "w": 0.8660254038},
        "yaw": math.pi / 3.0,
    }

    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["frame_id"] == "map"
    assert rows[0]["x"] == "1.250000000"
    assert rows[0]["y"] == "-2.500000000"
    assert rows[0]["yaw"] == "1.047197551"


def test_save_route_files_rejects_an_empty_output_prefix(tmp_path: Path):
    try:
        save_route_files(
            tmp_path / "",
            frame_id="map",
            robot_frame="base_link",
            waypoints=[],
            saved_at_utc="2026-09-22T10:00:00Z",
        )
    except ValueError as error:
        assert "filename" in str(error)
    else:
        raise AssertionError("directory-only output prefix must be rejected")
