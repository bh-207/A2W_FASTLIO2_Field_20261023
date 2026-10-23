"""Pure waypoint formatting and persistence helpers."""

from dataclasses import asdict, dataclass
import csv
from io import StringIO
import math
import os
from pathlib import Path
from typing import Iterable, Tuple

import yaml


@dataclass(frozen=True)
class Waypoint:
    index: int
    stamp_sec: int
    stamp_nanosec: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw: float


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw angle represented by a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_capture_key(key: str) -> bool:
    return key in ("s", "S", " ")


def _waypoint_document(waypoint: Waypoint) -> dict:
    return {
        "id": waypoint.index,
        "stamp": {"sec": waypoint.stamp_sec, "nanosec": waypoint.stamp_nanosec},
        "position": {"x": waypoint.x, "y": waypoint.y, "z": waypoint.z},
        "orientation": {
            "x": waypoint.qx,
            "y": waypoint.qy,
            "z": waypoint.qz,
            "w": waypoint.qw,
        },
        "yaw": waypoint.yaw,
    }


def _atomic_write(path: Path, text: str) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(text, encoding="utf-8")
    os.replace(temporary, path)


def save_route_files(
    output_prefix: Path,
    *,
    frame_id: str,
    robot_frame: str,
    waypoints: Iterable[Waypoint],
    saved_at_utc: str,
) -> Tuple[Path, Path]:
    """Atomically save a route as canonical YAML and convenient CSV."""
    output_prefix = Path(output_prefix).expanduser()
    if not output_prefix.name or (output_prefix.exists() and output_prefix.is_dir()):
        raise ValueError("output_prefix must include a filename")
    if not frame_id or not robot_frame:
        raise ValueError("frame_id and robot_frame must be non-empty")

    route = tuple(waypoints)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    yaml_path = output_prefix.with_suffix(".yaml")
    csv_path = output_prefix.with_suffix(".csv")

    document = {
        "schema": "a2w_waypoint_route_v1",
        "frame_id": frame_id,
        "robot_frame": robot_frame,
        "saved_at_utc": saved_at_utc,
        "waypoint_count": len(route),
        "waypoints": [_waypoint_document(point) for point in route],
    }
    yaml_text = yaml.safe_dump(document, allow_unicode=True, sort_keys=False)

    csv_stream = StringIO(newline="")
    fieldnames = [
        "id", "frame_id", "stamp_sec", "stamp_nanosec",
        "x", "y", "z", "qx", "qy", "qz", "qw", "yaw",
    ]
    writer = csv.DictWriter(csv_stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    for point in route:
        row = asdict(point)
        row["id"] = row.pop("index")
        row["frame_id"] = frame_id
        for field in ("x", "y", "z", "qx", "qy", "qz", "qw", "yaw"):
            row[field] = f"{row[field]:.9f}"
        writer.writerow(row)

    _atomic_write(yaml_path, yaml_text)
    _atomic_write(csv_path, csv_stream.getvalue())
    return yaml_path, csv_path
