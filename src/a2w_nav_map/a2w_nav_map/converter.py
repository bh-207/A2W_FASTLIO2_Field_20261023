"""Deterministic conversion of a Map Bundle V1 PCD into a Nav2 map."""

from dataclasses import asdict, dataclass
import hashlib
import math
import os
from pathlib import Path
import tempfile
import shutil
import subprocess
from typing import Tuple

import numpy as np
import yaml


class ConversionError(RuntimeError):
    """The bundle cannot safely be converted."""


@dataclass(frozen=True)
class ConversionConfig:
    resolution: float = 0.05
    padding: float = 1.0
    ground_min_z: float = -0.85
    ground_max_z: float = -0.25
    obstacle_min_z: float = -0.20
    obstacle_max_z: float = 1.50
    map_z_confirmed: bool = False
    up_in_map: tuple = None
    method: str = "height"
    min_obstacle_points: int = 1
    voxel_leaf: float = 0.10
    threads: int = 4
    max_window_size: int = 32
    slope: float = 1.0
    initial_distance: float = 0.20
    max_distance: float = 2.5
    mean_k: int = 20
    stddev_mul: float = 1.0
    h_min: float = 0.10
    h_max: float = 1.50


@dataclass(frozen=True)
class ConversionResult:
    frame: str
    width: int
    height: int
    origin_x: float
    origin_y: float
    finite_points: int
    free_cells: int
    occupied_cells: int


def _validate_config(config: ConversionConfig) -> None:
    values = asdict(config)
    if not all(math.isfinite(value) for value in values.values() if isinstance(value, (int, float))):
        raise ConversionError("conversion parameters must be finite")
    if config.resolution <= 0.0 or config.padding < 0.0:
        raise ConversionError("resolution must be positive and padding non-negative")
    if config.ground_min_z > config.ground_max_z:
        raise ConversionError("invalid ground height range")
    if config.obstacle_min_z > config.obstacle_max_z:
        raise ConversionError("invalid obstacle height range")
    if config.method not in ("height", "pmf"):
        raise ConversionError("method must be height or pmf")
    if (config.min_obstacle_points < 1 or config.threads < 1 or config.voxel_leaf < 0
            or config.max_window_size < 1 or config.mean_k < 0 or config.h_min > config.h_max
            or min(config.slope, config.initial_distance, config.max_distance, config.stddev_mul) < 0):
        raise ConversionError("invalid obstacle/PMF parameters")


def projection_rotation(config: ConversionConfig) -> np.ndarray:
    """Rows are projection axes in the source map; no terrain/gravity inference."""
    if type(config.map_z_confirmed) is not bool:
        raise ConversionError("map_z_confirmed must be a YAML boolean, not a quoted string")
    if config.up_in_map is None:
        if not config.map_z_confirmed:
            raise ConversionError("vertical reference unknown: supply --up or --confirm-map-z after verification")
        return np.eye(3)
    up = np.array(config.up_in_map, dtype=np.float64, copy=True)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-9:
        raise ConversionError("vertical reference must be a finite nonzero 3-vector")
    up /= np.linalg.norm(up)
    forward = np.array([1., 0., 0.]) if abs(up[0]) < .99 else np.array([0., 1., 0.])
    forward -= (forward @ up) * up
    forward /= np.linalg.norm(forward)
    return np.vstack([forward, np.cross(up, forward), up])


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _verify_manifest(bundle: Path) -> None:
    manifest = bundle / "manifest.sha256"
    if not manifest.is_file():
        raise ConversionError("manifest.sha256 is missing")
    for line_number, raw in enumerate(manifest.read_text(encoding="utf-8").splitlines(), 1):
        parts = raw.split(maxsplit=1)
        if len(parts) != 2 or len(parts[0]) != 64:
            raise ConversionError(f"invalid manifest entry on line {line_number}")
        digest, relative = parts
        target = (bundle / relative).resolve()
        try:
            target.relative_to(bundle.resolve())
        except ValueError as error:
            raise ConversionError("manifest path escapes bundle") from error
        if not target.is_file():
            raise ConversionError(f"manifest file missing: {relative}")
        actual = _sha256(target)
        if actual != digest:
            raise ConversionError(f"checksum mismatch: {relative}")


def _read_metadata(bundle: Path) -> dict:
    path = bundle / "metadata.yaml"
    try:
        metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ConversionError(f"cannot read metadata.yaml: {error}") from error
    if not isinstance(metadata, dict) or metadata.get("schema") != "a2w_map_bundle_v1":
        raise ConversionError("unsupported Map Bundle schema")
    if metadata.get("map_frame") != "map":
        raise ConversionError("metadata map_frame must be map")
    if not metadata.get("bundle_uuid"):
        raise ConversionError("metadata bundle_uuid is missing")
    return metadata


def _parse_binary_pcd(path: Path) -> np.ndarray:
    try:
        with path.open("rb") as stream:
            values = {}
            for _ in range(100):
                line = stream.readline(65536).decode("ascii").strip()
                if not line or line.startswith("#"):
                    continue
                key, *parts = line.split()
                values[key] = parts
                if key == "DATA":
                    break
            else:
                raise ConversionError("invalid PCD header")
            fields = values["FIELDS"]
            counts = list(map(int, values.get("COUNT", ["1"] * len(fields))))
            sizes = list(map(int, values["SIZE"]))
            types = values["TYPE"]
            count = int(values["POINTS"][0])
            if count < 0 or not {"x", "y", "z"}.issubset(fields):
                raise ConversionError("PCD must contain x y z")
            if len(set(map(len, [fields, counts, sizes, types]))) != 1 or any(c < 1 for c in counts):
                raise ConversionError("invalid PCD field layout")
            if values["DATA"] == ["ascii"]:
                data = np.loadtxt(stream, ndmin=2) if count else np.empty((0, sum(counts)))
                if data.shape != (count, sum(counts)):
                    raise ConversionError("PCD payload size does not match POINTS")
                offsets = np.cumsum([0] + counts[:-1])
                columns = {name: data[:, index] for name, index in zip(fields, offsets)}
            elif values["DATA"] == ["binary"]:
                dtype = np.dtype([(name, f"<{ {'F': 'f', 'I': 'i', 'U': 'u'}[typ]}{size}", (n,))
                                  for name, typ, size, n in zip(fields, types, sizes, counts)])
                remaining = path.stat().st_size - stream.tell()
                if remaining != count * dtype.itemsize:
                    raise ConversionError("PCD payload size does not match POINTS")
                data = np.fromfile(stream, dtype=dtype, count=count)
                columns = {name: data[name][:, 0] for name in fields}
            else:
                raise ConversionError("compressed PCD requires --method pmf (PCL reader)")
        return np.column_stack([columns["x"], columns["y"], columns["z"],
                                columns.get("intensity", np.zeros(count))]).astype(np.float64)
    except (OSError, ValueError, KeyError, IndexError, UnicodeError) as error:
        raise ConversionError(f"cannot read PCD: {error}") from error


def _rasterize(points: np.ndarray, config: ConversionConfig, classes=None,
               frame="map") -> Tuple[np.ndarray, ConversionResult]:
    finite_mask = np.isfinite(points[:, :3]).all(axis=1)
    finite = points[finite_mask]
    if classes is not None:
        classes = classes[finite_mask]
    if finite.size == 0:
        raise ConversionError("global_map.pcd has no finite points")
    origin_x = math.floor((float(finite[:, 0].min()) - config.padding) / config.resolution) * config.resolution
    origin_y = math.floor((float(finite[:, 1].min()) - config.padding) / config.resolution) * config.resolution
    max_x = math.ceil((float(finite[:, 0].max()) + config.padding) / config.resolution) * config.resolution
    max_y = math.ceil((float(finite[:, 1].max()) + config.padding) / config.resolution) * config.resolution
    width = max(1, int(round((max_x - origin_x) / config.resolution)) + 1)
    height = max(1, int(round((max_y - origin_y) / config.resolution)) + 1)
    if width * height > 100_000_000:
        raise ConversionError("navigation raster would exceed 100 million cells")

    columns = np.floor((finite[:, 0] - origin_x) / config.resolution).astype(np.int64)
    rows = np.floor((finite[:, 1] - origin_y) / config.resolution).astype(np.int64)
    valid = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    columns, rows, z = columns[valid], rows[valid], finite[valid, 2]

    grid = np.full((height, width), 205, dtype=np.uint8)
    ground = ((z >= config.ground_min_z) & (z <= config.ground_max_z)
              if classes is None else classes[valid] == 0)
    grid[rows[ground], columns[ground]] = 254
    obstacle = ((z >= config.obstacle_min_z) & (z <= config.obstacle_max_z)
                if classes is None else classes[valid] == 1)
    cell_ids, counts = np.unique(rows[obstacle] * width + columns[obstacle], return_counts=True)
    grid.ravel()[cell_ids[counts >= config.min_obstacle_points]] = 0
    result = ConversionResult(
        frame=frame, width=width, height=height,
        origin_x=float(origin_x), origin_y=float(origin_y),
        finite_points=int(len(finite)),
        free_cells=int(np.count_nonzero(grid == 254)),
        occupied_cells=int(np.count_nonzero(grid == 0)),
    )
    if result.free_cells + result.occupied_cells == 0:
        raise ConversionError("height filters selected no map cells")
    return np.flipud(grid), result


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _publish_map_files(output: Path, contents: dict) -> None:
    """Stage the complete set and restore prior files if a replacement fails."""
    if output.exists() and not output.is_dir():
        raise ConversionError("output must be a directory")
    for name in contents:
        target = output / name
        if target.is_symlink() or (target.exists() and not target.is_file()):
            raise ConversionError(f"output target is not a regular file: {name}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".a2w-map-", dir=output.parent) as directory:
        staging = Path(directory)
        backup = staging / 'backup'
        backup.mkdir()
        for name, data in contents.items():
            _atomic_write(staging / name, data)
            if (output / name).exists():
                shutil.copy2(output / name, backup / name)
        output.mkdir(exist_ok=True)
        replaced = []
        try:
            for name in contents:
                os.replace(staging / name, output / name)
                replaced.append(name)
        except BaseException:
            for name in reversed(replaced):
                if (backup / name).exists():
                    os.replace(backup / name, output / name)
                else:
                    (output / name).unlink()
            raise


def _pmf_clouds(source, rotation, config):
    helper = os.environ.get("A2W_PCD_FILTER_BIN") or shutil.which("pcd_ground_filter")
    if helper is None:
        try:
            from ament_index_python.packages import get_package_prefix
            helper = str(Path(get_package_prefix("a2w_pcd_filters")) / "lib/a2w_pcd_filters/pcd_ground_filter")
        except (ImportError, LookupError) as error:
            raise ConversionError("PMF helper missing; run build.sh and source env.sh") from error
    with tempfile.TemporaryDirectory(prefix="a2w-pmf-") as directory:
        args = [helper, str(source), directory, *map(str, rotation.ravel()),
                *map(str, [config.voxel_leaf, config.threads, config.max_window_size, config.slope,
                          config.initial_distance, config.max_distance, config.mean_k,
                          config.stddev_mul, config.h_min, config.h_max])]
        try:
            subprocess.run(args, check=True)
        except (OSError, subprocess.CalledProcessError) as error:
            raise ConversionError(f"PMF failed: {error}") from error
        ground = _parse_binary_pcd(Path(directory) / "ground.pcd")
        obstacles = _parse_binary_pcd(Path(directory) / "obstacles.pcd")
        return np.vstack([ground, obstacles]), np.r_[np.zeros(len(ground)), np.ones(len(obstacles))]


def convert_pcd(pcd_path: Path, output_path: Path, config: ConversionConfig,
                source_frame="map", metadata=None, overwrite=False) -> ConversionResult:
    """Read-only conversion; changed axes get a distinct frame and explicit transform."""
    _validate_config(config)
    rotation = projection_rotation(config)
    source = Path(pcd_path).resolve()
    output = Path(output_path).resolve()
    output_names = ("nav_map.pgm", "nav_map.yaml", "conversion.yaml")
    if any((output / name).resolve() == source for name in output_names):
        raise ConversionError("source cloud collides with an output file")
    if not source_frame or not source_frame.replace('_', '').isalnum():
        raise ConversionError("invalid source frame")
    if not overwrite and any((output / name).exists() for name in ("nav_map.pgm", "nav_map.yaml", "conversion.yaml")):
        raise ConversionError("output exists; use --overwrite explicitly")
    frame = source_frame if np.allclose(rotation, np.eye(3), atol=1e-12) else source_frame + "_projection"
    if config.method == "pmf":
        points, classes = _pmf_clouds(source, rotation, config)
    else:
        points = _parse_binary_pcd(source)
        points[:, :3] = points[:, :3] @ rotation.T
        classes = None
    image, result = _rasterize(points, config, classes, frame)

    pgm = f"P5\n{result.width} {result.height}\n255\n".encode("ascii") + image.tobytes()
    nav_yaml = {
        "image": "nav_map.pgm", "mode": "trinary",
        "resolution": config.resolution,
        "origin": [result.origin_x, result.origin_y, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }
    conversion = {
        "schema": "a2w_nav_map_v1",
        "source_bundle_uuid": (metadata or {}).get("bundle_uuid"),
        "map_frame": frame,
        "source_frame": source_frame,
        "source": source.name,
        "source_sha256": _sha256(source),
        "projection_T_source": np.block([[rotation, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]).tolist(),
        "source_T_projection": np.block([[rotation.T, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]).tolist(),
        "navigation_frame_note": "Changed projection axes are not interchangeable with source map XY; use the recorded transform.",
        "config": asdict(config),
        "result": asdict(result),
    }
    contents = {
        "nav_map.pgm": pgm,
        "nav_map.yaml": yaml.safe_dump(nav_yaml, sort_keys=False).encode(),
        "conversion.yaml": yaml.safe_dump(conversion, sort_keys=False).encode(),
    }
    _publish_map_files(output, contents)
    return result


def convert_bundle(bundle_path: Path, output_path: Path,
                   config: ConversionConfig, overwrite=False) -> ConversionResult:
    bundle = Path(bundle_path).resolve()
    _verify_manifest(bundle)
    metadata = _read_metadata(bundle)
    return convert_pcd(bundle / "global_map.pcd", output_path, config,
                       source_frame=metadata["map_frame"], metadata=metadata, overwrite=overwrite)

