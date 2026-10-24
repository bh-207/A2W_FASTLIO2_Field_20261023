#!/usr/bin/python3
"""Compare replay geometry against fixed factory-map evidence, without writing maps.

Requires NumPy, SciPy and Matplotlib (available in /usr/bin/python3 here).
Example:
  /usr/bin/python3 scripts/diagnostics/loop_recall_geometry.py \
    maps/factory_map_1016 BASELINE.csv ENHANCED.csv \
    --output artifacts/loop_recall_20261020/geometry

All geometry/region/point selections and original-map nearest-neighbour matches
are frozen before either replay variant is evaluated. Floor observations are
internal consistency evidence, NOT an independently surveyed horizontal truth.
"""
import argparse
import csv
from functools import lru_cache
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


POSE_COLUMNS = ("tx", "ty", "tz", "qx", "qy", "qz", "qw")
ALIASES = {"id": ("id", "keyframe_id"), "tx": ("tx", "x", "translation_x"),
           "ty": ("ty", "y", "translation_y"), "tz": ("tz", "z", "translation_z"),
           **{key: (key,) for key in POSE_COLUMNS[3:]}}
LIMITATIONS = [
    "No independent pose, gravity, horizontal-ground or elevation truth is available.",
    "These are internal geometric consistency metrics, not absolute localization accuracy or a true false-loop rate.",
    "Ground pairs and regions were selected using the original saved map; this selection can favour that map.",
    "A_reference is a provisional data-based stable region, not independently certified normal horizontal ground.",
    "Legacy loop measurements reconstructed from rounded logs are approximate, not serialized original factors.",
    "No automatic map acceptance is inferred from improved floor metrics; real slopes and non-ground geometry must be retained.",
]


def summary(values):
    values = np.asarray(values, dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("non-finite metric; refusing to silently drop evidence")
    if not values.size:
        return {"count": 0, "p50": None, "median": None, "p95": None, "max": None}
    median = float(np.median(values))
    return dict(count=int(values.size), p50=median, median=median,
                p95=float(np.quantile(values, .95)), max=float(np.max(values)))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def check_destination(output, bundle, evidence):
    output = Path(output).resolve()
    for original in (bundle, evidence):
        original = Path(original).resolve()
        if output == original or original in output.parents:
            raise ValueError("output must be outside the read-only Bundle and evidence directories")


def read_pose_csv(path, expected_count):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        names = {name.strip().lstrip("#").strip(): name for name in reader.fieldnames or []}
        selected = {}
        for key, aliases in ALIASES.items():
            selected[key] = next((names[alias] for alias in aliases if alias in names), None)
            if selected[key] is None:
                raise ValueError(f"{path}: missing column {key}")
        rows = list(reader)
    ids = np.array([float(row[selected["id"]]) for row in rows])
    if (not np.isfinite(ids).all() or not np.equal(ids, np.floor(ids)).all() or
            sorted(ids.tolist()) != list(range(expected_count))):
        raise ValueError(f"{path}: IDs must cover every Bundle frame exactly once")
    poses = np.array([[float(row[selected[key]]) for key in POSE_COLUMNS] for row in rows])
    poses = poses[np.argsort(ids)]
    if not np.isfinite(poses).all() or np.any(np.linalg.norm(poses[:, 3:], axis=1) < 1e-12):
        raise ValueError(f"{path}: non-finite pose or invalid quaternion")
    poses[:, 3:] = Rotation.from_quat(poses[:, 3:]).as_quat()
    return poses


def validate_manifest(bundle, evidence):
    manifest = bundle / "manifest.sha256"
    if (evidence / "original_manifest.sha256").read_bytes() != manifest.read_bytes():
        raise ValueError("ground evidence belongs to a different original Bundle")
    count = 0
    for line in manifest.read_text().splitlines():
        digest, relative = line.split(maxsplit=1)
        target = (bundle / relative.lstrip("*")).resolve()
        if bundle not in target.parents or sha256(target) != digest:
            raise ValueError(f"Bundle manifest validation failed: {relative}")
        count += 1
    return dict(manifest_sha256=sha256(manifest), validated_files=count)


def ground_rows(poses, pairs):
    rotations = Rotation.from_quat(poses[:, 3:]).as_matrix()
    rows = []
    for pair in pairs:
        i, j = pair["i"], pair["j"]
        a, b = rotations[i] @ pair["na"], rotations[j] @ pair["nb"]
        if min(abs(a[2]), abs(b[2])) < 1e-8:
            raise ValueError(f"ground plane becomes vertical at pair {i},{j}; cannot report height")
        d, e = pair["da"] - a @ poses[i, :3], pair["db"] - b @ poses[j, :3]
        height = -(d + a[:2] @ pair["xy"]) / a[2] + (e + b[:2] @ pair["xy"]) / b[2]
        angle = np.degrees(np.arccos(np.clip(a @ b, -1, 1)))
        rows.append([i, j, abs(float(height)), float(angle)])
    return np.array(rows).reshape(-1, 4)


def ground_metrics(rows, pairs, memberships):
    result = {}
    selectors = {"all": list(range(len(pairs)))}
    for partition in ("train", "heldout"):
        for scope in ("local", "outside"):
            for revisit in (False, True):
                key = f'{partition}_{scope}_{"revisit" if revisit else "adjacent"}'
                selectors[key] = [k for k, pair in enumerate(pairs) if
                                  pair["partition"] == partition and pair["scope"] == scope and
                                  pair["revisit"] == revisit]
    for region, members in memberships.items():
        selectors[f"region_{region}_heldout_revisit"] = [k for k, pair in enumerate(pairs) if
            pair["partition"] == "heldout" and pair["revisit"] and
            pair["i"] in members and pair["j"] in members]
    for name, indices in selectors.items():
        result[name] = dict(height_m=summary(rows[indices, 2]), normal_deg=summary(rows[indices, 3]))
    return result


def region_for_pair(i, j, memberships):
    return "|".join(name for name, ids in memberships.items() if i in ids and j in ids) or "cross_or_unassigned"


def freeze_surfaces(bundle, index, observations, pairs, saved, memberships):
    # Reuse original binary PCD reader, never its map-writing utility.
    path = Path(__file__).parent / "map_repair_1016" / "geometry.py"
    spec = importlib.util.spec_from_file_location("_original_map_geometry", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    @lru_cache(maxsize=300)
    def walls(i):
        cloud = (bundle / index[i]["cloud"]).resolve()
        if bundle not in cloud.parents:
            raise ValueError("point cloud path escapes Bundle")
        if not cloud.exists():
            cloud = bundle / "keyframes" / index[i]["cloud"]
        points = module.read_pcd(cloud)[:, :3].astype(float)
        obs = observations[i]
        height = points @ obs["normal"] + obs["d"]
        radius = np.linalg.norm(points[:, :2], axis=1)
        points = points[(height > .25) & (height < 3) & (radius < 10) & (radius > 1)]
        if len(points) < 30:
            return np.empty((0, 3))
        points = points[::max(1, len(points) // 1800)]
        distances, indices = cKDTree(points).query(points, k=12)
        local = points[indices]
        centered = local - local.mean(1)[:, None, :]
        _, vectors = np.linalg.eigh(np.einsum("nki,nkj->nij", centered, centered))
        normals = vectors[:, :, 0]
        return points[(np.abs(normals @ np.array(obs["normal"])) < .35) & (distances[:, -1] < .6)]

    rotations = Rotation.from_quat(saved[:, 3:]).as_matrix()
    wall_pairs = [pair for pair in pairs if pair["partition"] == "heldout" and pair["revisit"]]
    wall_pairs = wall_pairs[::max(1, len(wall_pairs) // 80)]
    fixed = []
    digest = hashlib.sha256()
    for pair in wall_pairs:
        i, j = pair["i"], pair["j"]
        a, b = walls(i), walls(j)
        if min(len(a), len(b)) < 40:
            continue
        aw, bw = a @ rotations[i].T + saved[i, :3], b @ rotations[j].T + saved[j, :3]
        lo = np.maximum(aw[:, :2].min(0), bw[:, :2].min(0))
        hi = np.minimum(aw[:, :2].max(0), bw[:, :2].max(0))
        a = a[((aw[:, :2] >= lo) & (aw[:, :2] <= hi)).all(1)]
        b = b[((bw[:, :2] >= lo) & (bw[:, :2] <= hi)).all(1)]
        if min(len(a), len(b)) < 40:
            continue
        aw, bw = a @ rotations[i].T + saved[i, :3], b @ rotations[j].T + saved[j, :3]
        ab, ba = cKDTree(bw).query(aw)[1], cKDTree(aw).query(bw)[1]
        fixed.append(dict(i=i, j=j, a=a, b=b, ab=ab, ba=ba, scope=pair["scope"],
                          region=region_for_pair(i, j, memberships)))
        for values in (np.array([i, j], dtype="<i8"), a.astype("<f8"), b.astype("<f8"),
                       ab.astype("<i8"), ba.astype("<i8")):
            digest.update(values.tobytes())
    return fixed, dict(frame_pair_count=len(fixed), directed_match_count=sum(
        len(pair["ab"]) + len(pair["ba"]) for pair in fixed),
        fixed_points_and_matches_sha256=digest.hexdigest(),
        selection="Original saved-map heldout revisit pairs, original local points and original XY overlap only; no variant-dependent crop, distance clipping or pair rejection.")


def surface_rows(poses, fixed):
    rotations = Rotation.from_quat(poses[:, 3:]).as_matrix()
    rows = []
    for pair in fixed:
        i, j = pair["i"], pair["j"]
        a = pair["a"] @ rotations[i].T + poses[i, :3]
        b = pair["b"] @ rotations[j].T + poses[j, :3]
        nearest = np.r_[cKDTree(b).query(a)[0], cKDTree(a).query(b)[0]]
        frozen = np.r_[np.linalg.norm(a - b[pair["ab"]], axis=1),
                       np.linalg.norm(b - a[pair["ba"]], axis=1)]
        rows.append([i, j, len(a), len(b), float(np.median(nearest)),
                     float(np.quantile(nearest, .95)), float(np.mean(nearest < .5)),
                     float(np.median(frozen)), float(np.quantile(frozen, .95)),
                     float(np.mean(frozen < .5))])
    return rows


def surface_metrics(rows, fixed, memberships):
    array = np.array(rows).reshape(-1, 10)
    selectors = {"all": list(range(len(fixed))), **{scope: [k for k, pair in enumerate(fixed)
        if pair["scope"] == scope] for scope in ("local", "outside")}}
    for region, ids in memberships.items():
        selectors[region] = [k for k, pair in enumerate(fixed) if pair["i"] in ids and pair["j"] in ids]
    return {name: dict(pair_count=len(ids),
        median_seam_m=summary(array[ids, 4]), p95_seam_m=summary(array[ids, 5]),
        coverage=summary(array[ids, 6]), fixed_match_p50_m=summary(array[ids, 7]),
        fixed_match_p95_m=summary(array[ids, 8]), fixed_match_coverage=summary(array[ids, 9]))
        for name, ids in selectors.items()}


def relative_pose(poses, i, j):
    rotations = Rotation.from_quat(poses[:, 3:])
    return rotations[i].inv().apply(poses[j, :3] - poses[i, :3]), rotations[i].inv() * rotations[j]


def factor_residual(poses, i, j, translation, rotation):
    actual_t, actual_r = relative_pose(poses, i, j)
    error_t = rotation.inv().apply(actual_t - translation)
    phi = (rotation.inv() * actual_r).as_rotvec()
    theta = np.linalg.norm(phi)
    x, y, z = phi
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    coefficient = 1 / 12 if theta < 1e-5 else (1 - .5 * theta / np.tan(.5 * theta)) / theta ** 2
    rho = (np.eye(3) - .5 * skew + coefficient * skew @ skew) @ error_t
    return float(np.linalg.norm(rho)), float(np.degrees(theta))


def odometry_rows(raw, poses):
    rows = []
    for i in range(len(raw) - 1):
        translation, rotation = relative_pose(raw[i:i + 2], 0, 1)
        rows.append([i, i + 1, *factor_residual(poses[i:i + 2], 0, 1, translation, rotation)])
    return np.array(rows).reshape(-1, 4)


def loop_rows(path, poses):
    if path is None:
        return [], "unavailable: no actual loop-measurement CSV supplied; no factors inferred from optimized poses"
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    result = []
    for row in rows:
        def field(*names):
            for name in names:
                if name in row:
                    return row[name]
            raise ValueError(f"{path}: missing one of {names}")
        i = int(field("from_id", "current", "current_id"))
        j = int(field("to_id", "history", "candidate_id"))
        if not (0 <= i < len(poses) and 0 <= j < len(poses)):
            raise ValueError("loop endpoint outside evaluated Bundle")
        measurement = np.array([float(field(key, "relative_" + key)) for key in POSE_COLUMNS])
        if not np.isfinite(measurement).all() or np.linalg.norm(measurement[3:]) < 1e-12:
            raise ValueError("invalid loop measurement")
        result.append([i, j, *factor_residual(poses, i, j, measurement[:3], Rotation.from_quat(measurement[3:]))])
    return result, "supplied fixed relative measurement from_id -> to_id; unwhitened SE(3) Log residual"


def pose_metrics(poses, reference, ids):
    ids = np.array(sorted(ids), dtype=int)
    if not len(ids):
        return {"frames": 0}
    delta = poses[ids, :3] - reference[ids, :3]
    rotation = Rotation.from_quat(poses[ids, 3:])
    euler = rotation.as_euler("xyz", degrees=True)
    change = (rotation * Rotation.from_quat(reference[ids, 3:]).inv()).as_euler("xyz", degrees=True)
    return dict(frames=len(ids), xyz_min=poses[ids, :3].min(0).tolist(), xyz_max=poses[ids, :3].max(0).tolist(),
                displacement_m=summary(np.linalg.norm(delta, axis=1)),
                absolute_xyz_change_m={axis: summary(abs(delta[:, k])) for k, axis in enumerate("xyz")},
                absolute_map_roll_deg=summary(abs(euler[:, 0])), absolute_map_pitch_deg=summary(abs(euler[:, 1])),
                map_roll_change_deg=summary(abs(change[:, 0])), map_pitch_change_deg=summary(abs(change[:, 1])))


def regression_warnings(baseline, enhanced, prefix=""):
    warnings = []
    for key, before in baseline.items():
        after = enhanced.get(key)
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(before, dict) and isinstance(after, dict):
            warnings.extend(regression_warnings(before, after, name))
        elif key in ("p50", "p95", "max") and before is not None and after is not None:
            # Coverage increases are improvements; errors increase when they worsen.
            worse = after < before - 1e-10 if "coverage" in name else after > before + 1e-10
            if worse:
                warnings.append(dict(metric=name, baseline=before, enhanced=after, delta=after - before))
    return warnings


def paired_regressions(baseline, enhanced, metrics, memberships):
    """Compare every original pair/factor; a positive delta always means worse.

    A region includes a pair if either endpoint belongs to its fixed original
    frame set. This catches changes at region boundaries. These memberships
    identify data-based reference areas, not surveyed horizontal truth.
    """
    def indexed(rows):
        result = {}
        for row in rows:
            key = (int(row[0]), int(row[1]))
            if key in result:
                raise ValueError(f"duplicate fixed pair {key}")
            result[key] = row
        return result

    before, after = indexed(baseline), indexed(enhanced)
    if before.keys() != after.keys():
        raise ValueError("variants must contain the same fixed pair/factor IDs")
    warnings = []
    for key in sorted(before):
        regions = [name for name, ids in memberships.items() if any(frame in ids for frame in key)]
        for metric, (column, higher_is_better) in metrics.items():
            a, b = float(before[key][column]), float(after[key][column])
            if not np.isfinite([a, b]).all():
                raise ValueError(f"non-finite pair metric {key}: {metric}")
            delta = a - b if higher_is_better else b - a
            if delta > 1e-10:
                warnings.append(dict(metric=metric, keyframes=list(key), baseline=a, enhanced=b,
                                     delta=delta, signed_change=b - a, regions=regions))

    def summarize(rows, compared_pairs):
        result = dict(compared_pairs=compared_pairs,
                      worsened_pairs=len({tuple(row["keyframes"]) for row in rows}), metrics={})
        for metric in metrics:
            selected = [row for row in rows if row["metric"] == metric]
            worst = max(selected, key=lambda row: row["delta"]) if selected else None
            result["metrics"][metric] = dict(
                worsened_pairs=len(selected), worst_delta=worst["delta"] if worst else None,
                worst_keyframes=worst["keyframes"] if worst else None,
                worst_baseline=worst["baseline"] if worst else None,
                worst_enhanced=worst["enhanced"] if worst else None)
        return result

    result = summarize(warnings, len(before))
    result["regions"] = {name: summarize(
        [row for row in warnings if name in row["regions"]],
        sum(any(frame in ids for frame in key) for key in before))
        for name, ids in memberships.items()}
    result["region_membership_rule"] = "either endpoint in the original saved-map region; no horizontal truth asserted"
    result["delta_convention"] = "positive means degradation; distance increase or coverage decrease; numerical tolerance 1e-10"
    result["warnings"] = warnings
    return result


def write_csv(path, header, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def plot_comparison(path, baseline, enhanced, results, memberships):
    os.environ.setdefault("MPLCONFIGDIR", tempfile.mkdtemp(prefix="loop-recall-mpl-"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 2, figsize=(14, 10))
    for label, poses in (("baseline", baseline), ("enhanced", enhanced)):
        axes[0, 0].plot(poses[:, 0], poses[:, 1], label=label, linewidth=1)
        axes[0, 1].plot(poses[:, 2], label=label, linewidth=1)
    axes[0, 0].set(title="Same frames in map XY", xlabel="map X (m)", ylabel="map Y (m)", aspect="equal")
    axes[0, 1].set(title="Map Z trajectory (not ground truth)", xlabel="frame ID", ylabel="map Z (m)")
    regions = [name for name in ("A_reference", "C_connector", "B_rising", "far") if name in memberships]
    positions = np.arange(len(regions))
    for offset, label in ((-.18, "baseline"), (.18, "enhanced")):
        values = [results[label]["ground"][f"region_{name}_heldout_revisit"]["height_m"]["p95"]
                  for name in regions]
        axes[1, 0].bar(positions + offset, [value if value is not None else np.nan for value in values], .35, label=label)
        surface = [results[label]["non_ground"][name]["fixed_match_p95_m"]["p50"] for name in regions]
        axes[1, 1].bar(positions + offset, [value if value is not None else np.nan for value in surface], .35, label=label)
    axes[1, 0].set(title="Fixed heldout revisit ground pairs: height P95", ylabel="m")
    axes[1, 1].set(title="Fixed non-ground matches: median pair P95", ylabel="m")
    for axis in axes[1]:
        axis.set_xticks(positions, regions, rotation=25, ha="right")
    for axis in axes.flat:
        axis.legend()
        axis.grid(alpha=.2)
    figure.suptitle("Internal geometry only; original-map selections; no independent horizontal or pose truth")
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def evaluate(args):
    bundle, evidence, output = args.bundle.resolve(), args.evidence.resolve(), args.output.resolve()
    check_destination(output, bundle, evidence)
    sources = [bundle / "manifest.sha256", args.baseline.resolve(), args.enhanced.resolve(),
               *[evidence / name for name in ("original_manifest.sha256", "ground_observations.json",
                                               "ground_pairs.json", "regions.json", "survey.npz")]]
    sources += sorted(evidence.glob("ground_pairs*.csv"))
    sources += [path.resolve() for path in (args.baseline_loops, args.enhanced_loops) if path]
    if any(path == output or output in path.parents for path in sources):
        raise ValueError("output directory must not contain any evaluation input")
    before_hashes = {str(path): sha256(path) for path in sources}
    manifest = validate_manifest(bundle, evidence)
    with (bundle / "keyframes/index.csv").open(newline="") as stream:
        index = list(csv.DictReader(stream))
    count = len(index)
    if [int(row["id"]) for row in index] != list(range(count)):
        raise ValueError("Bundle keyframe IDs are not contiguous")
    raw = np.array([[float(row[key]) for key in POSE_COLUMNS] for row in index])
    saved = read_pose_csv(bundle / "optimized_poses.csv", count)
    baseline, enhanced = read_pose_csv(args.baseline, count), read_pose_csv(args.enhanced, count)
    with np.load(evidence / "survey.npz", allow_pickle=False) as survey:
        if not (np.allclose(survey["raw"], raw, rtol=0, atol=1e-9) and
                np.allclose(survey["saved"], saved, rtol=0, atol=1e-9)):
            raise ValueError("survey poses do not match the original Bundle")
        sample, sample_ids = survey["sample"].copy(), survey["sample_ids"].copy()
    observations = json.loads((evidence / "ground_observations.json").read_text())
    pairs = json.loads((evidence / "ground_pairs.json").read_text())
    if len(observations) != count or any(not observations[pair[key]]["reliable"] for pair in pairs for key in ("i", "j")):
        raise ValueError("ground evidence has missing or unreliable selected observations")
    regions = json.loads((evidence / "regions.json").read_text())
    memberships = {name: set(ids) for name, ids in regions["keyframes"].items()}
    memberships["far"] = set().union(*(ids for name, ids in memberships.items() if name.startswith("H_")))
    print(f"Validated {count} frames; freezing original-map non-ground points and matches", flush=True)
    fixed, surface_audit = freeze_surfaces(bundle, index, observations, pairs, saved, memberships)
    if not fixed:
        raise ValueError("no fixed heldout non-ground surface pairs; cannot silently omit this check")
    output.mkdir(parents=True, exist_ok=True)
    results = {}
    pair_inputs = {}
    for label, poses, loop_path in (("baseline", baseline, args.baseline_loops),
                                    ("enhanced", enhanced, args.enhanced_loops)):
        ground = ground_rows(poses, pairs)
        surfaces = surface_rows(poses, fixed)
        odometry = odometry_rows(raw, poses)
        pair_inputs[label] = dict(ground=ground, non_ground=surfaces, odometry=odometry)
        loops, loop_note = loop_rows(loop_path, poses)
        rotations = Rotation.from_quat(poses[:, 3:]).as_matrix()
        saved_rotations = Rotation.from_quat(saved[:, 3:]).as_matrix()
        sample_world = np.einsum("nij,nj->ni", rotations[sample_ids], sample) + poses[sample_ids, :3]
        saved_world = np.einsum("nij,nj->ni", saved_rotations[sample_ids], sample) + saved[sample_ids, :3]
        sample_delta = sample_world - saved_world
        results[label] = dict(ground=ground_metrics(ground, pairs, memberships),
            non_ground=surface_metrics(surfaces, fixed, memberships),
            odometry=dict(translation_log_m=summary(odometry[:, 2]), rotation_deg=summary(odometry[:, 3])),
            loops=dict(available=loop_path is not None, note=loop_note,
                       translation_log_m=summary([row[2] for row in loops]), rotation_deg=summary([row[3] for row in loops])),
            poses_vs_saved={name: pose_metrics(poses, saved, ids) for name, ids in
                            {"all": set(range(count)), **memberships}.items()},
            fixed_survey_points_vs_saved=dict(count=len(sample), displacement_m=summary(np.linalg.norm(sample_delta, axis=1)),
                absolute_xyz_change_m={axis: summary(abs(sample_delta[:, k])) for k, axis in enumerate("xyz")}))
        write_csv(output / f"{label}_ground_pairs.csv",
            ["i", "j", "absolute_height_difference_m", "normal_difference_deg", "partition", "scope", "revisit", "region"],
            [list(row) + [pair["partition"], pair["scope"], pair["revisit"], region_for_pair(pair["i"], pair["j"], memberships)]
             for row, pair in zip(ground, pairs)])
        write_csv(output / f"{label}_non_ground.csv",
            ["i", "j", "source_points", "target_points", "median_nn_m", "p95_nn_m", "coverage_within_0.5m",
             "fixed_match_p50_m", "fixed_match_p95_m", "fixed_match_coverage_within_0.5m", "scope", "region"],
            [row + [pair["scope"], pair["region"]] for row, pair in zip(surfaces, fixed)])
        for name, rows in (("odometry", odometry), ("loops", loops)):
            write_csv(output / f"{label}_{name}_residuals.csv", ["i", "j", "translation_log_m", "rotation_deg"], rows)
        euler = Rotation.from_quat(poses[:, 3:]).as_euler("xyz", degrees=True)
        write_csv(output / f"{label}_poses.csv", ["id", *POSE_COLUMNS, "map_roll_deg", "map_pitch_deg", "map_yaw_deg"],
                  [[i, *pose, *angles] for i, (pose, angles) in enumerate(zip(poses, euler))])
    comparison = {name: pose_metrics(enhanced, baseline, ids) for name, ids in
                  {"all": set(range(count)), **memberships}.items()}
    changes = (Rotation.from_quat(enhanced[:, 3:]) * Rotation.from_quat(baseline[:, 3:]).inv()).as_euler("xyz", degrees=True)
    write_csv(output / "enhanced_vs_baseline_pose_changes.csv",
              ["id", "map_dx_m", "map_dy_m", "map_dz_m", "map_delta_roll_deg", "map_delta_pitch_deg", "map_delta_yaw_deg"],
              [[i, *(enhanced[i, :3] - baseline[i, :3]), *changes[i]] for i in range(count)])
    warning_inputs = {label: {key: results[label][key] for key in ("ground", "non_ground", "odometry")} for label in results}
    warnings = regression_warnings(warning_inputs["baseline"], warning_inputs["enhanced"])
    aggregate_warning_count = len(warnings)
    pair_metric_columns = dict(
        ground={"height_m": (2, False), "normal_deg": (3, False)},
        non_ground={"median_nn_m": (4, False), "p95_nn_m": (5, False), "nn_coverage": (6, True),
                    "fixed_match_p50_m": (7, False), "fixed_match_p95_m": (8, False),
                    "fixed_match_coverage": (9, True)},
        odometry={"translation_log_m": (2, False), "rotation_deg": (3, False)})
    pairwise = {kind: paired_regressions(pair_inputs["baseline"][kind], pair_inputs["enhanced"][kind],
                                       columns, memberships) for kind, columns in pair_metric_columns.items()}
    pair_warning_rows, pair_summary_rows = [], []
    for kind, comparison_rows in pairwise.items():
        for row in comparison_rows["warnings"]:
            warnings.append(dict(metric=f'{kind}.pair.{row["metric"]}', baseline=row["baseline"],
                                 enhanced=row["enhanced"], delta=row["signed_change"],
                                 degradation_delta=row["delta"], keyframes=row["keyframes"], regions=row["regions"]))
            pair_warning_rows.append([kind, *row["keyframes"], row["metric"], row["baseline"],
                                      row["enhanced"], row["signed_change"], row["delta"], "|".join(row["regions"])])
        for region, details in {"all": comparison_rows, **comparison_rows["regions"]}.items():
            for metric, values in details["metrics"].items():
                keyframes = values["worst_keyframes"] or [None, None]
                pair_summary_rows.append([kind, region, metric, details["compared_pairs"],
                    details["worsened_pairs"], values["worsened_pairs"], values["worst_delta"],
                    *keyframes, values["worst_baseline"], values["worst_enhanced"]])
    write_csv(output / "pairwise_regression_warnings.csv",
              ["kind", "i", "j", "metric", "baseline", "enhanced", "signed_change", "degradation_delta", "regions"],
              pair_warning_rows)
    write_csv(output / "pairwise_regression_summary.csv",
              ["kind", "region", "metric", "compared_pairs", "any_metric_worsened_pairs", "metric_worsened_pairs",
               "worst_degradation_delta", "worst_i", "worst_j", "worst_baseline", "worst_enhanced"], pair_summary_rows)
    plot_comparison(output / "geometry_comparison.png", baseline, enhanced, results, memberships)
    after_hashes = {str(path): sha256(path) for path in sources}
    if before_hashes != after_hashes:
        raise ValueError("an evaluation input changed during the run")
    report = dict(schema_version=2, bundle=str(bundle), frame_count=count, limitations=LIMITATIONS,
                  manifest=manifest, input_sha256=before_hashes, inputs_unchanged=True,
                  fixed_evidence=dict(ground_pairs=len(pairs), region_memberships={key: sorted(ids) for key, ids in memberships.items()},
                                      non_ground=surface_audit, regions_reference_note=regions["reference_note"]),
                  residual_convention="Unwhitened Log(Z_ij^-1 * (P_i^-1 * P_j)); no production noise/Huber changes",
                  results=results, enhanced_vs_baseline_poses=comparison,
                  regression_warnings=warnings, regression_warning_count=len(warnings),
                  aggregate_regression_warning_count=aggregate_warning_count,
                  pairwise_regression_warning_count=len(pair_warning_rows), pairwise_regressions=pairwise,
                  acceptance="not decided; inspect every regional degradation, including provisional normal/reference areas")
    (output / "geometry_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    write_csv(output / "regression_warnings.csv", ["metric", "baseline", "enhanced", "delta"],
              [[row[key] for key in ("metric", "baseline", "enhanced", "delta")] for row in warnings])
    print(json.dumps(dict(output=str(output), frames=count, ground_pairs=len(pairs),
                          fixed_surface_pairs=len(fixed), regression_warnings=len(warnings))))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("enhanced", type=Path)
    parser.add_argument("--evidence", type=Path, default=Path("artifacts/map_repair_1016"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline-loops", type=Path, help="actual relative measurements: from_id,to_id,tx,ty,tz,qx,qy,qz,qw")
    parser.add_argument("--enhanced-loops", type=Path)
    evaluate(parser.parse_args())


if __name__ == "__main__":
    main()
