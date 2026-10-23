#!/usr/bin/python3
"""Post-hoc fixed-loop graph attribution; never an online oracle for edge rejection.

First run loop_constraint_ablation BUNDLE ACTUAL_LOOP_CSV NEW_GRAPH_DIR.
Then run this script BUNDLE V2_REPLAY_DIR GRAPH_DIR --output NEW_REPORT_DIR.
Original v2 measurements remain fixed after removal of an earlier constraint;
their later registration inputs are not regenerated. This limits causal claims.
"""
import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


SPEC = importlib.util.spec_from_file_location("_frozen_loop_geometry", Path(__file__).with_name("loop_recall_geometry.py"))
geometry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(geometry)
FOCUS = ((582, 1322), (70, 1298), (1170, 1214))
LIMITATIONS = [
    "POSTHOC_FIXED_MEASUREMENT_ATTRIBUTION_NOT_ONLINE_LOOP_SELECTION",
    "Original odometry, original graph noise, robust loss and factor source timestamps are retained.",
    "Later v2 measurements are not regenerated after removal of an earlier edge; full-trajectory ablations are hindsight counterfactuals.",
    "Matched-source prefix controls inject no future factors; focus pairs involving future keyframes remain unavailable at that source time.",
    "Original fixed evaluation pairs have already been inspected; they are regression evidence, not a fresh blind validation set.",
    "Ground planes are relative internal evidence, not independent gravity or horizontal/elevation truth.",
    "beneficial/harmful labels refer only to these fixed metric changes. Mixed effects are uncertain; no error rate or true-loop label is inferred.",
    "Frozen original-map non-ground correspondences can penalize valid reassociation; NN and fixed-match metrics are both retained.",
    "Graph ablation alone cannot identify repeated roof features or IMU/gravity faults as the physical cause.",
]


def classify_effect(with_edge, without_edge):
    """Compare per metric without adding quantities with different units."""
    effects = {}
    for key in with_edge.keys() & without_edge.keys():
        a, b = with_edge[key], without_edge[key]
        if a is None or b is None:
            continue
        delta = b - a if "coverage" in key else a - b
        effects[key] = dict(with_edge=a, without_edge=b, degradation_delta=delta,
                            effect="harmful" if delta > 1e-9 else "beneficial" if delta < -1e-9 else "uncertain")
    labels = {value["effect"] for value in effects.values()} - {"uncertain"}
    return (next(iter(labels)) if len(labels) == 1 else "uncertain"), effects


def make_ground_anchors(saved, pairs):
    rotations = Rotation.from_quat(saved[:, 3:]).as_matrix()
    anchors = []
    for pair in pairs:
        local = []
        for key, normal_key, d_key in (("i", "na", "da"), ("j", "nb", "db")):
            i = pair[key]
            normal = rotations[i] @ pair[normal_key]
            d = pair[d_key] - normal @ saved[i, :3]
            z = -(d + normal[:2] @ pair["xy"]) / normal[2]
            world = np.array([*pair["xy"], z])
            local.append(rotations[i].T @ (world - saved[i, :3]))
        anchors.append(local)
    return np.array(anchors)


def invariant_ground_rows(poses, pairs, anchors):
    rotations = Rotation.from_quat(poses[:, 3:]).as_matrix()
    result = []
    for pair, (a, b) in zip(pairs, anchors):
        i, j = pair["i"], pair["j"]
        delta = rotations[i] @ a + poses[i, :3] - rotations[j] @ b - poses[j, :3]
        na, nb = rotations[i] @ pair["na"], rotations[j] @ pair["nb"]
        separation = .5 * (abs(na @ delta) + abs(nb @ delta))
        result.append([i, j, float(separation), float(np.linalg.norm(delta))])
    return np.array(result).reshape(-1, 4)


def focus_values(ground, pairs, invariant, surfaces, fixed, count):
    by_ground = {(pair["i"], pair["j"]): k for k, pair in enumerate(pairs)}
    by_surface = {(pair["i"], pair["j"]): k for k, pair in enumerate(fixed)}
    result = {}
    for i, j in FOCUS:
        metrics = {}
        if j < count:
            if (i, j) in by_ground:
                k = by_ground[(i, j)]
                metrics.update(height_m=float(ground[k, 2]), normal_deg=float(ground[k, 3]),
                               invariant_point_plane_m=float(invariant[k, 2]), anchor_separation_m=float(invariant[k, 3]))
            if (i, j) in by_surface:
                row = surfaces[by_surface[(i, j)]]
                metrics.update(nn_p50_m=row[4], nn_p95_m=row[5], nn_coverage=row[6],
                               fixed_p50_m=row[7], fixed_p95_m=row[8], fixed_coverage=row[9])
        result[f"{i}_{j}"] = dict(available=j < count and bool(metrics), metrics=metrics,
                                 keyframes=[i, j], future_endpoint=j >= count)
    return result


def flatten_metrics(ground, pairs, invariant, surfaces, fixed, raw, poses, memberships=None):
    result = {}
    for partition in ("train", "heldout"):
        for scope in ("local", "outside"):
            selected = [k for k, pair in enumerate(pairs) if pair["partition"] == partition and
                        pair["scope"] == scope and pair["revisit"]]
            for name, array in (("height_m", ground[:, 2]), ("normal_deg", ground[:, 3]),
                                ("invariant_point_plane_m", invariant[:, 2])):
                for quantile, value in geometry.summary(array[selected]).items():
                    if quantile in ("p50", "p95", "max"):
                        result[f"ground.{partition}.{scope}.{name}.{quantile}"] = value
    array = np.array(surfaces).reshape(-1, 10)
    for scope in ("local", "outside"):
        selected = [k for k, pair in enumerate(fixed) if pair["scope"] == scope]
        for name, column in (("nn_p50_m", 4), ("nn_p95_m", 5), ("nn_coverage", 6),
                             ("fixed_p50_m", 7), ("fixed_p95_m", 8), ("fixed_coverage", 9)):
            for quantile, value in geometry.summary(array[selected, column]).items():
                if quantile in ("p50", "p95", "max"):
                    result[f"surface.{scope}.{name}.{quantile}"] = value
    # Same original saved-map region memberships as v2; never recrop a variant.
    for region, ids in (memberships or {}).items():
        selected = [k for k, pair in enumerate(pairs) if pair["i"] in ids and pair["j"] in ids and
                    pair["partition"] == "heldout" and pair["revisit"]]
        for name, values in (("height_m", ground[:, 2]), ("normal_deg", ground[:, 3]),
                             ("invariant_point_plane_m", invariant[:, 2])):
            for quantile, value in geometry.summary(values[selected]).items():
                if quantile in ("p50", "p95", "max"):
                    result[f"region.{region}.ground.{name}.{quantile}"] = value
        selected = [k for k, pair in enumerate(fixed) if pair["i"] in ids and pair["j"] in ids]
        for name, column in (("fixed_p50_m", 7), ("fixed_p95_m", 8), ("fixed_coverage", 9)):
            for quantile, value in geometry.summary(array[selected, column]).items():
                if quantile in ("p50", "p95", "max"):
                    result[f"region.{region}.surface.{name}.{quantile}"] = value
    residuals = geometry.odometry_rows(raw[:len(poses)], poses)
    for name, column in (("translation_log_m", 2), ("rotation_deg", 3)):
        for quantile, value in geometry.summary(residuals[:, column]).items():
            if quantile in ("p50", "p95", "max"):
                result[f"odometry.{name}.{quantile}"] = value
    return result


def evaluate(args):
    bundle, replay, graph, output, evidence = [path.resolve() for path in
        (args.bundle, args.replay, args.graph, args.output, args.evidence)]
    geometry.check_destination(output, bundle, evidence)
    if output.exists():
        raise ValueError("choose a new attribution report directory; existing evidence is preserved")
    if any(path == output or output in path.parents for path in (replay, graph)):
        raise ValueError("report output overlaps graph/replay inputs")
    manifest = geometry.validate_manifest(bundle, evidence)
    if "status=complete" not in (graph / "metadata.txt").read_text():
        raise ValueError("graph counterfactual replay has not completed")
    with (bundle / "keyframes/index.csv").open() as stream:
        index = list(csv.DictReader(stream))
    count = len(index)
    raw = np.array([[float(row[key]) for key in geometry.POSE_COLUMNS] for row in index])
    saved = geometry.read_pose_csv(bundle / "optimized_poses.csv", count)
    final_v2 = geometry.read_pose_csv(replay / "optimized_poses.csv", count)
    reference = geometry.read_pose_csv(graph / "all.csv", count)
    position_error = float(np.linalg.norm(reference[:, :3] - final_v2[:, :3], axis=1).max())
    rotation_error = float((Rotation.from_quat(reference[:, 3:]).inv() * Rotation.from_quat(final_v2[:, 3:])).magnitude().max())
    if position_error > 1e-6 or rotation_error > 1e-7:
        raise ValueError(f"fixed-factor replay does not reproduce v2: {position_error} m, {rotation_error} rad")
    pairs = json.loads((evidence / "ground_pairs.json").read_text())
    observations = json.loads((evidence / "ground_observations.json").read_text())
    regions = json.loads((evidence / "regions.json").read_text())
    memberships = {name: set(ids) for name, ids in regions["keyframes"].items()}
    anchors = make_ground_anchors(saved, pairs)
    fixed, surface_audit = geometry.freeze_surfaces(bundle, index, observations, pairs, saved, memberships)
    variants = list(csv.DictReader((graph / "variants.csv").open()))
    loops = list(csv.DictReader((replay / "loop_constraints.csv").open()))
    sources = [bundle / "manifest.sha256", bundle / "keyframes/index.csv", bundle / "config/mapping_effective.yaml",
               Path(__file__).resolve(), Path(__file__).with_name("loop_recall_geometry.py").resolve(),
               replay / "loop_constraints.csv", replay / "optimized_poses.csv", *replay.glob("snapshots/*.csv"),
               graph / "metadata.txt", graph / "variants.csv", *graph.glob("*.csv"),
               *graph.glob("snapshots/*.csv"),
               evidence / "ground_pairs.json", evidence / "ground_observations.json", evidence / "regions.json"]
    hashes = {str(path): geometry.sha256(path) for path in sources}
    output.mkdir(parents=True)
    metrics, focuses = {}, {}
    pair_csv = (output / "all_variant_fixed_pairs.csv").open("w", newline="")
    pair_writer = csv.writer(pair_csv)
    pair_writer.writerow(["variant", "kind", "i", "j", "height_m", "normal_deg", "invariant_point_plane_m",
                         "anchor_separation_m", "nn_p50_m", "nn_p95_m", "nn_coverage", "fixed_p50_m", "fixed_p95_m", "fixed_coverage"])

    def measure(poses, name=None):
        selected = [k for k, pair in enumerate(pairs) if max(pair["i"], pair["j"]) < len(poses)]
        selected_pairs = [pairs[k] for k in selected]
        selected_fixed = [pair for pair in fixed if max(pair["i"], pair["j"]) < len(poses)]
        ground = geometry.ground_rows(poses, selected_pairs)
        invariant = invariant_ground_rows(poses, selected_pairs, anchors[selected])
        surfaces = geometry.surface_rows(poses, selected_fixed)
        values = flatten_metrics(ground, selected_pairs, invariant, surfaces, selected_fixed, raw, poses, memberships)
        focus = focus_values(ground, selected_pairs, invariant, surfaces, selected_fixed, len(poses))
        if name:
            for row, inv in zip(ground, invariant):
                pair_writer.writerow([name, "ground", int(row[0]), int(row[1]), row[2], row[3], inv[2], inv[3], *[""] * 6])
            for row in surfaces:
                pair_writer.writerow([name, "non_ground", row[0], row[1], *[""] * 4, *row[4:]])
        return values, focus

    for variant in variants:
        name = variant["variant"]
        poses = geometry.read_pose_csv(graph / (name + ".csv"), count)
        metrics[name], focuses[name] = measure(poses, name)
        print("evaluated", name, flush=True)
    pair_csv.close()
    geometry.write_csv(output / "variant_metrics.csv", ["variant", "metric", "value"],
        [[name, metric, value] for name, values in metrics.items() for metric, value in values.items()])
    geometry.write_csv(output / "focus_metrics.csv", ["variant", "i", "j", "metric", "value"],
        [[name, *FOCUS[k], metric, value] for name, entries in focuses.items()
         for k, entry in enumerate(entries.values()) for metric, value in entry["metrics"].items()])
    attribution, influence_rows, immediate_rows, immediate_metric_rows = [], [], [], []
    snapshot_audit = []
    for k, loop in enumerate(loops):
        current, historical = int(loop["current_id"]), int(loop["candidate_id"])
        label, effects = classify_effect(metrics["all"], metrics[f"drop_{k}"])
        focus_effects = {}
        for pair in FOCUS:
            key = f"{pair[0]}_{pair[1]}"
            status, entries = classify_effect(focuses["all"][key]["metrics"], focuses[f"drop_{k}"][key]["metrics"])
            focus_effects[key] = dict(classification=status, effects=entries)
            for metric, effect in entries.items():
                influence_rows.append([k, current, historical, *pair, metric, effect["with_edge"],
                    effect["without_edge"], effect["degradation_delta"], effect["effect"]])
        after = geometry.read_pose_csv(graph / "snapshots" / f"after_{current}.csv", current + 1)
        without = geometry.read_pose_csv(graph / "snapshots" / f"without_{k}_at_{current}.csv", current + 1)
        original = geometry.read_pose_csv(replay / "snapshots" / f"loop_{current:07d}_to_{historical:07d}.csv", current + 1)
        error = float(np.linalg.norm(after[:, :3] - original[:, :3], axis=1).max())
        if error > 1e-6:
            raise ValueError("source-time replay snapshot differs from original committed v2 snapshot")
        snapshot_audit.append(dict(loop_index=k, source=current, position_error_max_m=error))
        after_metrics, after_focus = measure(after)
        without_metrics, without_focus = measure(without)
        prefix_label, prefix_effects = classify_effect(after_metrics, without_metrics)
        for metric, effect in prefix_effects.items():
            immediate_metric_rows.append([k, current, historical, current + 1, metric,
                effect["with_edge"], effect["without_edge"], effect["degradation_delta"], effect["effect"]])
        for pair in FOCUS:
            key = f"{pair[0]}_{pair[1]}"
            if not after_focus[key]["available"]:
                immediate_rows.append([k, current, historical, *pair, False, "future_or_no_fixed_evidence", "", "", "", "uncertain"])
                continue
            _, entries = classify_effect(after_focus[key]["metrics"], without_focus[key]["metrics"])
            for metric, effect in entries.items():
                immediate_rows.append([k, current, historical, *pair, True, metric, effect["with_edge"],
                    effect["without_edge"], effect["degradation_delta"], effect["effect"]])
        attribution.append(dict(loop_index=k, source=current, history=historical,
            classification=label, effects=effects, focuses=focus_effects,
            immediate_prefix_classification=prefix_label, immediate_nodes=current + 1,
            immediate_prefix_effects=prefix_effects,
            immediate_focus_available={name: value["available"] for name, value in after_focus.items()}))
    geometry.write_csv(output / "loop_focus_final_influence.csv",
        ["loop_index", "current", "history", "focus_i", "focus_j", "metric", "with_edge", "without_edge", "degradation_delta", "effect"], influence_rows)
    geometry.write_csv(output / "loop_focus_immediate_influence.csv",
        ["loop_index", "current", "history", "focus_i", "focus_j", "available_at_source", "metric", "with_edge", "without_edge", "degradation_delta", "effect"], immediate_rows)
    geometry.write_csv(output / "loop_metric_immediate_influence.csv",
        ["loop_index", "current", "history", "available_nodes", "metric", "with_edge", "without_edge", "degradation_delta", "effect"], immediate_metric_rows)
    geometry.write_csv(output / "loop_classifications.csv", ["loop_index", "current", "history", "classification", "immediate_prefix_classification"],
        [[row["loop_index"], row["source"], row["history"], row["classification"], row["immediate_prefix_classification"]] for row in attribution])
    groups = {}
    for variant in variants:
        name = variant["variant"]
        if name.startswith("drop_group_"):
            groups[name] = dict(excluded_indices=variant["excluded_loop_indices"], focuses=focuses[name], metrics=metrics[name])
    after_hashes = {str(path): geometry.sha256(path) for path in sources}
    if hashes != after_hashes:
        raise ValueError("read-only attribution inputs changed")
    report = dict(schema_version=1, limitations=LIMITATIONS, frames=count, actual_loops=len(loops),
        original_manifest=manifest, fixed_surface_audit=surface_audit, ground_pairs=len(pairs),
        full_replay_v2_position_error_max_m=position_error, full_replay_v2_rotation_error_max_rad=rotation_error,
        original_snapshot_checks=snapshot_audit, inputs_sha256=hashes, inputs_unchanged=True,
        variant_metrics=metrics, variant_focus_metrics=focuses, loop_attribution=attribution, group_ablations=groups,
        classification_rule="Per-metric sign only; no adding metres/degrees; conflicting evidence -> uncertain; tolerance 1e-9 numerical only",
        acceptance="not decided; attribution is not an online oracle and does not establish physical loop correctness")
    (output / "attribution_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(dict(output=str(output), loops=len(loops), variants=len(variants), original_snapshot_checks=len(snapshot_audit))))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("replay", type=Path)
    parser.add_argument("graph", type=Path)
    parser.add_argument("--evidence", type=Path, default=Path("artifacts/map_repair_1016"))
    parser.add_argument("--output", type=Path, required=True)
    evaluate(parser.parse_args())


if __name__ == "__main__":
    main()
