#!/usr/bin/env python3
"""Offline v4/native state audit, standard library only. Never starts ROS."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
from html import escape
import json
import math
from pathlib import Path
import statistics
import sys

STATE_FIELDS = ("tx ty tz qx qy qz qw ext_tx ext_ty ext_tz ext_qx ext_qy ext_qz ext_qw "
                "g_W_x g_W_y g_W_z bg_x bg_y bg_z ba_x ba_y ba_z").split()


def finite(values, context):
    try:
        result = [float(v) for v in values]
    except ValueError as error:
        raise ValueError(f"{context}: invalid numeric field") from error
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{context}: nonfinite numeric field")
    return result


def rotate(q, v):
    x, y, z, w = q
    ux, uy, uz = 2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0])
    return [v[0]+w*ux+y*uz-z*uy, v[1]+w*uy+z*ux-x*uz, v[2]+w*uz+x*uy-y*ux]


def derive(row):
    for prefix in ["", "ext_"]:
        q = [row[prefix + name] for name in ["qx", "qy", "qz", "qw"]]
        length = math.hypot(*q)
        if abs(length - 1) > .01:
            raise ValueError("quaternion norm must be within .01 of unity")
        q = [v / length for v in q]
        for name, value in zip(["qx", "qy", "qz", "qw"], q):
            row[prefix + name] = value
    q = [row[n] for n in ["qx", "qy", "qz", "qw"]]
    gravity = [row["g_W_" + n] for n in "xyz"]
    body = rotate([-q[0], -q[1], -q[2], q[3]], gravity)
    for name, value in zip("xyz", body):
        row["g_I_" + name] = value
    row["g_norm_m_s2"] = math.hypot(*gravity)
    row["world_z_down_misalignment_rad"] = (math.acos(max(-1., min(1., -gravity[2] / row["g_norm_m_s2"])))
        if row["g_norm_m_s2"] > 0 else None)
    row["bg_norm_rad_s"] = math.hypot(*(row["bg_" + n] for n in "xyz"))
    row["ba_norm_normalized_m_s2"] = math.hypot(*(row["ba_" + n] for n in "xyz"))
    x, y, z, w = q
    row["roll_rad"] = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    row["pitch_rad"] = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    row["yaw_rad"] = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return row


def parse_trace(lines):
    rows = []
    for number, line in enumerate(lines, 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 25 or parts[1] not in ["predicted", "updated"]:
            raise ValueError(f"trace line {number}: expected25 columns and predicted/updated")
        values = finite([parts[0], *parts[2:]], f"trace line {number}")
        row = dict(zip(STATE_FIELDS, values[1:]))
        row.update(time_s=values[0], phase=parts[1], source_line=number,
                   time_basis="sensor_scan_end_double_seconds")
        rows.append(derive(row))
    if not rows:
        raise ValueError("trace has no states")
    return rows


def rpy_quaternion(roll, pitch, yaw):
    cr, cp, cy = (math.cos(v / 2) for v in [roll, pitch, yaw])
    sr, sp, sy = (math.sin(v / 2) for v in [roll, pitch, yaw])
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def parse_native(lines):
    rows = []
    for number, line in enumerate(lines, 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 26:
            raise ValueError(f"native line {number}: mat_out must have26 columns")
        v = finite(parts, f"native line {number}")
        q, eq = rpy_quaternion(*(n / 57.3 for n in v[1:4])), rpy_quaternion(*(n / 57.3 for n in v[7:10]))
        if v[25] < 0 or not v[25].is_integer():
            raise ValueError("native point count must be nonnegative integer")
        row = dict(zip(STATE_FIELDS, [*v[4:7], *q, *v[10:13], *eq, *v[22:25], *v[16:19], *v[19:22]]))
        row.update(time_s=v[0], phase="native_updated", source_line=number, point_count=int(v[25]),
                   time_basis="relative_scan_begin_label_state_at_scan_end")
        rows.append(derive(row))
    if not rows:
        raise ValueError("native file has no states")
    return rows


def parse_sensor(lines):
    result = {"imu": [], "cloud": [], "sync": []}
    for number, line in enumerate(lines, 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        p = line.split()
        kind = p[0]
        if kind == "imu" and len(p) == 8:
            v = finite(p[1:], f"sensor line {number}")
            result[kind].append({"time_s": v[0], "acc": v[1:4], "gyro": v[4:7]})
        elif kind == "cloud" and len(p) == 4:
            v = finite(p[1:2], f"sensor line {number}")
            width, digest = int(p[2]), int(p[3])
            if width < 0 or not 0 <= digest < 2**64:
                raise ValueError("cloud width/hash outside unsigned range")
            result[kind].append({"time_s": v[0], "width": width, "data_fnv1a64": str(digest)})
        elif kind == "sync" and len(p) >= 4:
            count = int(p[3])
            v = finite([p[1], p[2], *p[4:]], f"sensor line {number}")
            if count < 0 or len(p) != 4 + count or v[1] < v[0]:
                raise ValueError(f"sensor line {number}: invalid sync count/bounds")
            result[kind].append({"begin_s": v[0], "end_s": v[1], "imu_count": count, "imu_times_s": v[2:]})
        else:
            raise ValueError(f"sensor line {number}: unknown record/wrong width")
    return result


def startup_statistics(sensor, cutoff_s, gyro_unit, accel_unit, gyro_limit):
    result = {"definition": "sensor_timestamp_prefix_through_first_observed_predicted_end_not_internal_init_window",
        "cutoff_sensor_s": cutoff_s, "accel_raw_unit": accel_unit, "gyro_raw_unit": gyro_unit,
        "internal_initialization_window": {"status": "missing", "internal_N": None,
            "reason": "v4 trace does not log exact initialization membership, counter, or raw-to-filter selection"},
        "sample_count": None, "span_s": None, "gyro_mean_norm_rad_s": None, "gyro_mean_norm_check_pass": None,
        "gyro_mean_norm_limit_rad_s": gyro_limit, "quality_gate_pass": None,
        "not_a_stationarity_certificate": True}
    if sensor is None or cutoff_s is None:
        result["status"] = "missing_sensor_or_absolute_trace_cutoff"
        return result
    samples = [row for row in sensor["imu"] if row["time_s"] <= cutoff_s]
    result["sample_count"] = len(samples)
    if not samples:
        result["status"] = "missing_samples_before_cutoff"
        return result
    times = [r["time_s"] for r in samples]
    delta = [b-a for a, b in zip(times, times[1:])]
    ordered = sorted(set(times))
    result.update(status="proxy_only", first_sensor_s=min(times), last_sensor_s=max(times),
        span_s=max(times)-min(times), duplicate_timestamp_count=len(times)-len(set(times)),
        backward_timestamp_count=sum(dt < 0 for dt in delta),
        maximum_sorted_gap_s=max((b-a for a,b in zip(ordered, ordered[1:])), default=None))
    for name in ["acc", "gyro"]:
        vectors = [[r[name][axis] for r in samples] for axis in range(3)]
        result[name + "_mean_raw"] = [statistics.fmean(values) for values in vectors]
        result[name + "_variance_population_raw"] = [statistics.pvariance(values) for values in vectors]
    result["acc_mean_norm_raw"] = math.hypot(*result["acc_mean_raw"])
    if gyro_unit != "unknown":
        scale = math.pi/180 if gyro_unit == "deg/s" else 1.
        result["gyro_mean_norm_rad_s"] = math.hypot(*result["gyro_mean_raw"]) * scale
        result["gyro_mean_norm_check_pass"] = result["gyro_mean_norm_rad_s"] <= gyro_limit
    return result


def svg_chart(rows, panels, title, provenance):
    width, panel_height = 1050, 190
    height = 70 + panel_height * len(panels)
    times = [r["time_s"] for r in rows]
    lo, hi = min(times), max(times)
    span = hi-lo or 1.
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="20" y="22" font-family="sans-serif" font-size="16">{escape(title)} | {escape(provenance)} | state, not truth</text>',
        f'<text x="20" y="44" font-family="sans-serif" font-size="12">time origin {lo:.12g} s; solid updated / dashed predicted; see summary time basis</text>']
    palette = ["#a92624", "#217a35", "#2456a5", "#7d3690"]
    for index, (label, fields) in enumerate(panels):
        top = 80 + index*panel_height
        values = [r[f] for r in rows for f in fields if r.get(f) is not None]
        ymin, ymax = min(values), max(values)
        padding = (ymax-ymin)*.08 or .01
        ymin, ymax = ymin-padding, ymax+padding
        parts += [f'<text x="20" y="{top}" font-family="sans-serif" font-size="12">{escape(label)} [{ymin:.5g}, {ymax:.5g}]</text>',
                  f'<path d="M70 {top+15} V{top+140} H1020" fill="none" stroke="#888"/>']
        for field_index, field in enumerate(fields):
            color = palette[field_index % len(palette)]
            parts.append(f'<text x="{160+field_index*210}" y="{top}" font-size="12" fill="{color}">{escape(field)}</text>')
            for phase in sorted({r["phase"] for r in rows}):
                points = [(70+(r["time_s"]-lo)/span*950, top+140-(r[field]-ymin)/(ymax-ymin)*120)
                          for r in rows if r["phase"] == phase and r.get(field) is not None]
                # One marker per one-point series, no invented interpolation to a different phase.
                if len(points) == 1:
                    x,y = points[0]
                    parts.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="2" fill="{color}"/>')
                elif points:
                    dash = ' stroke-dasharray="5,3"' if phase == "predicted" else ""
                    coordinate = " ".join(f"{x:.3f},{y:.3f}" for x,y in points)
                    parts.append(f'<polyline points="{coordinate}" fill="none" stroke="{color}" stroke-width="1"{dash}/>')
    parts.append("</svg>")
    return "\n".join(parts)


def file_identity(path):
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, help="v4 dbg.txt exact state trace")
    parser.add_argument("--native-mat-out", type=Path, help="optional native26-column low-precision state")
    parser.add_argument("--sensor-audit", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="new directory, never overwritten")
    parser.add_argument("--provenance", choices=["synthetic", "real"], required=True)
    parser.add_argument("--provenance-note", required=True, help="capture/generator and run identity; assertion is not verified truth")
    parser.add_argument("--gyro-unit", choices=["unknown", "rad/s", "deg/s"], default="unknown")
    parser.add_argument("--accel-unit", choices=["unknown", "m/s^2", "g"], default="unknown")
    parser.add_argument("--gyro-mean-limit-rad-s", type=float, default=.5, help="diagnostic norm check, NOT a stationarity gate")
    parser.add_argument("--producer-patch", type=Path)
    parser.add_argument("--producer-build-id", default="unknown")
    args = parser.parse_args(argv)
    try:
        if not (args.trace or args.native_mat_out):
            raise ValueError("supply --trace and/or --native-mat-out")
        if not args.provenance_note.strip() or not math.isfinite(args.gyro_mean_limit_rad_s) or args.gyro_mean_limit_rad_s < 0:
            raise ValueError("nonempty provenance-note and finite nonnegative gyro limit required")
        inputs = [p for p in [args.trace, args.native_mat_out, args.sensor_audit, args.producer_patch] if p]
        output = args.output.resolve()
        if any(output == p.resolve() or output in p.resolve().parents for p in inputs):
            raise ValueError("output overlaps input path/ancestor")
        if output.exists():
            raise ValueError("output already exists; refusing overwrite")
        rows = parse_trace(args.trace.read_text().splitlines()) if args.trace else []
        native = parse_native(args.native_mat_out.read_text().splitlines()) if args.native_mat_out else []
        sensor = parse_sensor(args.sensor_audit.read_text().splitlines()) if args.sensor_audit else None
        warnings = ["source provenance is a caller assertion, not independent truth",
            "FAST-LIO internally scales acceleration by9.81/initial_mean_norm; raw acceleration units and ba domain are not interchangeable",
            "exact internal initialization window/N/quality are missing; startup prefix is only a sensor-timestamp proxy, not log-arrival or selection membership",
            "v4 time uses double seconds, not lossless original integer sec/nanosec"]
        if args.accel_unit == "unknown":
            warnings.append("raw acceleration unit unknown; no inferred SI acceleration quality check")
        if args.gyro_unit == "unknown":
            warnings.append("raw gyro unit unknown; SI norm check missing")
        if args.producer_build_id == "unknown":
            warnings.append("producer build identity unknown")
        predicted = [r for r in rows if r["phase"] == "predicted"]
        updated = [r for r in rows if r["phase"] == "updated"]
        if not predicted:
            warnings.append("missing predicted trace; startup cutoff missing")
        if not updated:
            warnings.append("missing updated trace")
        if sensor is None:
            warnings.append("missing sensor_audit; IMU statistics unavailable")
        if native:
            warnings.append("native mat_out LOW PRECISION (~6 significant digits), relative scan BEGIN label for scan END state; no absolute historical time is synthesized; RPY usesrad*57.3")
        backward = sum(b["time_s"] < a["time_s"] for a,b in zip(rows, rows[1:]))
        duplicates = len(rows)-len({(r["time_s"], r["phase"]) for r in rows})
        if backward or duplicates:
            warnings.append("trace negative temporal quality: backward or duplicate same-phase timestamps preserved")
        cutoff = min((r["time_s"] for r in predicted), default=None)
        startup = startup_statistics(sensor, cutoff, args.gyro_unit, args.accel_unit, args.gyro_mean_limit_rad_s)
        if startup["gyro_mean_norm_check_pass"] is False:
            warnings.append("startup proxy gyro norm exceeds diagnostic warning limit; not an internal initialization quality verdict")
        if startup.get("backward_timestamp_count", 0) or startup.get("duplicate_timestamp_count", 0):
            warnings.append("startup proxy has backward or duplicate IMU timestamps; preserved, not silently corrected")
        summary = {"schema": "a2w_frontend_state_audit_v1", "created_utc": datetime.now(timezone.utc).isoformat(),
            "tool": file_identity(Path(__file__)), "provenance": {"kind": args.provenance, "note": args.provenance_note,
                "verified_independently": False, "producer_build_id": args.producer_build_id},
            "frames": {"W": "camera_init", "I": "body", "L": "LiDAR"},
            "conventions": {"pose": "T_W_I", "extrinsic": "T_I_L", "gravity": "g_W; g_I=inverse(R_W_I)*g_W",
                "bias": "bg/ba in I; bg rad/s, ba normalized-acc m/s^2 domain", "quaternion": "xyzw", "variance": "population ddof=0"},
            "inputs": [file_identity(p) for p in inputs], "trace": {"format": "isolated_v4_dbg_25_columns",
                "state_count": len(rows), "predicted_count": len(predicted), "updated_count": len(updated),
                "backward_timestamp_count": backward, "duplicate_same_phase_count": duplicates},
            "native_state_count": len(native), "sensor_record_counts": {k:len(v) for k,v in sensor.items()} if sensor else None,
            "startup_imu_proxy": startup, "warnings": warnings, "no_ros_or_motion": True}
        output.mkdir(parents=True)
        def write_csv(name, records):
            if not records:
                return
            fields = list(dict.fromkeys(key for row in records for key in row))
            with (output / name).open("x", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(records)
        write_csv("states.csv", rows)
        write_csv("native_states.csv", native)
        write_csv("startup_imu_proxy.csv", [{"time_s": r["time_s"], **dict(zip(["acc_x_raw","acc_y_raw","acc_z_raw","gyro_x_raw","gyro_y_raw","gyro_z_raw"], [*r["acc"],*r["gyro"]]))}
            for r in sensor["imu"] if cutoff is not None and r["time_s"] <= cutoff] if sensor else [])
        for prefix, records in [("", rows), ("native_", native)]:
            if not records:
                continue
            for name, panels in [
                ("gravity", [("gravity W m/s^2", ["g_W_x","g_W_y","g_W_z"]), ("gravity I m/s^2",["g_I_x","g_I_y","g_I_z"])]),
                ("bias", [("bg I rad/s",["bg_x","bg_y","bg_z"]), ("ba I normalized m/s^2",["ba_x","ba_y","ba_z"])]),
                ("extrinsic", [("T_I_L translation m",["ext_tx","ext_ty","ext_tz"]), ("R_I_L quaternion xyzw",["ext_qx","ext_qy","ext_qz","ext_qw"])])]:
                (output / (prefix + name + ".svg")).write_text(svg_chart(records, panels, prefix + name, args.provenance))
        (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
        (output / "WARNINGS.txt").write_text("\n".join(warnings)+"\n")
        print(json.dumps({"output": str(args.output), "states": len(rows), "native_states": len(native),
            "startup_proxy_count": startup["sample_count"], "internal_initialization": "missing"}))
        return 0
    except (OSError, ValueError) as error:
        print(f"frontend_state_audit: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
