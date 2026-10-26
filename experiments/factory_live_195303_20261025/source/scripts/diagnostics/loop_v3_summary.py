#!/usr/bin/env python3
"""Read-only replay statistics. No loop truth labels, no synthetic wall queue."""
import argparse
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

DEFAULT_RUNS = {
    "baseline_live_full": Path("artifacts/loop_recall_v3_20261020/baseline_live_full"),
    "scheduled_live_full_paced": Path("artifacts/loop_recall_v3_20261020/scheduled_live_full_paced"),
    "enhanced_v2_full": Path("artifacts/loop_recall_20261020/enhanced_v2_full"),
}
FRAME_STAGES = ["process_wall_ms", "pipeline_frame_total_ms", "descriptor_ms", "scan_context_ms", "gtsam_ms"]
EVENT_STAGES = ["target_local_map_ms", "quatro_ms", "nano_gicp_ms", "evidence_ms", "validator_ms", "registration_total_ms"]
REQUIRED = {
    "frames": {"current", "stamp_ns", "process_wall_ms", "scan_context_ms"},
    "events": {"current", "candidate", "accepted", "graph_committed", "reason"},
    "queries": {"current", "query_number", "candidate", "raw_result_count"},
    "loop_constraints": {"current_id", "candidate_id"},
    "summary": {"metric", "value"},
}


def number(value):
    if value is None or str(value).strip() == "":
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def integer(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def yes(value):
    return str(value).lower() in ["1", "true"]


def distribution(values):
    converted = [number(v) for v in values]
    invalid = sum(v is None for v in converted)
    values = sorted(v for v in converted if v is not None)
    def quantile(fraction):
        if not values:
            return None
        position = (len(values)-1)*fraction
        low = int(position)
        high = min(low+1, len(values)-1)
        return values[low]+(values[high]-values[low])*(position-low)
    return {"count": len(values), "missing_or_nonfinite_count": invalid,
            "sum": sum(values) if values else None, "mean": sum(values)/len(values) if values else None,
            "p50": quantile(.5), "p95": quantile(.95), "max": max(values) if values else None}


def read_csv(path, required):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not required.issubset(set(reader.fieldnames or [])):
            raise ValueError(f"{path.name}: missing required columns {sorted(required-set(reader.fieldnames or []))}")
        rows = list(reader)
    if any(None in row for row in rows):
        raise ValueError(f"{path.name}: extra columns or unquoted delimiter")
    return rows


def metadata(path):
    result = {}
    for line in path.read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value  # terminal status supersedes the initial status.
    return result


def identity(path):
    return {"path": str(path), "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def stage_distribution(rows, field, active=False):
    if not any(field in row for row in rows):
        return distribution([])
    values = [row.get(field) for row in rows]
    if active:
        values = [value for value in values if number(value) is not None and number(value) > 0]
    return distribution(values)


def actual_wall(path, flags):
    empty = {"status": "missing", "source": "pace_frames.csv/commit_latency.csv only; never simulated from process costs",
        "real_transport_queue": False, "preloaded_memory_pacing": None,
        "start_latency_ms": distribution([]), "end_latency_ms": distribution([]), "commit_latency_ms": distribution([]),
        "service_with_primary_io_ms": distribution([]),
        "maximum_pending": None, "queue_capacity": None, "capacity_acceptance": "missing",
        "end_waiting_capacity_check": "missing", "start_including_active_capacity_upper_bound_check": "missing",
        "continuous_queue_bound_established": False,
        "paced_rows": 0, "frame_rows": 0, "cold_load_included": None}
    pace = path / "pace_frames.csv"
    if not pace.exists():
        return empty
    rows = read_csv(pace, {"current", "pace_enabled", "start_wall_latency_ms", "end_wall_latency_ms"})
    service_costs = []
    for row in rows:
        start, end = number(row.get("actual_start_wall_ms")), number(row.get("actual_end_after_io_wall_ms"))
        cost = end-start if start is not None and end is not None else None
        service_costs.append(cost if cost is not None and cost >= 0 else None)
    empty["service_with_primary_io_ms"] = distribution(service_costs)
    empty["service_scope"] = "actual_end_after_io_wall_ms - actual_start_wall_ms; includes primary CSV/snapshot IO, excludes active pacing sleep and later timing-row/console/interframe gaps; not OS cold load"
    active = [row for row in rows if yes(row["pace_enabled"])]
    empty["frame_rows"] = len(rows)
    if not active:
        empty.update(status="not_paced", capacity_acceptance="not_assessed")
        return empty
    required = {"arrived_unprocessed_at_start", "arrived_waiting_after_end", "queue_capacity", "capacity_exceeded"}
    if not required.issubset(set(active[0])):
        raise ValueError("pace_frames.csv: active pace rows missing queue columns")
    capacities = {integer(r["queue_capacity"]) for r in active}
    # Producer cap flag samples END waiting only. START includes the frame about
    # to become active and is a separate conservative upper bound. Neither
    # sampled endpoint proves the continuous backlog during timing/console gaps.
    pending = [integer(r["arrived_waiting_after_end"]) for r in active]
    start_counts = [integer(r["arrived_unprocessed_at_start"]) for r in active]
    valid_pending = [v for v in pending if v is not None]
    valid_start_counts = [v for v in start_counts if v is not None]
    cap = next(iter(capacities)) if len(capacities) == 1 else None
    exceeded = any(yes(r["capacity_exceeded"]) for r in active)
    exceeded = exceeded or (cap is not None and any(v > cap for v in valid_pending))
    if cap != 32:
        flags.append("pace_queue_capacity_not_32_or_missing")
    if exceeded:
        flags.append("pace_capacity_exceeded")
    if len(valid_pending) != len(pending):
        flags.append("pace_pending_values_missing")
    start_exceeds = cap is not None and any(v > cap for v in valid_start_counts)
    if start_exceeds:
        flags.append("pace_start_capacity_upper_bound_exceeded_not_proof_of_true_waiting_overflow")
    end_check = "FAIL" if exceeded else "PASS" if cap == 32 and len(valid_pending) == len(pending) else "missing"
    start_check = ("EXCEEDS_UPPER_BOUND" if start_exceeds else "PASS"
                   if cap == 32 and len(valid_start_counts) == len(start_counts) else "missing")
    commits = path / "commit_latency.csv"
    commit_rows = read_csv(commits, {"current", "candidate", "pace_enabled", "commit_wall_latency_ms"}) if commits.exists() else []
    empty.update(status="measured", preloaded_memory_pacing=True, paced_rows=len(active), cold_load_included=False,
        start_latency_ms=stage_distribution(active,"start_wall_latency_ms"),
        end_latency_ms=stage_distribution(active,"end_wall_latency_ms"),
        commit_latency_ms=stage_distribution([r for r in commit_rows if yes(r["pace_enabled"])], "commit_wall_latency_ms"),
        maximum_pending=max(valid_pending, default=None), queue_capacity=cap,
        maximum_unprocessed_at_start_including_next_active=max((v for v in start_counts if v is not None), default=None),
        end_waiting_capacity_check=end_check,
        start_including_active_capacity_upper_bound_check=start_check,
        producer_end_sample_capacity_exceeded=exceeded,
        capacity_acceptance="FAIL" if exceeded else "NOT_ESTABLISHED" if end_check == "PASS" else "missing",
        commit_pairs=[[integer(r["current"]),integer(r["candidate"])] for r in commit_rows if yes(r["pace_enabled"])],
        epoch_note="producer contract: steady epoch after Bundle/algorithm preload, before first input; end includes primary CSV/snapshot flush, not timing-row write",
        memory_queue_note="frozen sensor timestamp arrival-index backlog; no ROS/transport bounded queue; all input frames retained even above capacity",
        capacity_sampling_note="end samples exclude completed active; start count includes next active and is a conservative upper bound; timing-row/console/interframe gaps are not continuously sampled; producer PASS is not bounded-queue acceptance")
    return empty


def summarize_run(path):
    path = Path(path)
    base = {"path": str(path), "status": "waiting", "errors": [], "non_pass_markers": [],
        "truth": {"independent_ground_truth": False, "true_count": None, "false_count": None,
                  "meaning": "提交是图优化事件，不是独立标签；几何回归不可换算真假回环数量"}}
    if not path.is_dir():
        base["waiting_reason"] = "目录尚不存在，未产生scheduled结果"
        return base
    try:
        missing = [name+".csv" for name in REQUIRED if not (path/(name+".csv")).is_file()]
        if not (path/"run_metadata.txt").is_file():
            missing.append("run_metadata.txt")
        if missing:
            base.update(status="waiting", waiting_reason="运行尚未提供完整文件", missing_files=missing)
            return base
        tables = {name: read_csv(path/(name+".csv"),fields) for name,fields in REQUIRED.items()}
        frames, events, queries, constraints = (tables[k] for k in ["frames","events","queries","loop_constraints"])
        meta = metadata(path/"run_metadata.txt")
        saved_summary = {r["metric"]:r["value"] for r in tables["summary"]}
        flags = base["non_pass_markers"]
        committed = [r for r in events if yes(r["graph_committed"])]
        pairs = Counter((integer(r["current"]),integer(r["candidate"])) for r in committed)
        constraint_pairs = Counter((integer(r["current_id"]),integer(r["candidate_id"])) for r in constraints)
        if pairs != constraint_pairs:
            flags.append("constraint_event_commit_mismatch")
        if integer(saved_summary.get("committed_loops")) != len(committed):
            flags.append("saved_summary_commit_count_mismatch")
        if integer(saved_summary.get("frames")) != len(frames):
            flags.append("saved_summary_frame_count_mismatch")
        frame_by_id = {integer(r["current"]):r for r in frames}
        if len(frame_by_id) != len(frames) or None in frame_by_id:
            flags.append("duplicate_or_invalid_frame_ids")
        if "last_id" in meta:
            expected = integer(meta["last_id"])
            if expected is not None and set(frame_by_id) != set(range(expected+1)):
                flags.append("missing_or_nonsequential_frame_ids")
        if any(yes(r.get("matching_suppressed")) for r in frames) or integer(meta.get("first_match_id","0")) != 0:
            flags.append("window_diagnostic_not_full_sequence_acceptance")
        stamps = [integer(r["stamp_ns"]) for r in frames]
        valid_stamps = [t for t in stamps if t is not None]
        if len(valid_stamps) != len(stamps) or any(b <= a for a,b in zip(valid_stamps,valid_stamps[1:])):
            flags.append("invalid_or_nonincreasing_sensor_stamps")
        first_stamp = min(valid_stamps) if valid_stamps else None
        def event_record(row):
            current = integer(row["current"])
            stamp = integer(row.get("stamp_ns"))
            if stamp is None:
                stamp = integer(frame_by_id.get(current,{}).get("stamp_ns"))
            result = {"current": current, "candidate": integer(row["candidate"]), "stamp_ns": stamp,
                "sensor_elapsed_s": (stamp-first_stamp)/1e9 if stamp is not None and first_stamp is not None else None,
                "reason": row["reason"]}
            for key in ["fitness","overlap","correspondences","translation_difference_m","rotation_difference_rad",
                        "relative_roll_rad","relative_pitch_rad","relative_yaw_rad","confirmation_count"]:
                result[key] = number(row.get(key))
            return result
        commit_records = sorted([event_record(r) for r in committed], key=lambda r:r["current"])
        origin_count = integer(meta.get("loop_pipeline.origin_keyframe_count"))
        origin_count_source = "metadata"
        if origin_count is None:
            origin_count,origin_count_source = 31,"documented experiment default (metadata missing)"
        origin_commits = [r for r in commit_records if r["candidate"] is not None and r["candidate"] < origin_count]
        window_commits = [r for r in commit_records if 617 <= r["current"] <= 646]
        origin_window_commits = [r for r in window_commits if r["candidate"] < origin_count]
        geometry = [r for r in events if any((number(r.get(key)) or 0)>0 for key in ["quatro_ms","nano_gicp_ms","registration_total_ms"])]
        def metric_pass(row):
            limits = [number(meta.get("loop_validation.maximum_fitness")), number(meta.get("loop_validation.minimum_overlap")),
                      number(meta.get("loop_validation.minimum_correspondences"))]
            values = [number(row.get("fitness")),number(row.get("overlap")),number(row.get("correspondences"))]
            return (all(v is not None for v in [*limits,*values]) and yes(row.get("registration_success")) and
                yes(row.get("registration_converged")) and values[0] <= limits[0] and values[1] >= limits[1] and values[2] >= limits[2])
        groups = defaultdict(list)
        for row in queries:
            groups[(integer(row["current"]),integer(row["query_number"]))].append(row)
        counts = {"frames":len(frames), "event_rows":len(events), "query_calls":len(groups),
            "nonempty_query_calls":sum(any(r["candidate"]!="" for r in group) for group in groups.values()),
            "raw_query_candidate_rows":sum(r["candidate"]!="" for r in queries),
            "anchor_additional_query_calls":sum(key[1] is not None and key[1]>0 for key in groups),
            "candidate_event_rows":sum((integer(r.get("candidates")) or 0)>0 for r in events),
            "candidate_event_frames":len({r["current"] for r in events if (integer(r.get("candidates")) or 0)>0}),
            "geometry_attempts":len(geometry), "quatro_invocations":sum((number(r.get("quatro_ms")) or 0)>0 for r in events),
            "nano_gicp_invocations":sum((number(r.get("nano_gicp_ms")) or 0)>0 for r in events),
            "fine_converged_rows":sum(yes(r.get("registration_success")) and yes(r.get("registration_converged")) for r in geometry),
            "metric_gate_pass_rows":sum(metric_pass(r) for r in geometry),
            "pose_guard_pass_rows":sum(yes(r.get("has_pose_check")) and yes(r.get("position_pass")) and yes(r.get("rotation_pass")) for r in geometry),
            "joint_geometry_pose_pass_rows":sum(metric_pass(r) and yes(r.get("has_pose_check")) and yes(r.get("position_pass")) and yes(r.get("rotation_pass")) for r in geometry),
            "accepted_event_rows":sum(yes(r["accepted"]) for r in events),
            "committed_loops":len(committed), "constraint_rows":len(constraints),
            "start_window_geometry_attempts":sum(617<=integer(r["current"])<=646 for r in geometry),
            "start_window_commits":len(window_commits), "start_window_origin_commits":len(origin_window_commits)}
        cached_values = {yes(r.get("sc_cached",meta.get("sc_cached","0"))) for r in frames}
        cached = any(cached_values) or yes(meta.get("sc_cached"))
        if len(cached_values)>1:
            flags.append("mixed_sc_cache_flags")
        frame_timings = {key:stage_distribution(frames,key) for key in FRAME_STAGES}
        excluding_sc = [max(0.,number(r["process_wall_ms"])-number(r["scan_context_ms"])) for r in frames
                        if number(r["process_wall_ms"]) is not None and number(r["scan_context_ms"]) is not None]
        frame_timings["process_excluding_sc_ms"] = distribution(excluding_sc)
        active_ids = {integer(r["current"]) for r in geometry}
        active_frame_timings = {key:stage_distribution([r for r in frames if integer(r["current"]) in active_ids],key)
                                for key in FRAME_STAGES}
        event_timings = {key:stage_distribution(events,key,True) for key in EVENT_STAGES}
        source_map_by_frame = defaultdict(float)
        for row in events:
            source_map_by_frame[row["current"]] = max(source_map_by_frame[row["current"]],number(row.get("source_local_map_ms")) or 0.)
        event_timings["source_local_map_ms_once_per_frame_max"] = distribution([v for v in source_map_by_frame.values() if v>0])
        process_sum = frame_timings["process_wall_ms"]["sum"]
        valid_process_costs = frame_timings["process_wall_ms"]["count"] == len(frames) and all(
            number(r["process_wall_ms"]) is not None and number(r["process_wall_ms"]) >= 0 for r in frames)
        if not valid_process_costs:
            flags.append("invalid_frame_process_times")
        replay_wall = number(saved_summary.get("replay_wall_ms_including_csv_snapshot_io"))
        sensor_span = (max(valid_stamps)-min(valid_stamps))/1e9 if len(valid_stamps)>1 else None
        actual = actual_wall(path,flags)
        if actual["status"] == "measured":
            if actual["frame_rows"] != len(frames): flags.append("pace_frame_row_count_mismatch")
            if Counter(map(tuple,actual.get("commit_pairs",[]))) != pairs: flags.append("pace_commit_pair_mismatch")
        service_stats = actual["service_with_primary_io_ms"]
        service_fps = (len(frames)*1000/service_stats["sum"] if service_stats["sum"] and
            service_stats["count"] == len(frames) and actual["frame_rows"] == len(frames) else None)
        process_fps = len(frames)*1000/process_sum if process_sum and valid_process_costs else None
        arrival_fps = (len(frames)-1)/sensor_span if sensor_span else None
        sleep_limited = actual["status"] == "measured" or yes(meta.get("pace_enabled"))
        baseline_path = path/"run_metadata.txt"
        base.update(status="complete" if meta.get("status")=="complete" else "running", metadata=meta,
            producer_summary=saved_summary,
            inputs=[identity(path/(name+".csv")) for name in REQUIRED]+[identity(baseline_path)]+
                   [identity(path/name) for name in ["pace_frames.csv","commit_latency.csv"] if (path/name).exists()],
            counts=counts, reason_counts=dict(Counter(r["reason"] for r in events)),
            rejection_stage_counts=dict(Counter(r.get("rejection_stage","missing") for r in events)),
            reason_frame_counts={reason:len({r["current"] for r in events if r["reason"]==reason}) for reason in {r["reason"] for r in events}},
            frame_timings_ms=frame_timings, geometry_active_frame_timings_ms=active_frame_timings,
            event_work_timings_ms_positive_invocations=event_timings,
            timing_denominators={"frame": "one row per processed frame, including zero-work frames",
                "active_frame": "frames containing actual registration invocation",
                "event_work": "positive timed invocation rows, not all rejected/empty candidates",
                "shared_event_fields": "SC/descriptor/GTSAM use frames.csv; source map uses maximum once per frame; never sum repeated shared candidate-stage times"},
            sc_timing={"cached":cached,"online_comparable":not cached,
                "interpretation":"SC缓存lookup耗时不是在线SC速度；不得直接与live检索比较" if cached else "实际在线SC query计时；整段仍是只读预加载回放，不是机器人实时验收"},
            throughput={"process_only_fps":process_fps,
                "process_wall_sum_ms":process_sum,"process_excludes_csv_snapshot_io":True,
                "service_with_primary_io_fps":service_fps,
                "service_with_primary_io_sum_ms":service_stats["sum"] if service_fps is not None else None,
                "process_to_sensor_mean_rate_ratio":process_fps/arrival_fps if process_fps and arrival_fps else None,
                "service_with_primary_io_to_sensor_mean_rate_ratio":service_fps/arrival_fps if service_fps and arrival_fps else None,
                "elapsed_rate_includes_pacing_sleep":sleep_limited,
                "headroom_scope":"n/sum(process_wall_ms) and n/sum(actual_end-start) only; paced n/elapsed wall includes intentional sleep and is not processing margin; averages do not certify bursts/latency/continuous queue capacity",
                "cold_load_included":False if "process_only" in meta.get("frame_process_wall_ms","") else None,
                "cold_load_measurement_ms":None,"replay_wall_ms_including_csv_snapshot_io":replay_wall,
                "preload_setup_wall_ms":number(meta.get("preload_setup_wall_ms",saved_summary.get("preload_setup_wall_ms"))),
                "preload_note":"若生产者提供则为run入口到节拍epoch前的校验/输入加载/setup实际wall；不是OS冷盘保证或机器人启动延迟",
                "replay_with_io_fps":len(frames)*1000/replay_wall if replay_wall else None,
                "replay_wall_cold_load_included":False,"cold_load_note":"producer replay timer starts after readonly Bundle/cache/algorithm/file preload; cold load is excluded and not measured here",
                "sensor_span_s":sensor_span,"sensor_arrival_fps":arrival_fps},
            actual_wall=actual, first_commit=commit_records[0] if commit_records else None,
            first_origin_commit=origin_commits[0] if origin_commits else None,
            first_start_window_commit=window_commits[0] if window_commits else None,
            first_start_window_origin_commit=origin_window_commits[0] if origin_window_commits else None,
            commits=commit_records,start_window_commits=window_commits,
            origin_definition={"candidate_id_less_than":origin_count,"source":origin_count_source,"independent_spatial_truth":False},
            start_window={"first_id":617,"last_id":646,"label":"历史首次返起点诊断窗口，不是独立真值标签"},
            limitations=["无独立真值：false_count/true_count均null；提交数不是正确回环数",
                "实际PACE是预加载内存节拍，不是ROS传输/真实有界队列；end与含active的start保守上界分别比较cap32，间隙未连续采样；原summary PASS不构成有界queue结论",
                "无PACE文件不模拟latency/queue；SC缓存v2不作live速度对比；冷载入不在process/replay计时内"])
        if flags and any("mismatch" in value or "invalid" in value or "nonsequential" in value for value in flags):
            base["data_integrity"]="FAIL"
        else:
            base["data_integrity"]="PASS"
        return base
    except (OSError,ValueError,TypeError) as error:
        base.update(status="invalid",errors=[str(error)])
        base["non_pass_markers"].append("invalid_or_missing_schema")
        return base


def build_report(runs):
    results = {name:summarize_run(path) for name,path in runs.items()}
    waiting = any(r["status"] in ["waiting","running"] for r in results.values())
    comparisons = {}
    baseline = results.get("baseline_live_full",{})
    for name,result in results.items():
        origin, reference = result.get("first_origin_commit"),baseline.get("first_origin_commit")
        if origin and reference:
            comparisons[name] = {"first_origin_sensor_seconds_earlier_than_baseline":
                (reference["stamp_ns"]-origin["stamp_ns"])/1e9,
                "commit_timing_only_not_correctness_or_latency":True}
    return {"schema":"a2w_loop_v3_statistics_v1","created_utc":datetime.now(timezone.utc).isoformat(),
        "status":"waiting" if waiting else "invalid" if any(r["status"]=="invalid" for r in results.values()) else "complete",
        "all_runs_complete":all(r["status"]=="complete" for r in results.values()),
        "overall_loop_acceptance":"not_established_without_independent_truth_and_runtime_acceptance",
        "runs":results,"comparisons":comparisons,
        "limitations_zh":"回环真假没有独立真值标签。缺失scheduled保持waiting；完成计算不代表安全/几何/实时验收通过。"}


def chinese_text(report):
    lines = ["v3离线回放统计（只读）",f"汇总状态：{report['status']}；全部运行完成：{report['all_runs_complete']}",
        "没有独立真值，真假回环计数均为null；不得由提交或几何候选数推断误回环率。",
        "所有分位数为线性插值p50/p95/max；空值是missing，不填0或PASS。"]
    def show(value):return "missing" if value is None else f"{value:.6g}" if isinstance(value,(int,float)) else str(value)
    for name,result in report["runs"].items():
        lines += ["",f"[{name}] {result['path']}",f"运行状态：{result['status']}"]
        if result["status"] in ["waiting","invalid"]:
            lines.append(result.get("waiting_reason") or "；".join(result["errors"]))
            continue
        c=result["counts"]
        lines += [f"帧{c['frames']}；query{c['query_calls']}；原始候选行{c['raw_query_candidate_rows']}；候选event{c['candidate_event_rows']}；配准尝试{c['geometry_attempts']}；几何metric门{c['metric_gate_pass_rows']}；独立pose标志通过{c['pose_guard_pass_rows']}；几何且pose通过{c['joint_geometry_pose_pass_rows']}；临时accepted行{c['accepted_event_rows']}；实际提交{c['committed_loops']}（非真值）",
            f"首次提交：{result['first_commit']}",f"首次起点候选提交：{result['first_origin_commit']}",
            f"617..646窗口提交{c['start_window_commits']}、其中起点候选{c['start_window_origin_commits']}；首次：{result['first_start_window_commit']}",
            "SC解释："+result["sc_timing"]["interpretation"],
            f"处理能力：N/sum(process_wall)={show(result['throughput']['process_only_fps'])}帧/秒；N/sum(actual_end-start)含primaryIO={show(result['throughput']['service_with_primary_io_fps'])}帧/秒；输入均值{show(result['throughput']['sensor_arrival_fps'])}帧/秒。",
            f"均值处理/到达比：process-only={show(result['throughput']['process_to_sensor_mean_rate_ratio'])}、含primaryIO={show(result['throughput']['service_with_primary_io_to_sensor_mean_rate_ratio'])}；仅均值余量，不代表burst/延迟/容量验收。",
            f"N/replay总wall={show(result['throughput']['replay_with_io_fps'])}帧/秒；含主动节拍sleep={result['throughput']['elapsed_rate_includes_pacing_sleep']}，此值不能当处理余量。实际服务范围排除timing/console等帧间隙；冷载入排除。",
            f"入口校验/预加载/setup实际wall ms：{show(result['throughput']['preload_setup_wall_ms'])}；不保证OS冷盘。",
            "逐帧耗时ms（含无工作帧；共享SC等只计一次）："]
        for field,stats in result["frame_timings_ms"].items():
            lines.append(f"  {field}: n={stats['count']} p50={show(stats['p50'])} p95={show(stats['p95'])} max={show(stats['max'])} sum={show(stats['sum'])}")
        lines.append("几何激活帧process耗时："+str(result["geometry_active_frame_timings_ms"]["process_wall_ms"]))
        lines.append("实际调用分阶段耗时ms（只含正耗时调用；不把重复共享字段求和）：")
        for field,stats in result["event_work_timings_ms_positive_invocations"].items():
            lines.append(f"  {field}: n={stats['count']} p50={show(stats['p50'])} p95={show(stats['p95'])} max={show(stats['max'])} sum={show(stats['sum'])}")
        wall=result["actual_wall"]
        lines += [f"实际wall/节拍状态：{wall['status']}；不由旧process成本模拟队列/延迟。",
            f"实际end latency ms：{wall['end_latency_ms']}；commit latency ms：{wall['commit_latency_ms']}",
            f"end waiting最大{wall['maximum_pending']}、cap32采样检查={wall['end_waiting_capacity_check']}；start含将active帧保守上界最大{wall.get('maximum_unprocessed_at_start_including_next_active')}、cap32上界检查={wall['start_including_active_capacity_upper_bound_check']}。",
            f"连续队列界已建立={wall['continuous_queue_bound_established']}；容量结论={wall['capacity_acceptance']}。end采样可能漏timing/console间隙；不是ROS/真实传输queue，原summary PASS不等于有界queue。",
            "逐层拒绝："+str(result["rejection_stage_counts"]),"原因计数："+str(result["reason_counts"]),
            "不通过标记："+("；".join(result["non_pass_markers"]) or "无已检测数据/容量不通过；不等于整体验收PASS")]
    lines += ["", "首次起点提交sensor提前秒数（不是实际延迟/正确性）："+str(report["comparisons"]),report["limitations_zh"]]
    return "\n".join(lines)+"\n"


def write_report(runs, output):
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("output already exists; refusing overwrite")
    if any(output==Path(path).resolve() or output in Path(path).resolve().parents or Path(path).resolve() in output.parents for path in runs.values()):
        raise ValueError("output must be separate from all input directories")
    report=build_report(runs)
    report["tool"] = identity(Path(__file__))
    output.mkdir(parents=True)
    (output/"summary.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n")
    (output/"summary_zh.txt").write_text(chinese_text(report))
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",action="append",help="NAME=PATH; repeat to override default three runs")
    parser.add_argument("--output",type=Path,default=Path("artifacts/loop_recall_v3_20261020/statistics/v1"))
    args=parser.parse_args(argv)
    try:
        runs=dict(DEFAULT_RUNS)
        if args.run:
            runs={}
            for value in args.run:
                name,path=value.split("=",1)
                if not name or name in runs:raise ValueError("run names must be unique and nonempty")
                runs[name]=Path(path)
        report=write_report(runs,args.output)
        print(json.dumps({"output":str(args.output),"status":report["status"],"all_runs_complete":report["all_runs_complete"]}))
        return 0
    except (OSError,ValueError) as error:
        print(f"loop_v3_summary: {error}",file=sys.stderr)
        return 2


if __name__=="__main__":sys.exit(main())
