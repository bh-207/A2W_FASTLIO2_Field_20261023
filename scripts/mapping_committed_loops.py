#!/usr/bin/env python3
"""Observe explicit backend graph commits, not candidates or save completion.

The parser and tests need only Python. ROS imports happen only in main(). Sensor
and system clock synchronization is deliberately NOT inferred from their values.
"""
import argparse
import csv
import math
from pathlib import Path


PREFIX = "[LoopCommitted] "
FIELDS = ["audit_v", "source_id", "target_id", "sensor_stamp_ns", "commit_wall_ns",
          "process_to_commit_ms", "process_clock_status", "sensor_wall_latency_ms",
          "clock_relation", "fitness", "overlap", "correspondences", "factors",
          "optimized_translation_m", "optimized_rotation_rad", "gtsam_ms"]
INTEGER_FIELDS = ("audit_v", "source_id", "target_id", "sensor_stamp_ns",
                  "commit_wall_ns", "correspondences", "factors")
METRIC_FIELDS = ("process_to_commit_ms", "fitness", "overlap", "optimized_translation_m",
                 "optimized_rotation_rad", "gtsam_ms")


def parse_committed_loop(node_name, message, expected_node="mapping_backend_node"):
    if node_name != expected_node or not message.startswith(PREFIX):
        return None
    try:
        tokens = [token.split("=", 1) for token in message[len(PREFIX):].split()]
        if any(len(token) != 2 for token in tokens):
            return None
        record = dict(tokens)
        if len(record) != len(tokens) or set(record) != set(FIELDS):
            return None
        for field in INTEGER_FIELDS:
            # int64 timestamps never pass through binary64 floats.
            raw = record[field]
            if not raw.isascii() or not raw.isdigit():
                return None
            record[field] = int(raw)
        if record["audit_v"] != 1 or record["sensor_stamp_ns"] > 2**63-1 or record["commit_wall_ns"] > 2**63-1:
            return None
        for field in METRIC_FIELDS:
            raw = record[field]
            record[field] = None if raw == "unknown" else float(raw)
            if record[field] is not None and (not math.isfinite(record[field]) or record[field] < 0):
                return None
        if record["overlap"] is not None and record["overlap"] > 1:
            return None
        if ((record["process_to_commit_ms"] is None and record["process_clock_status"] != "anomaly") or
                (record["process_to_commit_ms"] is not None and record["process_clock_status"] != "ok")):
            return None
        if record["sensor_wall_latency_ms"] != "unknown" or record["clock_relation"] != "unverified":
            return None
        record["sensor_wall_latency_ms"] = None
        return record
    except (TypeError, ValueError, OverflowError):
        return None


def format_chinese(record):
    processing = (f"{record['process_to_commit_ms']:.3f} ms" if record["process_to_commit_ms"] is not None
                  else "unknown（steady时钟异常）")
    return (f"[真实回环已提交] {record['source_id']} -> {record['target_id']} "
            f"sensor_stamp_ns={record['sensor_stamp_ns']} commit_wall_ns={record['commit_wall_ns']} "
            f"本帧处理至提交={processing} fitness={record['fitness']} overlap={record['overlap']} "
            "传感器到系统墙钟延迟=unknown（未验证同源）；这不是地图保存完成。")


def csv_writer(stream):
    writer = csv.DictWriter(stream, fieldnames=FIELDS)
    writer.writeheader()
    return writer


def rosout_qos():
    # Humble's rclpy.qos has no qos_profile_rosout_default export. Match the
    # public rcl rosout defaults explicitly, without creating a ROS context.
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", default="mapping_backend_node", help="exact rosout logger name, never suffix matching")
    parser.add_argument("--rosout-topic", default="/rosout")
    parser.add_argument("--csv", type=Path, help="new CSV path; existing files are never overwritten")
    args = parser.parse_args(argv)
    if not args.node or not args.rosout_topic:
        parser.error("node and rosout topic must not be empty")
    # No ROS dependency is imported by pure parser consumers/tests.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rcl_interfaces.msg import Log
    stream = args.csv.open("x", newline="") if args.csv else None
    writer = csv_writer(stream) if stream else None
    node = None
    try:
        rclpy.init(args=[])
        node = rclpy.create_node("mapping_committed_loops_monitor")
        count = 0
        def callback(message):
            nonlocal count
            record = parse_committed_loop(message.name, message.msg, args.node)
            if record is None:
                return
            count += 1
            if writer:
                writer.writerow(record)
                stream.flush()
            print(f"#{count} {format_chinese(record)}", flush=True)
        subscription = node.create_subscription(Log, args.rosout_topic, callback, rosout_qos())
        print(f"仅监视精确节点={args.node} 前缀={PREFIX.strip()}；候选成功/保存日志不计数。", flush=True)
        rclpy.spin(node)
        del subscription
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if stream:
            stream.close()


if __name__ == "__main__":
    main()
