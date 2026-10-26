#!/usr/bin/python3
"""Bounded Nav2 uprobe diagnostics; never writes ROS parameters/actions/commands.

Requires the operator's sudo authentication for perf only. Does not alter sysctl,
libraries, algorithms, or launch files. Temporary uniquely named probes are removed
in finally. Use --check without sudo to check the selected process and symbols.
"""
import argparse
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time


FUNCTIONS = {
    "cycle": ("libcontroller_server_core.so", "nav2_controller::ControllerServer::computeAndPublishVelocity("),
    "mppi": ("libmppi_controller.so", "nav2_mppi_controller::MPPIController::computeVelocityCommands("),
    "path": ("libmppi_controller.so", "mppi::PathHandler::transformPath("),
    "eval": ("libmppi_controller.so", "mppi::Optimizer::evalControl("),
    "viz": ("libmppi_controller.so", "nav2_mppi_controller::MPPIController::visualize("),
    "map_update": ("libnav2_costmap_2d_core.so", "nav2_costmap_2d::Costmap2DROS::updateMap("),
}


def stats(values):
    ordered = sorted(values)
    return {"count": len(ordered), "min": ordered[0],
            "p50": ordered[math.ceil(len(ordered) * .5) - 1],
            "p95": ordered[math.ceil(len(ordered) * .95) - 1], "max": ordered[-1]}


def analyze(text):
    # perf script -F tid,time,event; stack keyed by thread AND function.
    pattern = re.compile(r"^\s*(\d+)\s+(\d+\.\d+):\s+a2w_navprof_\w+:(\w+)_(in|out):")
    stacks, spans, values = defaultdict(list), defaultdict(list), defaultdict(list)
    missing = 0
    for line in text.splitlines():
        match = pattern.match(line)
        if not match:
            continue
        tid, when, stage, edge = match.groups()
        when = float(when)
        key = (int(tid), stage)
        if edge == "in":
            stacks[key].append(when)
        elif stacks[key]:
            begin = stacks[key].pop()
            if when >= begin:
                spans[key].append((begin, when))
                values[stage].append((when - begin) * 1000.)
            else:
                missing += 1
        else:
            missing += 1
    for (tid, stage), calls in list(spans.items()):
        if stage != "mppi":
            continue
        for begin, end in calls:
            paths = [p for p in spans[(tid, "path")] if begin <= p[0] <= p[1] <= end]
            evals = [e for e in spans[(tid, "eval")] if begin <= e[0] <= e[1] <= end]
            if len(paths) == len(evals) == 1 and paths[0][1] <= evals[0][0]:
                # Includes lock acquisition + tiny bookkeeping/scheduling, NOT
                # an exact mutex-wait timer and NOT pure on-CPU computation.
                values["pre_eval_gap"].append((evals[0][0] - paths[0][1]) * 1000.)
    return {
        "active_control_sampled": bool(values.get("mppi")),
        "stages_ms": {key: stats(value) for key, value in values.items() if value},
        "unpaired_entries": sum(len(value) for value in stacks.values()),
        "unpaired_returns": missing,
        "notes": ["函数耗时是墙钟时间，包含线程调度/等待，不等于纯 CPU 时间。",
                  "pre_eval_gap 包含 costmap 锁获取及少量代码/调度，不单独证明锁竞争。",
                  "map_update 与控制器可能并行，不能将各阶段 p95 相加。",
                  "缺少入口/出口的调用不计入耗时；异常退出/采样边界可能造成缺项。"],
    }


def run(args, **kwargs):
    return subprocess.run(args, check=True, text=True, capture_output=True,
                          timeout=15, **kwargs).stdout


def find_controller():
    found = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdecimal():
            continue
        try:
            if (proc / "exe").resolve(strict=True).name == "controller_server":
                found.append(int(proc.name))
        except (OSError, RuntimeError):
            continue
    if len(found) != 1:
        raise RuntimeError(f"必须且只能找到一个 controller_server，当前：{found}")
    return found[0]


def probe_specs(pid):
    exe = Path(f"/proc/{pid}/exe").resolve(strict=True)
    if exe.name != "controller_server":
        raise RuntimeError("目标不是 controller_server")
    mapped = {Path(line.split()[-1]) for line in Path(f"/proc/{pid}/maps").read_text().splitlines()
              if line.split()[-1].startswith("/")}
    result, cache = {}, {}
    for stage, (library, prefix) in FUNCTIONS.items():
        paths = [p for p in mapped if p.name == library]
        if len(paths) != 1:
            raise RuntimeError(f"未唯一加载 {library}，请先让 Nav2 完成一次激活：{paths}")
        path = paths[0]
        if path not in cache:
            symbols = [line.split()[-1] for line in run(
                ["nm", "-D", "--defined-only", str(path)]).splitlines() if line.split()]
            readable = run(["c++filt"], input="\n".join(symbols) + "\n").splitlines()
            cache[path] = list(zip(symbols, readable))
        matches = [s for s, name in cache[path] if name.startswith(prefix)]
        if len(matches) != 1:
            raise RuntimeError(f"函数符号不唯一：{prefix}: {matches}")
        result[stage] = (str(path), matches[0])
    return result


def wait_bridge(node, snapshot, seconds=5):
    import rclpy
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        when, disabled = snapshot[0]
        if time.monotonic() - when < 1.:
            if not disabled:
                raise RuntimeError("硬件输出未关闭；拒绝开始采样。先执行 a2w_stop_now.sh。")
            return
    raise RuntimeError("没有收到新鲜的 MotionBridge 状态，拒绝采样")


def collect(pid, specs, duration, output):
    import rclpy
    from std_msgs.msg import String
    # Authenticate externally in the user's terminal; never change perf policy.
    run(["sudo", "-n", "true"])
    rclpy.init()
    node = rclpy.create_node("a2w_readonly_perf_safety_check")
    snapshot = [(0., False)]

    def on_status(message):
        try:
            disabled = json.loads(message.data).get("hardware_output_enabled") is False
        except (ValueError, TypeError, AttributeError):
            disabled = False
        snapshot[0] = (time.monotonic(), disabled)

    subscription = node.create_subscription(String, "/a2w_motion_bridge/status", on_status, 10)
    group = f"a2w_navprof_{os.getpid()}_{time.time_ns()}"
    installed = []
    proc = None
    safe = True
    try:
        wait_bridge(node, snapshot)
        for stage, (library, symbol) in specs.items():
            for edge, suffix in (("in", ""), ("out", "%return")):
                event = f"{group}:{stage}_{edge}"
                run(["sudo", "-n", "perf", "probe", "-x", library,
                     "--add", f"{event}={symbol}{suffix}"])
                installed.append(event)
        wait_bridge(node, snapshot)
        output.mkdir(parents=True, exist_ok=False)
        (output / "metadata.json").write_text(json.dumps({
            "pid": pid, "seconds": duration, "specs": specs,
            "note": "诊断不发送 ROS 运动指令、Goal 或参数修改"}, ensure_ascii=False, indent=2))
        data = output / "perf.data"
        # Bound the privileged child independently even if this Python process
        # fails. perf inherits the terminal process group so Ctrl+C reaches it.
        args = ["sudo", "-n", "timeout", "--signal=INT", str(duration + 5),
                "perf", "record", "--clockid", "mono", "-m", "1024", "-o", str(data),
                "-p", str(pid)]
        for event in installed:
            args.extend(["-e", event])
        args.extend(["--", "sleep", str(duration)])
        print(f"采样已就绪：{duration} 秒内请在 RViz 发一个短目标；不要开启硬件输出。", flush=True)
        with (output / "perf_record.log").open("w") as log:
            proc = subprocess.Popen(args, stdout=log, stderr=log)
            while proc.poll() is None:
                rclpy.spin_once(node, timeout_sec=.1)
                when, disabled = snapshot[0]
                if not disabled or time.monotonic() - when > 2.:
                    safe = False
            if proc.returncode:
                raise RuntimeError(f"perf 采样失败，查看 {output / 'perf_record.log'}")
        raw = run(["sudo", "-n", "perf", "script", "-i", str(data), "-F", "tid,time,event"])
        (output / "events.txt").write_text(raw)
        result = analyze(raw)
        result["hardware_disabled_throughout"] = safe
        result["record_log_requires_loss_check"] = "检查 perf_record.log 是否报告 LOST/丢事件；有丢失则结果仅供参考"
        (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print(f"采样目录：{output}")
        if not result["active_control_sampled"]:
            print("没有有效 MPPI 调用：可能未授权 Nav2/未发 Goal；不能判断性能。", file=sys.stderr)
        if not safe:
            print("硬件状态曾不满足关闭/新鲜条件，此记录不作为静止测试验收。", file=sys.stderr)
    finally:
        if proc is not None and proc.poll() is None:
            # The timeout above guarantees this wait is bounded. No signals to
            # controller_server or any robot process are sent by this tool.
            proc.wait(timeout=duration + 10)
        for event in reversed(installed):
            try:
                run(["sudo", "-n", "perf", "probe", "--del", event])
            except (OSError, subprocess.SubprocessError) as error:
                print(f"警告：临时探针 {event} 未清理：{error}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="仅检查目标与符号，不加探针")
    parser.add_argument("--pid", type=int)
    parser.add_argument("--seconds", type=int, default=20, choices=range(5, 31))
    parser.add_argument("--analyze", type=Path, help="离线分析 events.txt，不需要 ROS 或 sudo")
    args = parser.parse_args()
    if args.analyze:
        print(json.dumps(analyze(args.analyze.read_text()), ensure_ascii=False, indent=2))
        return
    pid = args.pid if args.pid is not None else find_controller()
    specs = probe_specs(pid)
    if args.check:
        print(json.dumps({"pid": pid, "symbols_found": specs}, ensure_ascii=False, indent=2))
        return
    root = Path(__file__).resolve().parents[2]
    output = root / "log/nav2_profiling" / f"profile_{time.time_ns()}"
    collect(pid, specs, args.seconds, output)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f"诊断未完成：{error}\n若 sudo 未授权，请在此终端先执行 sudo -v。", file=sys.stderr)
        sys.exit(1)
