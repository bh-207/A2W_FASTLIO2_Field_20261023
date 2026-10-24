#!/usr/bin/env python3
"""Record/evaluate existing localization/navigation streams; never starts hardware."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import heapq
import json
import os
from pathlib import Path
import signal
import shutil
import sqlite3
import subprocess
import sys
import time

import yaml

from trial_metrics import SCENARIOS, TrialEvaluator, record_plan, json_safe

ROOT = Path(__file__).resolve().parents[2]


def write_json(path, value):
    Path(path).write_text(json.dumps(json_safe(value), indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def route_data(path):
    if path is None:
        return None, []
    data = yaml.safe_load(Path(path).read_text())
    if data.get('frame_id') != 'map' or data.get('robot_frame') != 'base_link':
        raise ValueError('Route must use map/base_link')
    points = [(float(w['position']['x']), float(w['position']['y'])) for w in data['waypoints']]
    if not points:
        raise ValueError('Route is empty')
    return (*points[-1], float(data['waypoints'][-1]['yaw'])), points


def evaluate(bag, output, route=None):
    # Codecs only: evaluation neither calls rclpy.init nor creates a ROS entity.
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    bag = Path(bag).resolve()
    files = [bag] if bag.suffix == '.db3' else sorted(bag.glob('*.db3'))
    if not files:
        raise ValueError('No SQLite rosbag segments; supply an uncompressed .db3 bag')
    goal, points = route_data(route)
    evaluator = TrialEvaluator(goal, points)
    connections, iterators, type_cache, inventory, errors = [], [], {}, {}, []
    try:
        for file in files:
            connection = sqlite3.connect(file.as_uri()+'?mode=ro', uri=True)
            connections.append(connection)
            topics = {row[0]: (row[1], row[2], row[3]) for row in connection.execute(
                'SELECT id,name,type,serialization_format FROM topics')}
            def rows(conn=connection, table=topics):
                for receipt, topic_id, data in conn.execute(
                        'SELECT timestamp,topic_id,data FROM messages ORDER BY timestamp,id'):
                    name, type_name, serialization = table[topic_id]
                    yield receipt, name, type_name, serialization, data
            iterators.append(rows())
        # Merge split files by receipt time; filename lexical order is insufficient.
        for receipt, name, type_name, serialization, data in heapq.merge(
                *iterators, key=lambda row: row[0]):
            inventory[name] = {'type': type_name, 'count': inventory.get(name, {}).get('count', 0)+1}
            # Huge raw sensors need only their headers, but decode still verifies CDR.
            try:
                if serialization != 'cdr':
                    raise ValueError('Unsupported serialization '+serialization)
                if type_name not in type_cache:
                    type_cache[type_name] = get_message(type_name)
                message = deserialize_message(bytes(data), type_cache[type_name])
                evaluator.observe(name, type_name, message, receipt)
            except (ValueError, TypeError, RuntimeError, AttributeError, ImportError) as error:
                evaluator.invalid_messages['decode_or_contract:'+name] += 1
                if len(errors) < 50:
                    errors.append({'topic': name, 'receipt_ns': receipt, 'error': str(error)})
    finally:
        for connection in connections:
            connection.close()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    result = evaluator.summary()
    result['bag_files'] = [{'path': str(p), 'bytes': p.stat().st_size,
                            'sha256': file_sha256(p)} for p in files]
    result['inventory'] = inventory
    result['decode_errors'] = errors
    if evaluator.invalid_messages:
        result['capture_complete'] = False
    write_json(output/'summary.json', result)
    write_json(output/'registration.json', evaluator.registrations)
    write_json(output/'events.json', evaluator.events)
    with (output/'match_stages.csv').open('w') as stream:
        fields = sorted({k for row in evaluator.stages for k in row})
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(evaluator.stages)
    lines = ['A2W trial evaluation', '', 'Completeness: '+str(result['capture_complete']),
             'Missing: '+', '.join(result['missing_core_topics']),
             'Normal registration: '+str(result['normal_registration']),
             'State transitions: '+str(result['localization']['transitions']),
             'Estimated terminal: '+str(result['estimated_terminal']),
             'Independent physical endpoint accuracy: NOT VERIFIED',
             'Full numeric results: summary.json, match_stages.csv, registration.json']
    (output/'summary.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'capture_complete': result['capture_complete'],
                      'normal_registration_count': result['normal_registration']['count'],
                      'accuracy_validation': result['accuracy_validation'], 'output': str(output)}))
    return result


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(1024*1024), b''):
            digest.update(data)
    return digest.hexdigest()


def archive_bundle(source, destination):
    """Freeze and verify all manifest-listed inputs, including real keyframe PCDs."""
    source, destination = Path(source).resolve(), Path(destination)
    entries = []
    seen = set()
    for line in (source/'manifest.sha256').read_text().splitlines():
        digest, separator, name = line.partition('  ')
        relative = Path(name)
        if (not separator or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest)
                or relative.is_absolute() or '..' in relative.parts or name in seen):
            raise ValueError('Invalid bundle manifest entry '+line)
        original = (source/relative).resolve()
        if not original.is_relative_to(source) or not original.is_file():
            raise ValueError('Missing or unconfined bundle input '+name)
        if file_sha256(original) != digest:
            raise ValueError('Bundle SHA256 mismatch '+name)
        seen.add(name); entries.append((relative, original, digest))
    if not entries:
        raise ValueError('Empty bundle manifest')
    destination.mkdir(parents=True, exist_ok=False)
    for relative, original, digest in entries:
        target = destination/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, target)
        if file_sha256(target) != digest:
            raise ValueError('Input changed while archiving '+str(relative))
    shutil.copyfile(source/'manifest.sha256', destination/'manifest.sha256')
    return {str(path): digest for path, _, digest in entries}


def snapshot(output, args, plan):
    dest = output/'inputs'
    dest.mkdir()
    hashes = {}
    for package in ('a2w_fastlio_localization', 'a2w_fastlio2_bringup', 'a2w_navigation'):
        for file in (ROOT/'src'/package/'config').glob('*.yaml'):
            target = dest/package/file.name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(file.read_bytes()); hashes[str(file.relative_to(ROOT))] = file_sha256(file)
    if args.route_file:
        route_data(args.route_file)
        (dest/'route.yaml').write_bytes(args.route_file.read_bytes())
        hashes['route_file'] = file_sha256(args.route_file)
    if args.map_bundle:
        for name, digest in archive_bundle(args.map_bundle, dest/'map_bundle').items():
            hashes['map_bundle/'+name] = digest
        hashes['map_bundle/manifest.sha256'] = file_sha256(dest/'map_bundle/manifest.sha256')
    software = {}
    for package in ('a2w_fastlio_common', 'a2w_fastlio_localization', 'a2w_waypoint_tools', 'a2w_navigation'):
        for file in sorted((ROOT/'src'/package).rglob('*')):
            if file.is_file() and file.suffix in ('.cpp', '.hpp', '.py', '.yaml') and '__pycache__' not in file.parts:
                software[str(file.relative_to(ROOT))] = file_sha256(file)
    binaries = {}
    for name in ('build/a2w_fastlio_localization/localization_node',
                 'install/a2w_fastlio_common/lib/liba2w_registration.so',
                 'install/a2w_fastlio_common/lib/liba2w_algorithm_suite.so'):
        file = ROOT/name
        if file.is_file():
            target = dest/'local_build_binaries'/file.name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(file, target)
            binaries[name] = file_sha256(target)
    return {'created_utc': datetime.now(timezone.utc).isoformat(), 'scenario': args.scenario,
            'operator_note': args.note, 'hardware_started_by_tool': False, 'motion_sent_by_tool': False,
            'duration_s': args.duration, 'command': plan['argv'], 'inputs_sha256': hashes,
            'software_source_sha256': software, 'local_build_binary_sha256': binaries,
            'binary_identity_scope': 'local workspace build snapshot; actual launched binary and parameter overrides must be correlated',
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION'),
            'data_scope': 'operator-specified trial; recording itself does not verify physical accuracy',
            'working_tree_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()}


def record(args):
    output = args.output.resolve()
    plan = record_plan(output, args.scenario, args.duration, ROOT)
    if args.dry_run:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return 0
    # Fresh directory prevents silently mixing two real trials.
    output.mkdir(parents=True, exist_ok=False)
    (output/'qos.yaml').write_text(yaml.safe_dump(plan['qos']))
    metadata = snapshot(output, args, plan)
    write_json(output/'trial.json', metadata)
    process = None
    interrupted = False
    def stop(signum, frame):
        nonlocal interrupted
        interrupted = True
    old_int, old_term = signal.signal(signal.SIGINT, stop), signal.signal(signal.SIGTERM, stop)
    try:
        with (output/'recorder.log').open('w') as log:
            process = subprocess.Popen(plan['argv'], stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            start = time.monotonic()
            while not interrupted and time.monotonic()-start < args.duration and process.poll() is None:
                time.sleep(.1)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                return_code = process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    return_code = process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); return_code = process.wait(timeout=5)
            metadata.update(recorder_exit=return_code, interrupted=interrupted,
                            elapsed_wall_s=time.monotonic()-start)
            write_json(output/'trial.json', metadata)
        route = output/'inputs/route.yaml' if args.route_file else None
        result = evaluate(output/'bag', output/'evaluation', route)
        return 0 if return_code == 0 and result['capture_complete'] else 2
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=20)
        signal.signal(signal.SIGINT, old_int); signal.signal(signal.SIGTERM, old_term)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='action', required=True)
    rec = subs.add_parser('record', help='Record existing topics and evaluate; never launches drivers')
    rec.add_argument('--scenario', choices=SCENARIOS, required=True)
    rec.add_argument('--duration', type=float, default=60.)
    rec.add_argument('--output', type=Path, required=True)
    rec.add_argument('--map-bundle', type=Path)
    rec.add_argument('--route-file', type=Path)
    rec.add_argument('--note', default='')
    rec.add_argument('--dry-run', action='store_true')
    ev = subs.add_parser('evaluate', help='Read-only SQLite/CDR evaluation, no ROS participant')
    ev.add_argument('bag', type=Path)
    ev.add_argument('--output', type=Path, required=True)
    ev.add_argument('--route-file', type=Path)
    args = parser.parse_args()
    try:
        if args.action == 'record':
            return record(args)
        evaluate(args.bag, args.output, args.route_file)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print('Trial failed: '+str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
