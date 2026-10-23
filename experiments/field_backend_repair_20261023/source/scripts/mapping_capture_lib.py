"""Pure safety contracts for controlled mapping capture (no ROS imports)."""
from __future__ import annotations

import csv
import copy
from contextlib import closing
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import struct
import time

import yaml

from mapping_initial_conditions import STATUS, experimental_policy_errors, condition_labels

TOPICS = ['/lidar_points', '/lidar_imu', '/Odometry', '/tf', '/tf_static',
          '/mapping/registration_status']
RAW_TOPICS = TOPICS[:2]
POINT_FIELDS = {'x': 7, 'y': 7, 'z': 7, 'intensity': 7, 'ring': 4, 'timestamp': 8}
DEFAULT_RECORDER_NODE='rosbag2_recorder'
MAX_CLOCK_SKEW_NS = 2_000_000_000
MAX_REPORT_AGE_NS = 5_000_000_000


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''): digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, data, exclusive=False):
    path = Path(path)
    if exclusive:
        with path.open('x') as stream: json.dump(data, stream, indent=2, allow_nan=False); stream.write('\n')
    else:
        # A reader never sees a partially written live-check report.
        temporary = path.with_name(path.name + f'.tmp.{os.getpid()}')
        with temporary.open('x') as stream: json.dump(data, stream, indent=2, allow_nan=False); stream.write('\n')
        temporary.replace(path)


def interface_template():
    evidence = {'source': 'unknown', 'version': 'unknown', 'config': 'unknown', 'sha256': 'unknown'}
    return dict(schema='mapping_input_interface_v1', source_provenance='real',
                gyro_unit='unknown', accel_unit='unknown', imu_frame='unknown', lidar_frame='unknown',
                imu_minus_lidar_offset_ns=None,
                offset_definition='t_imu_raw_minus_t_lidar_same_event',
                gyro_unit_evidence=dict(evidence), accel_unit_evidence=dict(evidence),
                time_offset_evidence=dict(evidence), frame_evidence=dict(evidence),
                extrinsic_evidence=dict(evidence, base_config_sha256='unknown'))


def parameters(path):
    document = yaml.safe_load(Path(path).read_text())
    if not isinstance(document, dict) or '/**' not in document:
        raise ValueError('expected wildcard ROS parameter YAML: ' + str(path))
    result = document['/**'].get('ros__parameters')
    if not isinstance(result, dict): raise ValueError('missing ros__parameters: ' + str(path))
    return result


def interface_errors(data, base, expected_provenance='real', experimental_policy=None):
    errors = base_profile_errors(base)
    unverified = data.get('calibration_status') == STATUS or any(
        data.get(key, {}).get('physical_calibration_verified') is False
        for key in ['time_offset_evidence', 'extrinsic_evidence'] if isinstance(data.get(key), dict))
    if unverified or experimental_policy is not None:
        errors.extend(experimental_policy_errors(experimental_policy, data, parameters(base), sha256(base)))
    if data.get('schema') != 'mapping_input_interface_v1': errors.append('schema unsupported')
    if data.get('source_provenance') != expected_provenance: errors.append('source_provenance must be '+expected_provenance)
    for key, choices in [('gyro_unit', ['rad/s', 'deg/s']), ('accel_unit', ['m/s^2', 'g'])]:
        if data.get(key) not in choices: errors.append(key + ' explicit unit required (no norm inference)')
    for key in ['imu_frame', 'lidar_frame']:
        if not isinstance(data.get(key), str) or data[key].strip() in ['', 'unknown']:
            errors.append(key + ' explicit actual frame required')
    for key in ['gyro_unit_evidence', 'accel_unit_evidence', 'time_offset_evidence', 'frame_evidence', 'extrinsic_evidence']:
        evidence = data.get(key, {})
        if not isinstance(evidence, dict): errors.append(key + ' object required'); continue
        for field in ['source', 'version', 'config']:
            if not isinstance(evidence.get(field), str) or evidence[field].strip() in ['', 'unknown']:
                errors.append(key + '.' + field + ' missing')
        if not re.fullmatch('[0-9a-fA-F]{64}', str(evidence.get('sha256', ''))): errors.append(key + '.sha256 missing')
    if data.get('offset_definition') != 't_imu_raw_minus_t_lidar_same_event': errors.append('offset_definition unknown/opposite sign')
    offset = data.get('imu_minus_lidar_offset_ns')
    if type(offset) is not int: errors.append('imu_minus_lidar_offset_ns integer required')
    else:
        original = parameters(base).get('common', {}).get('time_offset_lidar_to_imu')
        if original is None or not math.isfinite(float(original)) or abs(float(original)*1e9-offset) > 0.1:
            errors.append('offset does not match original base config; calibration changes are outside this CLI')
    extrinsic=data.get('extrinsic_evidence')
    if not isinstance(extrinsic,dict) or extrinsic.get('base_config_sha256') != sha256(base):
        errors.append('extrinsic_evidence.base_config_sha256 does not bind actual base config')
    return errors


def base_profile_errors(base):
    common=parameters(base).get('common',{})
    errors=[]
    for key,expected in [('lid_topic','/lidar_points'),('imu_topic','/lidar_imu')]:
        if common.get(key)!=expected: errors.append('base common.'+key+' must be '+expected)
    if common.get('time_sync_en') is not False: errors.append('base common.time_sync_en must explicitly be false; never silently override time synchronization')
    return errors


def evidence_text(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'))


def frontend_parameters(base, contract, session):
    errors=base_profile_errors(base)
    if errors: raise ValueError('; '.join(errors))
    params = parameters(base)
    common = params.setdefault('common', {})
    common['imu_gyr_unit'] = {'rad/s': 'rad', 'deg/s': 'deg'}.get(contract.get('gyro_unit'), 'unknown')
    common['time_sync_en'] = False
    # Never change the original offset or mapping.R/T, even for an invalid/unknown init.
    v4 = params.setdefault('frontend_init_v4', {})
    v4.update(input_interface_enabled=True, quality_gate_enabled=True,
              input_gyro_unit=contract.get('gyro_unit', 'unknown'), input_accel_unit=contract.get('accel_unit', 'unknown'),
              input_imu_frame=contract.get('imu_frame', 'unknown'), source_provenance=contract.get('source_provenance','unknown'),
              gyro_unit_evidence=evidence_text(contract.get('gyro_unit_evidence', {})),
              accel_unit_evidence=evidence_text(contract.get('accel_unit_evidence', {})),
              time_offset_evidence=evidence_text(contract.get('time_offset_evidence', {})),
              quality_trace_path=str(Path(session)/'init_quality.csv'),
              input_trace_path=str(Path(session)/'input.csv'),
              runtime_state_log_enabled=True, runtime_state_log_path=str(Path(session)/'runtime_state.csv'),
              runtime_state_log_capacity=1024)
    params['runtime_pos_log_enable'] = False
    params.setdefault('pcd_save',{})['pcd_save_en']=False
    params.setdefault('trajectory_save',{})['tum_en']=False
    params['use_sim_time'] = False
    return params


def flatten(data, prefix=''):
    result = {}
    for key, value in data.items():
        name = prefix + key
        if isinstance(value, dict): result.update(flatten(value, name + '.'))
        else: result[name] = value
    return result


def assert_conservative(params):
    for name, value in flatten(params).items():
        if ('revisit' in name or 'scheduled' in name) and value not in [False, 0, None]:
            raise ValueError('enhanced backend profile forbidden: ' + name)
        if name == 'quatro.feature_cache_capacity' and value != 0:
            raise ValueError('experimental feature cache forbidden in controlled capture')


def qos_overrides():
    result = {topic: dict(history='keep_last', depth=2000, reliability='best_effort', durability='volatile') for topic in TOPICS}
    result['/tf_static'] = dict(history='keep_last', depth=1, reliability='reliable', durability='transient_local')
    for topic,depth in [('/lidar_points',32),('/tf',100),('/Odometry',50),('/mapping/registration_status',50)]:
        result[topic]['depth']=depth
    return result


def record_command(session, compression='none',cache_size_mib=16):
    session = Path(session)
    if type(cache_size_mib) is not int or not 1<=cache_size_mib<=256: raise ValueError('record cache-size-mib must be 1..256 (double buffer can use twice this)')
    command = ['ros2', 'bag', 'record', '--storage', 'sqlite3', '--output', str(session/'bag'),
               '--storage-preset-profile','resilient',
               '--max-bag-size', '536870912', '--max-bag-duration', '300',
               '--max-cache-size',str(cache_size_mib*1024*1024),
               '--qos-profile-overrides-path', str(session/'snapshots/record_qos.yaml')]
    if compression == 'zstd':
        command += ['--compression-mode', 'file', '--compression-format', 'zstd',
                    '--compression-threads', '1', '--compression-queue-size', '4']
    elif compression != 'none': raise ValueError('unsupported compression')
    return command + TOPICS


def process_identity(pid):
    root = Path('/proc')/str(int(pid))
    stat = (root/'stat').read_text()
    fields = stat[stat.rfind(')')+2:].split()
    return dict(pid=int(pid), start_ticks=int(fields[19]),
                cmdline=[x.decode(errors='surrogateescape') for x in (root/'cmdline').read_bytes().split(b'\0') if x],
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def process_matches(identity):
    try:
        current = process_identity(identity['pid'])
        return bool(current['cmdline']) and all(current[key] == identity[key] for key in ['start_ticks', 'cmdline', 'boot_id'])
    except (OSError, ValueError, KeyError): return False


def runtime_ready(path):
    result = dict(ready=False, map_gravity_alignment=False, reason='INIT_COMMIT not observed')
    if not Path(path).is_file(): return result
    try:
        with Path(path).open() as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            if row.get('stage') != 'INIT_COMMIT': continue
            gravity = [float(row['g_O_' + axis]) for axis in 'xyz']
            norm = math.sqrt(sum(x*x for x in gravity))
            stamps = [int(row.get(key,0)) for key in ['state_stamp_ns','accepted_window_last_imu_ns','group_last_imu_ns']]
            if not math.isfinite(norm) or not 9.0 <= norm <= 10.5 or row.get('init_stamp_consistent') != '1' or not stamps[0] > 0 or len(set(stamps)) != 1:
                result['reason'] = 'INIT_COMMIT has invalid gravity/stamp consistency'; continue
            result.update(ready=True, reason='real runtime INIT_COMMIT observed', gravity_camera_init=gravity,
                          gravity_norm_m_s2=norm, anti_g_to_camera_init_z_rad=math.acos(max(-1.0,min(1.0,-gravity[2]/norm))),
                          state_stamp_ns=int(row.get('state_stamp_ns', 0)))
    except (ValueError, KeyError, TypeError) as error: result['reason'] = 'incomplete runtime CSV: ' + str(error)
    return result


def runtime_shutdown(path,log_path=None):
    result = dict(valid=False,errors=[])
    try:
        status = read_json(str(path)+'.status.json')
        with Path(path).open() as stream: count=sum(1 for _ in csv.DictReader(stream))
        result.update(status=status,csv_rows=count)
        if status.get('closed') is not True or status.get('flush_ok') is not True: result['errors'].append('runtime writer did not close/flush successfully')
        if status.get('dropped') != 0 or status.get('write_errors') != 0: result['errors'].append('runtime drops/write errors observed')
        if not status.get('enqueued') == status.get('written') == status.get('attempted') == count:
            result['errors'].append('runtime attempted/enqueued/written/CSV count mismatch')
        if log_path is not None:
            logs=Path(log_path).read_text(errors='replace')
            if re.search(r'\[frontend_runtime_v4\] (?:WRITE_ERROR|STATUS_WRITE_ERROR|QUEUE_DROP)',logs):
                result['errors'].append('runtime writer error/drop alarm present in frontend log')
            closed=re.findall(r'\[frontend_runtime_v4\] CLOSED attempted=(\d+) enqueued=(\d+) written=(\d+) dropped=(\d+) write_errors=(\d+) flush_ok=(\d+)',logs)
            if not closed: result['errors'].append('runtime CLOSED log summary missing')
            else:
                summary=dict(zip(['attempted','enqueued','written','dropped','write_errors','flush_ok'],map(int,closed[-1])))
                if any(summary[key]!=status.get(key) for key in ['attempted','enqueued','written','dropped','write_errors']) or summary['flush_ok']!=1:
                    result['errors'].append('runtime CLOSED log/status summary mismatch')
                result['closed_log_summary']=summary
        result['valid'] = not result['errors']
    except (OSError,ValueError,TypeError) as error: result['errors'].append(str(error))
    return result


class InputAccumulator:
    """Observation only: keeps header gaps/counts, never modifies samples or infers units."""
    def __init__(self,session):
        self.session=str(Path(session).resolve()); self.topics={}
        self.subscriptions={topic:False for topic in RAW_TOPICS}
        self.publisher_qos={}; self.tf_static_publishers_seen=False

    def observe(self,topic,stamp_ns,frame_id,receipt_ns,fields=None,errors=None,extra=None):
        row=self.topics.setdefault(topic,dict(count=0,first_stamp_ns=stamp_ns,stamp_ns=stamp_ns,
            frame_id=frame_id,max_header_gap_ns=0,nonmonotonic_count=0,errors=[],clock_skew_ns=receipt_ns-stamp_ns))
        if row['count']:
            gap=stamp_ns-row['stamp_ns']; row['max_header_gap_ns']=max(row['max_header_gap_ns'],gap)
            if gap<=0: row['nonmonotonic_count']+=1; errors=list(errors or [])+['non-increasing header stamp']
            if row['frame_id']!=frame_id: errors=list(errors or [])+['frame_id changed within session']
        row.update(count=row['count']+1,stamp_ns=stamp_ns,frame_id=frame_id,clock_skew_ns=receipt_ns-stamp_ns)
        row['last_receipt_unix_ns']=receipt_ns
        if abs(receipt_ns-stamp_ns)>MAX_CLOCK_SKEW_NS:
            errors=list(errors or [])+['clock skew observed beyond 2s; timestamps were not changed']
        if fields is not None:
            if 'fields' in row and row['fields']!=fields: errors=list(errors or [])+['PointFields changed within session']
            row['fields']=fields
        if extra: row.update(extra)
        for error in errors or []:
            if error not in row['errors']: row['errors'].append(error)

    def report(self):
        return dict(schema='mapping_raw_input_report_v1',session=self.session,updated_unix_ns=time.time_ns(),
            topics=self.topics,recorder_subscriptions=self.subscriptions,publisher_qos=self.publisher_qos,
            tf_static_publishers_seen=self.tf_static_publishers_seen,lossless_proven=False,
            note='Header/count/gap observations are proxies only. No unit inference, timestamp conversion, or data mutation.')


def input_report_errors(report, contract, session, now_ns=None):
    errors = []
    now_ns = time.time_ns() if now_ns is None else now_ns
    if report.get('session') != str(Path(session).resolve()): errors.append('input report belongs to different session')
    age = now_ns - report.get('updated_unix_ns', 0)
    if not 0 <= age <= MAX_REPORT_AGE_NS: errors.append('input report stale or future')
    for topic, frame_key in zip(RAW_TOPICS, ['lidar_frame', 'imu_frame']):
        observed = report.get('topics', {}).get(topic, {})
        if not report.get('recorder_subscriptions', {}).get(topic): errors.append(topic + ' recorder subscription not discovered')
        if observed.get('count', 0) < 2 or observed.get('stamp_ns', 0) <= 0: errors.append(topic + ' actual messages/header stamps not received')
        if not 0<=now_ns-observed.get('last_receipt_unix_ns',0)<=MAX_CLOCK_SKEW_NS:
            errors.append(topic+' reception stale/future (not just report refresh)')
        if observed.get('frame_id') != contract.get(frame_key): errors.append(topic + ' frame differs from explicit contract')
        if abs(observed.get('clock_skew_ns', MAX_CLOCK_SKEW_NS+1)) > MAX_CLOCK_SKEW_NS: errors.append(topic + ' clock skew exceeds 2s; stamps unchanged')
        if observed.get('errors'): errors.extend(topic + ': ' + x for x in observed['errors'])
        if topic == '/lidar_points':
            fields = {item['name']: item for item in observed.get('fields', [])}
            for name, datatype in POINT_FIELDS.items():
                if name not in fields or fields[name].get('datatype') != datatype or fields[name].get('count') != 1:
                    errors.append('PointField ' + name + ' missing/wrong type/count for actual Hesai parser')
    return errors


def verify_bundle(root, uuid=None, count=None):
    root = Path(root).resolve(strict=True)
    seen = set()
    lines = (root/'manifest.sha256').read_text().splitlines()
    if not lines: raise ValueError('empty bundle manifest')
    for line in lines:
        match = re.fullmatch(r'([0-9a-fA-F]{64})  (.+)', line)
        if not match: raise ValueError('invalid manifest line')
        digest, relative = match.groups()
        path = root/relative
        if Path(relative).is_absolute() or '..' in Path(relative).parts or relative in seen or not path.resolve().is_relative_to(root):
            raise ValueError('unconfined/duplicate bundle manifest path')
        if not path.is_file() or sha256(path) != digest.lower(): raise ValueError('bundle hash mismatch: ' + relative)
        seen.add(relative)
    required = {'metadata.yaml','optimized_poses.csv','keyframes/index.csv','descriptors/scan_context.bin','global_map.pcd',
                'config/mapping_effective.yaml','config/scan_context.yaml','config/registration.yaml'}
    if not required <= seen: raise ValueError('bundle core files absent from manifest: ' + str(required-seen))
    metadata = yaml.safe_load((root/'metadata.yaml').read_text())
    if metadata.get('schema')!='a2w_map_bundle_v1': raise ValueError('unsupported MapBundle schema')
    if uuid is not None and metadata.get('bundle_uuid') != uuid: raise ValueError('bundle response UUID mismatch')
    actual_count = int(metadata.get('keyframe_count',0))
    if actual_count <= 0 or (count is not None and actual_count != count): raise ValueError('bundle response keyframe count mismatch')
    if metadata.get('descriptor_count')!=actual_count: raise ValueError('bundle descriptor/keyframe count mismatch')
    with (root/'keyframes/index.csv').open() as stream: frames=list(csv.DictReader(stream))
    with (root/'optimized_poses.csv').open() as stream: poses=list(csv.DictReader(stream))
    if len(frames)!=actual_count or len(poses)!=actual_count: raise ValueError('bundle index/pose/metadata count mismatch')
    for index,(frame,pose) in enumerate(zip(frames,poses)):
        if frame.get('id')!=str(index) or pose.get('id')!=str(index): raise ValueError('bundle noncontiguous frame/pose IDs')
        cloud=frame.get('cloud','')
        if cloud not in seen or not cloud.startswith('keyframes/clouds/'):
            raise ValueError('body cloud absent from verified manifest')
    return dict(valid=True, bundle_uuid=metadata.get('bundle_uuid'),keyframe_count=actual_count,
                manifest_sha256=sha256(root/'manifest.sha256'), files=len(seen))


def sqlite_files(bag):
    bag=Path(bag).resolve()
    metadata=bag/'metadata.yaml'
    if metadata.is_file():
        names=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']['relative_file_paths']
        files=[]
        for name in names:
            path=bag/name
            if path.suffix!='.db3': continue
            if Path(name).is_absolute() or '..' in Path(name).parts or path.is_symlink() or path.resolve().parent!=bag:
                raise ValueError('unconfined bag storage path')
            if not path.is_file(): raise ValueError('bag storage file missing: '+str(path))
            files.append(path)
        return files
    return sorted(bag.glob('*.db3'),key=lambda path:[int(part) if part.isdigit() else part for part in re.split(r'(\d+)',path.name)])


def sqlite_rows(connection,statement,arguments=()):
    # Explicit cursor finalization matters while the recorder is writing; do
    # not rely on temporary-cursor refcounts/GC to release a read statement.
    with closing(connection.execute(statement,arguments)) as cursor:
        return cursor.fetchall()


def sqlite_inventory(bag):
    files = sqlite_files(bag)
    result = dict(topics={}, storage_files=[str(p) for p in files], missing_topics=[],lossless_proven=False,
                  note='Received counts and stamp gaps are proxies, not proof of no driver/DDS/storage loss.')
    for path in files:
        with closing(sqlite3.connect('file:' + str(path.resolve()) + '?mode=ro', uri=True)) as connection:
            for id_, name, type_ in sqlite_rows(connection,'SELECT id,name,type FROM topics'):
                count, first, last = sqlite_rows(connection,'SELECT count(*),min(timestamp),max(timestamp) FROM messages WHERE topic_id=?',(id_,))[0]
                item = result['topics'].setdefault(name, dict(type=type_, count=0,first_record_ns=None,last_record_ns=None))
                if item['type'] != type_: raise ValueError('topic type changed across bag splits')
                item['count'] += count
                if first is not None:
                    item['first_record_ns'] = first if item['first_record_ns'] is None else min(first,item['first_record_ns'])
                    item['last_record_ns'] = last if item['last_record_ns'] is None else max(last,item['last_record_ns'])
    result['missing_topics'] = [topic for topic in TOPICS if result['topics'].get(topic,{}).get('count',0) == 0]
    return result


def cdr_header(data):
    """Minimum standard CDR Header decode, shared by PointCloud2 and Imu.

    No ROS context, value conversion or stamp rewriting. Reject other encodings.
    """
    if len(data)<16 or data[:2] not in [b'\0\0',b'\0\1']: raise ValueError('unsupported/truncated CDR header')
    endian='<' if data[1]==1 else '>'
    seconds,nanoseconds,length=struct.unpack_from(endian+'iII',data,4)
    if seconds<0 or nanoseconds>=1_000_000_000 or not 1<=length<=4096 or len(data)<16+length or data[15+length]!=0:
        raise ValueError('invalid serialized Header stamp/frame length')
    return dict(source_stamp_ns=seconds*1_000_000_000+nanoseconds,
                frame_id=data[16:15+length].decode('utf-8'))


def committed_raw_evidence(bag,now_ns=None):
    """Committed sqlite rows, not probe callbacks or the rosbag writer's RAM cache."""
    now_ns=time.time_ns() if now_ns is None else now_ns
    result=dict(schema='mapping_pre_frontend_bag_evidence_v1',checked_unix_ns=now_ns,topics={},errors=[],
                valid=False,lossless_proven=False,
                scope='Read-only sqlite visible committed rows. A minimum early-input witness, not complete recording/loss proof.')
    for path in sorted(Path(bag).glob('*.db3')):
        try:
            # A poll can arrive between file creation and the writer's WAL
            # pragma. Opening a SQLite reader there could block that upgrade.
            # These header bytes establish only journal readiness, NEVER a
            # committed message witness: all evidence still comes from SQL.
            with path.open('rb') as stream: header=stream.read(20)
            if len(header)<20 or header[:16]!=b'SQLite format 3\0' or header[18:20]!=b'\x02\x02':
                result['errors'].append(str(path)+': waiting for resilient WAL initialization; no SQL reader opened')
                continue
            with closing(sqlite3.connect('file:'+str(path.resolve())+'?mode=ro',uri=True,timeout=.2,isolation_level=None)) as connection:
                if sqlite_rows(connection,'PRAGMA journal_mode')[0][0]!='wal':
                    result['errors'].append(str(path)+': actual journal_mode is not WAL')
                    continue
                for id_,topic,type_ in sqlite_rows(connection,'SELECT id,name,type FROM topics'):
                    if topic not in RAW_TOPICS: continue
                    expected='sensor_msgs/msg/'+('PointCloud2' if topic=='/lidar_points' else 'Imu')
                    if type_!=expected: raise ValueError('recorded raw topic type mismatch: '+topic)
                    count=sqlite_rows(connection,'SELECT count(*) FROM messages WHERE topic_id=?',(id_,))[0][0]
                    if not count: continue
                    record_ns,data=sqlite_rows(connection,'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 1',(id_,))[0]
                    header=cdr_header(data)
                    row=dict(header,record_stamp_ns=int(record_ns),committed_count=int(count),storage_file=str(path.resolve()),journal_mode='wal')
                    previous=result['topics'].get(topic)
                    if previous:
                        total=previous['committed_count']+count
                        row=(row if row['record_stamp_ns']>=previous['record_stamp_ns'] else dict(previous))
                        row['committed_count']=total
                    result['topics'][topic]=row
        except (OSError,sqlite3.Error,ValueError,UnicodeError) as error: result['errors'].append(str(path)+': '+str(error))
    for topic in RAW_TOPICS:
        row=result['topics'].get(topic)
        if not row: result['errors'].append(topic+' has no actually committed bag row before frontend')
        else:
            row['record_age_at_check_ns']=now_ns-row['record_stamp_ns']
            row['source_age_at_check_ns']=now_ns-row['source_stamp_ns']
            if row['source_stamp_ns']<=0 or row['record_stamp_ns']<=0 or row['record_stamp_ns']>now_ns+MAX_CLOCK_SKEW_NS:
                result['errors'].append(topic+' committed row invalid/future stamp')
    result['valid']=not result['errors']
    return result


class RecorderEndpointBinding:
    """Humble 0.15.16 fallback: exclusive default node plus new fixed raw GIDs.

    This is NOT a unique node name or a PID-to-DDS cryptographic attestation.
    The caller must also check its own process identity and committed bag rows.
    Ambiguity/drift is permanent for this recording, never silently rebound.
    """
    def __init__(self,nodes,endpoints):
        self.baseline_gids={entry.get('gid') for rows in endpoints.values() for entry in rows}
        if any(name==DEFAULT_RECORDER_NODE for name,namespace in nodes) or any(
                entry.get('node_name')==DEFAULT_RECORDER_NODE for rows in endpoints.values() for entry in rows):
            raise ValueError('existing default rosbag2_recorder detected; use a separate ROS domain or stop its owner, never attach')
        self.gids={}; self.errors=[]

    def update(self,nodes,endpoints):
        nodes=[(name,namespace) for name,namespace in nodes if name==DEFAULT_RECORDER_NODE]
        errors=[]; proposed={}
        if len(nodes)>1: errors.append('multiple default recorder nodes; ambiguous ownership')
        if any(namespace!='/' for name,namespace in nodes): errors.append('unexpected default recorder namespace')
        for topic in RAW_TOPICS:
            rows=[entry for entry in endpoints.get(topic,[]) if entry.get('node_name')==DEFAULT_RECORDER_NODE]
            if len(rows)>1: errors.append(topic+' multiple default recorder endpoints/GIDs')
            if any(entry.get('node_namespace')!='/' for entry in rows): errors.append(topic+' unexpected recorder namespace')
            if len(rows)==1:
                gid=rows[0].get('gid','')
                if not re.fullmatch('[0-9a-f]{48}',gid) or int(gid,16)==0: errors.append(topic+' missing/invalid endpoint GID')
                elif gid in self.baseline_gids: errors.append(topic+' endpoint GID predates this recorder')
                else: proposed[topic]=gid
            if self.gids and proposed.get(topic)!=self.gids.get(topic): errors.append(topic+' bound GID disappeared or changed')
        if self.gids and len(nodes)!=1: errors.append('bound recorder node disappeared or became ambiguous')
        for error in errors:
            if error not in self.errors: self.errors.append(error)
        if not self.errors and not self.gids and len(nodes)==1 and set(proposed)==set(RAW_TOPICS):
            if len(set(proposed.values()))!=2: self.errors.append('two raw topics unexpectedly share one endpoint GID')
            else: self.gids=proposed
        bound=bool(self.gids) and not self.errors
        return dict(schema='mapping_recorder_binding_v1',identity_strategy='exclusive_default_node_new_fixed_endpoint_gids',
            node_name=DEFAULT_RECORDER_NODE,node_namespace='/',unique_node_name=False,bound=bound,
            gids=dict(self.gids),errors=list(self.errors),
            subscriptions={topic:bound for topic in RAW_TOPICS},
            limitation='Graph uniqueness/new fixed GIDs plus owned PID and own committed SQL evidence; not cryptographic PID-to-DDS attestation.')


def capture_evidence_labels(synthetic_test):
    """Requested mode is not independent hardware/deployment evidence."""
    return dict(capture_mode='synthetic' if synthetic_test else 'real_requested',
        real_hardware_evidence=False,hardware_evidence_status='unverified')


def startup_summary(identity,metadata,frontend_params,session,contract,actual_start_environment=None):
    """Pure aggregation of frozen requested inputs, not estimator readiness."""
    session=Path(session)
    build=identity.get('build') or {}; source=build.get('frontend_source_manifest',{})
    v4=frontend_params.get('frontend_init_v4',{}); common=frontend_params.get('common',{})
    backend=identity.get('backend_runtime_parameters',{})
    return dict(schema='mapping_capture_startup_summary_v1',session=str(session),uuid=metadata.get('uuid'),
        **condition_labels(metadata),
        source_provenance='synthetic' if metadata.get('synthetic_test') else 'real',
        synthetic_test=bool(metadata.get('synthetic_test')),**capture_evidence_labels(bool(metadata.get('synthetic_test'))),
        map_gravity_alignment=False,
        startup_scope='Frozen requested binaries/configuration; process exec is not INIT_COMMIT or physical accuracy. Run ready.',
        binaries=copy.deepcopy(build.get('binaries',{})),
        versions=dict(repository_git_head=identity.get('source',{}).get('git_head'),
            repository_dirty=identity.get('source',{}).get('git_dirty'),frontend_base_commit=source.get('base_commit'),
            frontend_patch_sha256=source.get('patch_sha256'),build_manifests=copy.deepcopy(build.get('manifests',{}))),
        base_configuration=dict(source=identity.get('base_source'),sha256=identity.get('base_sha256'),
            frozen_copy=str(session/'snapshots/jt128.original.yaml'),
            extrinsic_evidence=copy.deepcopy(contract.get('extrinsic_evidence',{}))),
        config_snapshots=[dict(path=str(session/name),sha256=digest) for name,digest in sorted(identity.get('snapshots',{}).items())],
        gates=dict(input_interface_enabled=v4.get('input_interface_enabled'),quality_gate_enabled=v4.get('quality_gate_enabled'),
            backend_revisit_verification_enabled=backend.get('loop_pipeline',{}).get('revisit_verification_enabled')),
        input_contract=copy.deepcopy(contract),
        topics=dict(raw_lidar=common.get('lid_topic'),raw_imu=common.get('imu_topic'),recorded=list(TOPICS)),
        frames=dict(raw_imu=contract.get('imu_frame'),raw_lidar=contract.get('lidar_frame'),
            frontend_odom='camera_init',frontend_tracking='body'),
        time=dict(time_sync_en=common.get('time_sync_en'),imu_minus_lidar_offset_ns=contract.get('imu_minus_lidar_offset_ns'),
            original_time_offset_lidar_to_imu_s=common.get('time_offset_lidar_to_imu'),
            offset_definition=contract.get('offset_definition'),timestamps_rewritten_by_capture=False),
        paths=dict(ROS_LOG_DIR=str(session/'ros_logs'),ROS_HOME=str(session/'ros_home'),
            runtime_state_csv=v4.get('runtime_state_log_path'),runtime_state_capacity=v4.get('runtime_state_log_capacity'),
            input_trace=v4.get('input_trace_path'),quality_trace=v4.get('quality_trace_path'),
            bundles=backend.get('bundle_root'),save_output_path='map_bundle',
            recorder_baseline=str(session/'recorder_baseline.json'),recorder_binding=str(session/'recorder_binding.json')),
        ros_environment=dict(declared_at_init=dict(ROS_DOMAIN_ID=metadata.get('ros_domain_id'),ROS_LOCALHOST_ONLY=metadata.get('ros_localhost_only')),
            actual_at_start=copy.deepcopy(actual_start_environment),
            actual_start_scope='Allowlisted environment passed to mapping child processes at start; unset values are null. Init declarations are not current observations.'),
        recorder_identity=dict(strategy='exclusive_default_node_new_fixed_endpoint_gids',node_name=DEFAULT_RECORDER_NODE,
            node_namespace='/',unique_node_name=False,PID_and_SQL_evidence_required=True))
