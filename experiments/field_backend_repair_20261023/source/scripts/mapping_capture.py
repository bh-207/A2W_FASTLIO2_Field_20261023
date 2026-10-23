#!/usr/bin/env python3
"""Controlled, session-scoped JT128 capture. Never starts or configures a driver.

Use scripts/mapping_capture.sh to source the isolated backend environment.
ROS imports are intentionally lazy so contract checks/tests work without ROS.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
from functools import wraps
import json
import math
import os
from pathlib import Path
import re
import signal
import sqlite3
import struct
import subprocess
import sys
import time
import uuid

import yaml

from mapping_initial_conditions import (POLICY_PATH, authorize_existing_initials, condition_labels)

from mapping_capture_lib import (TOPICS, RAW_TOPICS, POINT_FIELDS, MAX_CLOCK_SKEW_NS,
    sha256, read_json, write_json, interface_template, interface_errors, parameters,
    frontend_parameters, assert_conservative, qos_overrides, record_command,
    process_identity, process_matches, runtime_ready, input_report_errors,
    verify_bundle, sqlite_inventory, InputAccumulator, runtime_shutdown,committed_raw_evidence,sqlite_files,
    base_profile_errors,flatten,RecorderEndpointBinding,DEFAULT_RECORDER_NODE,startup_summary,capture_evidence_labels)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASE = ROOT/'src/a2w_fastlio2_bringup/config/jt128.yaml'
NODE_NAMES = {'frontend': '/laser_mapping', 'backend': '/mapping_backend_node',
              'ingress': '/mapping_ingress_node'}
CONFIG_SOURCES = {
    'ingress.yaml': ROOT/'src/a2w_fastlio_mapping/config/mapping.yaml',
    **{name: ROOT/'src/a2w_fastlio_mapping/config'/name for name in
       ['mapping_topics.yaml','pose_graph.yaml','scan_context.yaml','registration.yaml','loop_validation.yaml']},
    'map.yaml': ROOT/'src/a2w_fastlio_map/config/map.yaml'}


def run(command, **kwargs):
    return subprocess.run([str(x) for x in command], check=True, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, **kwargs)


def yaml_write(path, params):
    with Path(path).open('x') as stream:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, stream, sort_keys=False)


def build_root(value=None):
    return Path(value or os.environ.get('A2W_CONTROLLED_BUILD_ROOT', ROOT/'.controlled_mapping')).resolve()


def synthetic_environment_errors():
    errors=[]
    try:
        domain=int(os.environ.get('ROS_DOMAIN_ID','0'))
        if not 1<=domain<=232: errors.append('synthetic test requires explicit nonzero ROS_DOMAIN_ID (1..232)')
    except ValueError: errors.append('invalid ROS_DOMAIN_ID')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1': errors.append('synthetic test requires ROS_LOCALHOST_ONLY=1')
    for name in ['CYCLONEDDS_URI','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML']:
        if os.environ.get(name): errors.append('synthetic test forbids external DDS profile environment: '+name)
    return errors


def provenance(metadata):
    return 'synthetic' if metadata.get('synthetic_test') is True else 'real'


def ensure_ros_environment(session):
    session,metadata=load_session(session)
    if metadata.get('synthetic_test'):
        errors=synthetic_environment_errors()
        if errors: raise ValueError('; '.join(errors))
    for key,directory in [('ROS_LOG_DIR','ros_logs'),('ROS_HOME','ros_home')]:
        path=session/directory
        if path.is_symlink(): raise ValueError('session ROS output directory is symlink: '+str(path))
        path.mkdir(exist_ok=True)
        os.environ[key]=str(path)
    return session


@contextmanager
def session_lock(session):
    with (Path(session)/'.lifecycle.lock').open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def mapping_operation(function):
    """Serialize start/save/stop across CLI processes, but never block the recorder."""
    @wraps(function)
    def locked(session,*args,**kwargs):
        session,_=load_session(session)
        with (session/'.mapping_operation.lock').open('a') as stream:
            fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
            return function(session,*args,**kwargs)
    return locked


def load_session(session):
    session = Path(session).resolve(strict=True)
    metadata = read_json(session/'session.json')
    if metadata.get('schema') != 'mapping_capture_session_v1' or metadata.get('session') != str(session):
        raise ValueError('invalid/moved session; absolute paths are identity-bound')
    return session, metadata


def parse_ldd(output):
    dsos = []
    if 'not found' in output: raise ValueError('ldd has unresolved dependency')
    for line in output.splitlines():
        match = re.search(r'(?:=>\s*)?(/[^\s]+)\s+\(0x[0-9a-fA-F]+\)',line)
        if match:
            original = Path(match.group(1))
            resolved = original.resolve(strict=True)
            dsos.append(dict(path=str(resolved), requested_path=str(original),sha256=sha256(resolved)))
    if not dsos: raise ValueError('ldd produced no resolved libraries')
    return dsos


def binary_identity(path, expected_hash, role, isolated_install):
    path = Path(path).resolve(strict=True)
    if sha256(path) != expected_hash: raise ValueError(role + ' build binary hash mismatch')
    result = run(['ldd','-r',str(path)])
    if 'undefined symbol:' in result.stdout: raise ValueError(role+' unresolved binary/DSO relocation: '+result.stdout)
    dsos = parse_ldd(result.stdout)
    common = (Path(isolated_install)/'a2w_fastlio_common/lib').resolve()
    required = {'liba2w_algorithm_suite.so','liba2w_registration.so','liba2w_runtime_contracts.so'}
    found = set()
    for dso in dsos:
        target = Path(dso['path']); name = Path(dso['requested_path']).name
        if name.startswith('liba2w_'):
            if role == 'frontend': raise ValueError('frontend unexpectedly links backend libraries')
            if name in required:
                expected_parent=common
                found.add(name)
            elif name.startswith('liba2w_fastlio_msgs__'):
                expected_parent=(Path(isolated_install)/'a2w_fastlio_msgs/lib').resolve()
            elif name.startswith('liba2w_fastlio_map.'):
                expected_parent=(Path(isolated_install)/'a2w_fastlio_map/lib').resolve()
            else: raise ValueError('unknown controlled a2w DSO: '+name)
            if target.parent != expected_parent: raise ValueError('backend DSO escaped isolated package/lib: ' + str(target))
        # Main-workspace artifacts must never be accidentally selected through LD_LIBRARY_PATH.
        if target.is_relative_to(ROOT/'install') or target.is_relative_to(ROOT/'build'):
            raise ValueError('main workspace library contamination: ' + str(target))
    if role == 'backend' and not required <= found:
        raise ValueError('backend required libraries missing from ldd: ' + str(required-found))
    return dict(path=str(path),sha256=sha256(path),dsos=dsos,ldd=result.stdout)


def assert_build_libraries(role, identity, declared):
    actual={Path(item['requested_path']).name:item for item in identity['dsos']}
    for name, expected in declared.items():
        current=actual.get(name)
        if (current is None or Path(current['path']).resolve()!=Path(expected['path']).resolve() or
                current['sha256']!=expected['sha256']):
            raise ValueError(role+' build/runtime library mismatch for '+name+': expected '+str(expected)+'; got '+str(current))


def capture_build_identity(root):
    manifest_path = root/'build_identity.json'
    build = read_json(manifest_path)
    isolated_install = Path(build.get('backend_install',root/'backend_install')).resolve(strict=True)
    if isolated_install != (root/'backend_install').resolve(): raise ValueError('unexpected backend_install outside controlled root')
    frontend_manifest = Path(build.get('frontend_build_manifest',root/'frontend/build_manifest.json')).resolve(strict=True)
    isolation=Path(build.get('frontend_isolation',frontend_manifest.parent)).resolve(strict=True)
    if frontend_manifest.parent!=isolation: raise ValueError('frontend manifest/isolation disagreement')
    frontend = read_json(frontend_manifest)['binaries']['fastlio_mapping']
    selected_frontend = build.get('frontend_binary', frontend['path'])
    if Path(selected_frontend).resolve() != Path(frontend['path']).resolve(): raise ValueError('frontend manifest disagreement')
    roles = dict(frontend=frontend,
        backend=dict(path=str(isolated_install/'a2w_fastlio_mapping/lib/a2w_fastlio_mapping/mapping_backend_node')),
        ingress=dict(path=str(isolated_install/'a2w_fastlio_mapping/lib/a2w_fastlio_mapping/mapping_ingress_node')))
    output = dict(build_root=str(root),backend_install=str(isolated_install),
                  manifests={str(manifest_path):sha256(manifest_path),str(frontend_manifest):sha256(frontend_manifest)},binaries={})
    isolation_manifest=isolation/'isolation_manifest.json'
    if isolation_manifest.is_file():
        output['manifests'][str(isolation_manifest)]=sha256(isolation_manifest)
        source_manifest=read_json(isolation_manifest)
        declared=read_json(frontend_manifest)
        if source_manifest.get('patch_sha256') != declared.get('patch_sha256'):
            raise ValueError('frontend isolation/build patch disagreement')
        patch=ROOT/'patches/fast_lio_v4/0001-frontend-init-quality-units.patch'
        if declared.get('patch_sha256')!=sha256(patch): raise ValueError('frontend patch differs from current controlled source')
        if build.get('patch_sha256',declared.get('patch_sha256'))!=declared.get('patch_sha256'):
            raise ValueError('build/frontend patch disagreement')
        output['frontend_source_manifest']=source_manifest
    for role, expected in roles.items():
        entry = build.get('binaries',{}).get(role,{})
        path = entry.get('path',expected['path'])
        if Path(path).resolve() != Path(expected['path']).resolve(): raise ValueError(role+' path disagreement')
        expected_hash = entry.get('sha256',expected.get('sha256'))
        if not expected_hash: raise ValueError(role+' build manifest hash absent')
        if role=='frontend' and expected_hash != frontend['sha256']: raise ValueError('frontend manifest hash disagreement')
        output['binaries'][role] = binary_identity(path,expected_hash,role,isolated_install)
        assert_build_libraries(role,output['binaries'][role],entry.get('libraries',{}))
    return output


def file_identity_errors(entry):
    errors = []
    for item in [entry,*entry.get('dsos',[])]:
        try:
            path = Path(item['path'])
            if sha256(path) != item['sha256']: errors.append('binary/DSO hash changed: '+str(path))
            if item.get('requested_path') and Path(item['requested_path']).resolve() != path:
                errors.append('DSO symlink target changed: '+item['requested_path'])
        except (OSError,KeyError) as error: errors.append('binary/DSO unavailable: '+str(error))
    return errors


def source_identity():
    result = dict(files={})
    for directory in ['src','patches','frontend','scripts','experiments/field_fixed_20261023']:
        for path in sorted((ROOT/directory).rglob('*')):
            if path.is_file() and path.suffix in ['.hpp','.h','.cpp','.c','.yaml','.py','.sh','.patch','.xml','.txt'] and '.git' not in path.parts:
                result['files'][str(path.relative_to(ROOT))] = sha256(path)
    for name in ['mapping_capture.py', 'mapping_capture_lib.py', 'mapping_capture.sh',
                 'mapping_initial_conditions.py', 'mapping_capture_experiment.py', 'mapping_capture_experiment.sh']:
        path = ROOT/'scripts'/name
        if path.is_file(): result['files'][str(path.relative_to(ROOT))] = sha256(path)
    try:
        result['git_head'] = run(['git','-C',ROOT,'rev-parse','HEAD']).stdout.strip()
        result['git_dirty'] = bool(run(['git','-C',ROOT,'status','--porcelain','--untracked-files=no']).stdout)
        result['submodule_versions'] = run(['git','-C',ROOT,'submodule','status','--recursive']).stdout
    except subprocess.CalledProcessError: result['git_head'] = 'unavailable'
    return result


def init_session(name, sessions_root, controlled_root, interface_path, base=DEFAULT_BASE,synthetic_test=False,
                 experimental_policy=None,causal_geometry_guard=False):
    if not re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.-]{0,100}',name) or name in ['.','..']:
        raise ValueError('session name must be a single safe path component')
    if synthetic_test and synthetic_environment_errors(): raise ValueError('; '.join(synthetic_environment_errors()))
    base = Path(base).resolve(strict=True)
    profile_errors=base_profile_errors(base)
    if profile_errors: raise ValueError('; '.join(profile_errors))
    contract = read_json(interface_path) if interface_path else interface_template()
    if synthetic_test and interface_path is None: contract['source_provenance']='synthetic'
    if experimental_policy is not None:
        if synthetic_test: raise ValueError('existing JT128 experimental initial conditions require real input')
        contract, expected_policy = authorize_existing_initials(contract, base, parameters(base), sha256(base),
            experimental_policy.get('authorization_note'), authorized=experimental_policy.get('enabled'))
        if expected_policy != experimental_policy: raise ValueError('experimental policy differs from actual authorization/configuration')
    root = Path(sessions_root).resolve(); root.mkdir(parents=True,exist_ok=True)
    session = root/name
    session.mkdir(exist_ok=False)
    for directory in ['snapshots','logs','ros_logs','ros_home','bundles']: (session/directory).mkdir()
    write_json(session/'interface.json',contract,exclusive=True)
    if experimental_policy is not None: write_json(session/POLICY_PATH,experimental_policy,exclusive=True)
    (session/'snapshots/jt128.original.yaml').write_bytes(base.read_bytes())
    for name_, source in CONFIG_SOURCES.items():
        params = parameters(source); assert_conservative(params)
        with (session/'snapshots'/name_).open('xb') as stream: stream.write(source.read_bytes())
    actual_frontend=frontend_parameters(base,contract,session)
    yaml_write(session/'snapshots/frontend.yaml',actual_frontend)
    original_flat=flatten(parameters(base)); actual_flat=flatten(actual_frontend)
    write_json(session/'snapshots/frontend_overrides.json',dict(schema='mapping_frontend_overrides_v1',
        base_source=str(base),base_sha256=sha256(base),changes={name:dict(original=original_flat.get(name),snapshot=value)
            for name,value in actual_flat.items() if name not in original_flat or original_flat[name]!=value},
        scope='Strict input/quality/runtime logs and disabled duplicate outputs only; mapping extrinsic and filtering unchanged.'),exclusive=True)
    backend_runtime=dict(bundle_root=str(session/'bundles'),use_sim_time=False,
                         factor_ledger_path=str(session/'exact_factor_ledger.csv'),
        loop_pipeline={'revisit_verification_enabled':False,
            'causal_geometry_guard_enabled':bool(causal_geometry_guard),
            'causal_geometry_guard_ledger_path':str(session/'causal_geometry_guard.jsonl') if causal_geometry_guard else ''},
        quatro={'feature_cache_capacity':0})
    yaml_write(session/'snapshots/backend_runtime.yaml',backend_runtime)
    with (session/'snapshots/record_qos.yaml').open('x') as stream: yaml.safe_dump(qos_overrides(),stream)
    build = None; build_error = None
    try: build = capture_build_identity(Path(controlled_root).resolve())
    except (OSError,ValueError,KeyError,subprocess.SubprocessError) as error: build_error = str(error)
    identity = dict(schema='mapping_capture_identity_v1',build=build,build_error=build_error,
                    experimental_initials_authorized=experimental_policy is not None,
                    backend_runtime_parameters=backend_runtime,
                    base_source=str(base),base_sha256=sha256(base),source=source_identity(),
                    snapshots={str(p.relative_to(session)):sha256(p) for p in sorted((session/'snapshots').iterdir())},
                    interface_sha256=sha256(session/'interface.json'))
    write_json(session/'identity.json',identity,exclusive=True)
    metadata = dict(schema='mapping_capture_session_v1',session=str(session),uuid=uuid.uuid4().hex,
        created_utc=datetime.now(timezone.utc).isoformat(),build_root=str(Path(controlled_root).resolve()),
        synthetic_test=synthetic_test,source_provenance='synthetic' if synthetic_test else 'real',
        **capture_evidence_labels(synthetic_test),
        ros_domain_id=os.environ.get('ROS_DOMAIN_ID','0'),ros_localhost_only=os.environ.get('ROS_LOCALHOST_ONLY','0'),
        input_contract_valid=not interface_errors(contract,base,'synthetic' if synthetic_test else 'real',experimental_policy),map_gravity_alignment=False,
        calibration_status=contract.get('calibration_status','not_independently_certified'),physical_calibration_verified=False,
        experimental_initials_authorized=experimental_policy is not None,
        note='No driver is launched. Unknown input contract permits raw recording only. No lossless claim.')
    write_json(session/'session.json',metadata,exclusive=True)
    write_json(session/'processes.json',{},exclusive=True)
    return session


def check_session(session):
    session,metadata = load_session(session)
    identity = read_json(session/'identity.json'); contract = read_json(session/'interface.json')
    errors = []
    experimental_policy=None
    if bool(metadata.get('experimental_initials_authorized')) != bool(identity.get('experimental_initials_authorized')):
        errors.append('experimental authorization metadata/identity mismatch')
    if metadata.get('experimental_initials_authorized') is True:
        if POLICY_PATH not in identity['snapshots']: errors.append('experimental authorization snapshot is not hash-bound')
        try: experimental_policy=read_json(session/POLICY_PATH)
        except (OSError,ValueError) as error: errors.append('experimental authorization unavailable: '+str(error))
    for name,digest in identity['snapshots'].items():
        try:
            if sha256(session/name) != digest: errors.append('snapshot hash changed: '+name)
        except OSError: errors.append('snapshot missing: '+name)
    if sha256(session/'interface.json') != identity['interface_sha256']: errors.append('interface changed; create a new session')
    contract_errors = interface_errors(contract,session/'snapshots/jt128.original.yaml',provenance(metadata),experimental_policy)
    if metadata.get('synthetic_test'): errors.extend(synthetic_environment_errors())
    build_errors = []
    build = identity.get('build')
    if not build: build_errors.append('controlled build unavailable at init: '+str(identity.get('build_error')))
    else:
        for role,entry in build['binaries'].items():
            build_errors.extend(file_identity_errors(entry))
            try:
                current = binary_identity(entry['path'],entry['sha256'],role,build['backend_install'])
                if current['dsos'] != entry['dsos']: build_errors.append(role+' ldd identity changed')
            except (OSError,ValueError,subprocess.SubprocessError) as error: build_errors.append(str(error))
        for path,digest in build['manifests'].items():
            if not Path(path).is_file() or sha256(path)!=digest: build_errors.append('build manifest changed: '+path)
    for name in identity['snapshots']:
        if name.endswith('.yaml') and name not in ['snapshots/frontend.yaml','snapshots/jt128.original.yaml','snapshots/record_qos.yaml']:
            try: assert_conservative(parameters(session/name))
            except (ValueError,TypeError,yaml.YAMLError) as error: errors.append(str(error))
    processes = read_json(session/'processes.json')
    return dict(session=str(session),record_allowed=not errors,start_allowed=not(errors+contract_errors+build_errors),
                **condition_labels(metadata),
                errors=errors,contract_errors=contract_errors,build_errors=build_errors,
                alive={name:process_matches(value) for name,value in processes.items()},map_gravity_alignment=False)


def session_env(session):
    env = os.environ.copy(); env['ROS_LOG_DIR'] = str(Path(session)/'ros_logs')
    env['ROS_HOME']=str(Path(session)/'ros_home')
    return env


def register_process(session, role, identity):
    identity.update(owner_session=str(Path(session).resolve()),role=role)
    assert_owned_session_process(session,role,identity)
    with session_lock(session):
        state = read_json(Path(session)/'processes.json')
        if role in state and process_matches(state[role]): raise ValueError('session role already alive: '+role)
        state[role] = identity; write_json(Path(session)/'processes.json',state)


def assert_owned_session_process(session,role,identity):
    session=Path(session).resolve()
    if identity.get('owner_session')!=str(session) or identity.get('role')!=role:
        raise ValueError('process ownership marker does not match this session/role')
    try: cwd=Path('/proc')/str(identity['pid'])/'cwd'; cwd=cwd.resolve(strict=True)
    except OSError as error: raise ValueError('cannot resolve owned process cwd') from error
    bound=False
    for arg in identity.get('cmdline',[]):
        if arg.startswith('-'): continue
        path=Path(arg)
        if not path.is_absolute(): path=cwd/path
        candidate=path.resolve()
        if candidate==session or candidate.is_relative_to(session/'snapshots'): bound=True
    if not bound: raise ValueError('process cmdline is not bound to this session path; refusing signal/start')


def spawn_owned(session,role,command):
    session=Path(session).resolve()
    command=[str(arg) for arg in command]
    supervisor=Path(__file__).with_name('capture_process_supervisor.py')
    with (session/'logs'/(role+'.supervisor.log')).open('x') as errors:
        child=subprocess.Popen([sys.executable,str(supervisor),str(session),role,*command],
            env=session_env(session),start_new_session=True,stdout=subprocess.DEVNULL,stderr=errors)
    started=session/'process_exits'/(role+'.started.json')
    status_path=session/'process_exits'/(role+'.json')
    failed_path=session/'process_exits'/(role+'.startup_failed.json')

    def evidence(path):
        try: report=read_json(path)
        except (OSError,ValueError): return None
        if (report.get('owner_session')!=str(session) or report.get('role')!=role or
                report.get('supervisor_pid')!=child.pid or report.get('requested_command')!=command):
            raise ValueError('startup evidence ownership mismatch: '+str(path))
        return report

    def startup_failure(reason,settle=.25):
        try: child.wait(timeout=settle)
        except subprocess.TimeoutExpired: pass
        report=evidence(status_path)
        actual=report.get('exit_code') if report else 'pending/unknown; supervisor retains waiter'
        return ValueError(role+' startup failed: '+reason+'; actual child exit='+str(actual)+
            '; actual_status='+str(status_path)+'; startup_failure='+str(failed_path)+
            '; supervisor_pid='+str(child.pid)+'; supervisor_exit='+str(child.returncode))

    # Wait only for actual exec identity, never algorithm or DDS readiness.
    end = time.monotonic()+5
    while time.monotonic()<end:
        status=evidence(status_path)
        if status is not None:
            raise startup_failure(status.get('startup_error') or status.get('outcome','child exited before start'))
        failure=evidence(failed_path)
        if failure is not None:
            raise startup_failure(failure.get('startup_error','exec identity was not verified'))
        if started.is_file():
            try: identity=read_json(started)
            except (OSError,ValueError): time.sleep(.005); continue
            if (identity.get('owner_session')!=str(session) or identity.get('role')!=role or
                    identity.get('supervisor_pid')!=child.pid or identity.get('cmdline')!=command):
                raise startup_failure('unverified startup identity')
            try: register_process(session,role,identity)
            except BaseException as error:
                if process_matches(identity): signal_owned(identity,signal.SIGINT)
                raise startup_failure(str(error)) from error
            if not process_matches(identity):
                raise startup_failure('verified child exited before start returned')
            return child,identity
        if child.poll() is not None: break
        time.sleep(.01)
    raise startup_failure('no running verified exec identity observed')


def owned_exit_report(session,role,identity):
    result=dict(valid=False,exit_code=None,errors=[])
    path=Path(session)/'process_exits'/(role+'.json')
    try:
        status=read_json(path)
        if status.get('schema')!='owned_process_exit_v1' or status.get('identity')!=identity:
            raise ValueError('waiter identity/schema mismatch')
        result.update(status)
        if status.get('exit_code')!=0: result['errors'].append('actual child exit is nonzero')
        if status.get('log_flushed') is not True or status.get('log_closed') is not True:
            result['errors'].append('process log not flushed and closed')
    except (OSError,ValueError) as error: result['errors'].append('no trustworthy waitpid status: '+str(error))
    result['valid']=not result['errors']
    return result


def signal_owned(identity,signal_number):
    if not process_matches(identity): raise ValueError('PID identity changed; refusing signal')
    # pidfd removes the PID-reuse race between validation and kill on Linux.
    descriptor = os.pidfd_open(identity['pid'])
    try:
        if not process_matches(identity): raise ValueError('PID identity changed after pidfd open')
        signal.pidfd_send_signal(descriptor,signal_number)
    finally: os.close(descriptor)


@mapping_operation
def start_session(session):
    session,metadata = load_session(session)
    status = check_session(session)
    if not status['start_allowed']: raise ValueError('start refused: '+json.dumps(status))
    processes = read_json(session/'processes.json')
    if not all(process_matches(processes.get(key,{})) for key in ['recorder','record_manager']):
        raise ValueError('same-session foreground recorder and manager must be alive before frontend')
    for role in ['recorder','record_manager']: assert_owned_session_process(session,role,processes[role])
    if any(process_matches(processes.get(key,{})) for key in NODE_NAMES): raise ValueError('session nodes already running')
    report = read_json(session/'raw_input_report.json')
    assert_same_recorder_report(processes,report)
    if 'nodes' not in report: raise ValueError('input report ROS graph inventory missing')
    assert_no_node_collisions(report['nodes'])
    errors = input_report_errors(report,read_json(session/'interface.json'),session)
    if errors: raise ValueError('start input check refused: '+json.dumps(errors))
    bag_evidence=committed_raw_evidence(session/'bag')
    write_json(session/'pre_frontend_bag_evidence.json',bag_evidence)
    if not bag_evidence['valid']: raise ValueError('no committed early raw bag evidence: '+json.dumps(bag_evidence['errors']))
    if (session/'runtime_state.csv').exists(): raise ValueError('session already attempted frontend; create new session')
    identity_snapshot=read_json(session/'identity.json')
    build = identity_snapshot['build']
    summary=startup_summary(identity_snapshot,metadata,parameters(session/'snapshots/frontend.yaml'),
        session,read_json(session/'interface.json'),actual_start_environment={key:session_env(session).get(key)
            for key in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','ROS_DISTRO','ROS_LOG_DIR','ROS_HOME']})
    filenames = {'frontend':['frontend.yaml'],'ingress':['ingress.yaml'],
        'backend':['mapping_topics.yaml','pose_graph.yaml','scan_context.yaml','registration.yaml','loop_validation.yaml','map.yaml','backend_runtime.yaml']}
    launched = []
    try:
        write_json(session/'startup_summary.json',summary,exclusive=True)
        for role in ['backend','ingress','frontend']:
            command = [build['binaries'][role]['path'],'--ros-args']
            for filename in filenames[role]: command += ['--params-file',str(session/'snapshots'/filename)]
            child,identity = spawn_owned(session,role,command); launched.append(identity)
    except BaseException:
        for identity in reversed(launched):
            if process_matches(identity): signal_owned(identity,signal.SIGINT)
        raise
    write_json(session/'started.json',dict(started_unix_ns=time.time_ns(),map_gravity_alignment=False))
    return dict(started=True,session=str(session),nodes=NODE_NAMES,map_gravity_alignment=False,
                startup_summary=summary,
                instruction='Run ready. Save successfully before stop. Odometry alone is not readiness.')


def assert_same_recorder_report(processes,report):
    if report.get('recorder_identity')!=processes.get('recorder'):
        raise ValueError('input report recorder identity is not the live same-session recorder')
    bound=report.get('recorder_binding',{})
    if bound.get('bound') is not True or bound.get('errors') or bound.get('identity_strategy')!='exclusive_default_node_new_fixed_endpoint_gids' or bound.get('node_name')!=DEFAULT_RECORDER_NODE or bound.get('node_namespace')!='/':
        raise ValueError('exclusive default recorder/fixed new endpoint GID binding not established')
    gids=bound.get('gids',{})
    if set(gids)!=set(RAW_TOPICS) or len(set(gids.values()))!=2 or any(not re.fullmatch('[0-9a-f]{48}',gid) or int(gid,16)==0 for gid in gids.values()):
        raise ValueError('two valid distinct raw endpoint GIDs required')


def assert_no_node_collisions(nodes):
    found={'/'+('/'.join([namespace.strip('/'),name]).strip('/')) for name,namespace in nodes}
    collisions=found.intersection(NODE_NAMES.values())
    if collisions: raise ValueError('mapping node names already present; never attach to another session: '+str(sorted(collisions)))


def validate_save_response(response,status):
    if response.get('success') is not True: raise ValueError('save service did not report success: '+str(response))
    reason = status.get('reason','')
    if status.get('stage') != 'backend' or not all(re.search(r'(?:^|\s)'+item+r'(?:\s|$)',reason) for item in ['pending=0','active=none','state=saved']):
        raise ValueError('backend saved/pending=0/active=none evidence absent: '+str(status))
    counters = re.findall(r'(?:^|\s)(received|processed)=(\d+)(?=\s|$)',reason)
    if len(counters)!=2 or set(name for name,_ in counters)!={'received','processed'}:
        raise ValueError('unique received/processed drain counters required: '+str(status))
    counts={name:int(value) for name,value in counters}
    if counts['received']!=counts['processed']: raise ValueError('received != processed at save: '+str(status))
    return dict(**counts,pending=0,active='none',loop_tasks='synchronous with keyframes; no in-flight keyframe')


@mapping_operation
def stop_session(session,discard_unsaved=False,timeout=60):
    session,metadata = load_session(session)
    with session_lock(session):
        processes = read_json(session/'processes.json')
        running_nodes = any(process_matches(processes.get(role,{})) for role in NODE_NAMES)
        if running_nodes and not (session/'save_success.json').is_file() and not discard_unsaved:
            raise ValueError('save successfully before stop, or explicitly --discard-unsaved (data may not form a MapBundle)')
        if (session/'save_success.json').is_file() and running_nodes:
            saved = read_json(session/'save_success.json')
            verify_bundle(session/'bundles/map_bundle',saved['response']['bundle_uuid'],saved['response']['keyframe_count'])
        result = {}
        for role in ['frontend','ingress','backend','field_observer','recorder']:
            identity = processes.get(role)
            if not identity: continue
            if process_matches(identity):
                assert_owned_session_process(session,role,identity)
                signal_owned(identity,signal.SIGINT)
            end = time.monotonic()+timeout
            while (process_matches(identity) or not (session/'process_exits'/(role+'.json')).exists()) and time.monotonic()<end: time.sleep(.1)
            if process_matches(identity):
                result[role]=dict(valid=False,errors=['still alive; no SIGKILL issued']); break
            result[role]=owned_exit_report(session,role,identity)
        closures={}
        if 'frontend' in processes:
            closures['runtime']=runtime_shutdown(session/'runtime_state.csv',session/'logs/frontend.log')
            try:
                status=read_json(session/'frontend_exit_status.json')
                valid=(status.get('schema')=='frontend_exit_status_v1' and status.get('exit_code')==0
                    and all(status.get(key) is True for key in ['runtime_trace_closed','runtime_trace_flush_ok','handoff_trace_closed','handoff_trace_flush_ok']))
                closures['frontend']=dict(valid=valid,status=status)
            except (OSError,ValueError) as error: closures['frontend']=dict(valid=False,error=str(error))
        if 'recorder' in processes:
            manager=processes.get('record_manager')
            if manager:
                end=time.monotonic()+timeout
                while not (session/'record_closed.json').exists() and time.monotonic()<end: time.sleep(.1)
            try:
                status=read_json(session/'record_closed.json')
                closures['recorder']=dict(valid=status.get('exit_code')==0 and status.get('metadata_present') is True,status=status)
            except (OSError,ValueError) as error: closures['recorder']=dict(valid=False,error=str(error))
        report=dict(schema='ordered_capture_stop_v2',stopped_unix_ns=time.time_ns(),
            discard_unsaved=discard_unsaved,processes=result,closures=closures,
            success=bool(result) and all(r.get('valid') is True for r in result.values()) and all(r.get('valid') is True for r in closures.values()))
        write_json(session/'stop.json',report)
    return report


def dump_parameters(session):
    ensure_ros_environment(session)
    result = {}
    for role,node in NODE_NAMES.items():
        target = Path(session)/'snapshots'/('actual_'+role+'.yaml')
        try:
            output = run(['ros2','param','dump',node],timeout=15)
            with target.open('x') as stream: stream.write(output.stdout)
            result[role] = dict(path=str(target),sha256=sha256(target))
        except (OSError,subprocess.SubprocessError) as error: result[role] = dict(error=str(error))
    write_json(Path(session)/'actual_parameters.json',result)
    return result


def ready_session(session,timeout=0):
    session,metadata = load_session(session)
    end = time.monotonic()+timeout
    while True:
        result = runtime_commit_report(session)
        state = read_json(session/'processes.json')
        if not all(process_matches(state.get(role,{})) for role in NODE_NAMES):
            result.update(ready=False,reason='one or more owned mapping processes not alive')
        degradation=capture_degradation(session)
        if degradation: result.update(ready=False,reason='capture degraded: '+ '; '.join(degradation),capture_degraded=True,capture_degraded_reasons=degradation)
        if result['ready'] or time.monotonic()>=end: break
        time.sleep(.1)
    if result['ready'] and not (session/'actual_parameters.json').exists(): result['actual_parameter_dumps'] = dump_parameters(session)
    write_json(session/'ready.json',result)
    return result


def runtime_commit_report(session):
    _,session_metadata=load_session(session)
    path=Path(session)/'runtime_state.csv'
    result=runtime_ready(path)
    try:
        metadata=read_json(str(path)+'.metadata.json')
        if metadata.get('schema')!='frontend_runtime_state_v4_v1' or metadata.get('gravity_alignment') is not False or metadata.get('source_provenance')!=provenance(session_metadata) or metadata.get('frames')!={'O':'camera_init','I':'body','L':'LiDAR'}:
            raise ValueError('runtime metadata/frame/provenance/gravity-alignment contract mismatch')
        result['runtime_metadata_sha256']=sha256(str(path)+'.metadata.json')
    except (OSError,ValueError) as error: result.update(ready=False,reason=str(error))
    return result


def recorder_log_report(path):
    path=Path(path)
    result=dict(path=str(path),checked=False,loss_reported=False,reported_totals=[],errors=[],
        scope='Explicit rosbag cache-loss reports only. Totals are retained per report, not summed; absence is not lossless proof.')
    try: contents=path.read_text(errors='replace')
    except OSError as error:
        result['errors'].append('recorder log unreadable: '+str(error)); return result
    result['checked']=True
    result['reported_totals']=[int(value) for value in re.findall(r'(?m)^Total lost:\s*(\d+)\s*$',contents)]
    result['loss_reported']=('Cache buffers lost messages per topic:' in contents or
                             any(value>0 for value in result['reported_totals']))
    if result['loss_reported']:
        result['errors'].append('recorder cache loss explicitly reported; per-report totals='+str(result['reported_totals']))
    return result


def capture_degradation(session):
    session,metadata=load_session(session)
    errors=recorder_log_report(session/'logs/recorder.log')['errors']
    state=read_json(session/'processes.json')
    for role in ['recorder','record_manager']:
        if not state.get(role) or not process_matches(state[role]): errors.append(role+' not alive')
        else:
            try: assert_owned_session_process(session,role,state[role])
            except ValueError as error: errors.append(str(error))
    try:
        report=read_json(session/'raw_input_report.json'); assert_same_recorder_report(state,report)
        errors.extend(input_report_errors(report,read_json(session/'interface.json'),session))
    except (OSError,ValueError) as error: errors.append(str(error))
    return errors


def bag_child(session,compression,cache_size_mib=16):
    """Official Humble CLI. Its 0.15.16 native API has a fixed default node name.

    RecordOptions.node_prefix does NOT rename that node; the manager uses
    exclusive default-node/new fixed endpoint GIDs plus owned PID/SQL evidence.
    """
    session,metadata = load_session(session)
    ensure_ros_environment(session)
    import ros2bag.verb.record as record_verb
    from ros2cli.cli import main as ros2main
    if not hasattr(record_verb,'RecordVerb'): raise ValueError('installed Humble ros2bag RecordVerb missing')
    arguments=record_command(session,compression,cache_size_mib)[1:]
    # Humble BagCommand.add_arguments does not forward argv to its nested lazy
    # parser. Keep both argument sources identical, without leaking our private
    # _bag-child command into ros2cli (and restore the caller even on --help).
    previous_argv=sys.argv
    try:
        sys.argv=['ros2',*arguments]
        return ros2main(argv=arguments)
    finally:
        sys.argv=previous_argv


def pointcloud_summary(message):
    fields=[dict(name=field.name,offset=int(field.offset),datatype=int(field.datatype),count=int(field.count)) for field in message.fields]
    result=dict(fields=fields,width=int(message.width),height=int(message.height),point_step=int(message.point_step),
                row_step=int(message.row_step),is_bigendian=bool(message.is_bigendian),errors=[])
    indexed={field['name']:field for field in fields}
    for name,datatype in POINT_FIELDS.items():
        if name not in indexed or indexed[name]['datatype']!=datatype or indexed[name]['count']!=1:
            result['errors'].append('required Hesai PointField missing/wrong datatype: '+name)
    widths={1:1,2:1,3:2,4:2,5:4,6:4,7:4,8:8}
    for field in fields:
        if field['offset']<0 or field['count']<=0 or field['datatype'] not in widths or field['offset']+widths.get(field['datatype'],0)*field['count']>message.point_step:
            result['errors'].append('PointField exceeds point_step or unsupported datatype: '+field['name'])
    if len(indexed)!=len(fields): result['errors'].append('duplicate PointField names')
    if message.width==0 or message.height==0: result['errors'].append('empty PointCloud2')
    if message.point_step<=0 or message.row_step<message.width*message.point_step or len(message.data)<message.height*message.row_step:
        result['errors'].append('PointCloud2 layout/data size invalid')
    if not result['errors']:
        try:
            timestamp=indexed['timestamp']['offset']
            if timestamp+8>message.point_step: raise ValueError('timestamp exceeds point_step')
            endian='>' if message.is_bigendian else '<'
            indices=[0,message.width*message.height-1]
            values=[]
            for index in indices:
                offset=(index//message.width)*message.row_step+(index%message.width)*message.point_step+timestamp
                value=struct.unpack_from(endian+'d',message.data,offset)[0]
                if not math.isfinite(value): raise ValueError('nonfinite point timestamp')
                values.append(value)
            result.update(point_timestamp_first_s=values[0],point_timestamp_last_s=values[-1],point_timestamp_span_s=values[-1]-values[0])
            if not 0<=values[-1]-values[0]<=5: result['errors'].append('first/last point timestamps reversed or span exceeds 5s')
        except (ValueError,struct.error) as error: result['errors'].append(str(error))
    result['timestamp_scope']='First/last points only, parser seconds; not a per-point loss or complete timing proof.'
    return result


def stamp_ns(header):
    return int(header.stamp.sec)*1_000_000_000+int(header.stamp.nanosec)


def message_stamps(topic,message):
    if hasattr(message,'header'): return [stamp_ns(message.header)]
    if topic in ['/tf','/tf_static']: return [stamp_ns(transform.header) for transform in message.transforms]
    if topic=='/mapping/registration_status': return [int(message.stamp.sec)*1_000_000_000+int(message.stamp.nanosec)]
    return []


def observe_raw(accumulator,topic,message,receipt_ns):
    stamp=stamp_ns(message.header)
    errors=[]; extra={}; fields=None
    if topic=='/lidar_points':
        summary=pointcloud_summary(message); fields=summary.pop('fields'); errors=summary.pop('errors'); extra=summary
        if 'point_timestamp_first_s' in extra and abs(extra['point_timestamp_first_s']*1e9-stamp)>MAX_CLOCK_SKEW_NS:
            errors.append('point timestamp/header clock disagreement exceeds 2s (no timestamp rewrite)')
    else:
        values=[getattr(vector,axis) for vector in [message.angular_velocity,message.linear_acceleration] for axis in 'xyz']
        if not all(math.isfinite(float(value)) for value in values): errors.append('nonfinite raw IMU value')
        def finite_value(value): return float(value) if math.isfinite(float(value)) else None
        extra={'last_angular_velocity':[finite_value(getattr(message.angular_velocity,axis)) for axis in 'xyz'],
               'last_linear_acceleration':[finite_value(getattr(message.linear_acceleration,axis)) for axis in 'xyz']}
    accumulator.observe(topic,stamp,message.header.frame_id,receipt_ns,fields=fields,errors=errors,extra=extra)


def ros_context(name,session):
    ensure_ros_environment(session)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.node import Node
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    return rclpy,Node(name)


def publisher_qos_report(info,subscription_qos,check):
    compatibility,reason=check(info.qos_profile,subscription_qos)
    return dict(node_name=info.node_name,node_namespace=info.node_namespace,
        reliability=str(info.qos_profile.reliability),durability=str(info.qos_profile.durability),depth=info.qos_profile.depth,
        compatibility_code=int(compatibility),compatibility_reason=reason)


def recorder_endpoint_record(info,publishers,check):
    return dict(node_name=info.node_name,node_namespace=info.node_namespace,gid=bytes(info.endpoint_gid).hex(),
        endpoint_type=str(info.endpoint_type),
        qos=dict(reliability=str(info.qos_profile.reliability),durability=str(info.qos_profile.durability),
            history=str(info.qos_profile.history),depth=int(info.qos_profile.depth)),
        publisher_compatibility=[publisher_qos_report(publisher,info.qos_profile,check) for publisher in publishers])


def record_session(session,compression,cache_size_mib=16):
    session,metadata=load_session(session)
    status=check_session(session)
    if not status['record_allowed']: raise ValueError('raw recording refused: '+json.dumps(status['errors']))
    state=read_json(session/'processes.json')
    if any(process_matches(state.get(role,{})) for role in ['recorder','record_manager']): raise ValueError('session recorder already alive')
    if (session/'bag').exists(): raise ValueError('session bag exists; never overwrite or append')
    actual_record_command=record_command(session,compression,cache_size_mib)
    write_json(session/'record_settings.json',dict(command=actual_record_command,compression=compression,
        cache_size_bytes=cache_size_mib*1024*1024,cache_scope='rosbag2 double buffer may allocate twice max-cache-size',
        storage_preset_profile='resilient',sqlite_journal_mode='WAL',sqlite_synchronous='NORMAL',
        recorder_identity_strategy='exclusive_default_node_new_fixed_endpoint_gids',unique_node_name=False,
        topics=TOPICS,qos=qos_overrides(),source_provenance=provenance(metadata)),exclusive=True)
    # Initialize ROS before spawning anything: missing runtime dependencies leave no orphan recorder.
    rclpy,node=ros_context('mapping_capture_probe_'+metadata['uuid'],session)
    from sensor_msgs.msg import PointCloud2,Imu
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy,qos_check_compatible,QoSCompatibility
    raw_qos={topic:QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=qos_overrides()[topic]['depth'],
        reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE) for topic in RAW_TOPICS}
    accumulator=InputAccumulator(session)
    subscriptions=[]
    for topic,type_ in zip(RAW_TOPICS,[PointCloud2,Imu]):
        subscriptions.append(node.create_subscription(type_,topic,
            lambda message,topic=topic:observe_raw(accumulator,topic,message,time.time_ns()),raw_qos[topic]))
    manager=process_identity(os.getpid()); register_process(session,'record_manager',manager)
    child=None; identity=None; interrupted=False
    graph_evidence={}
    def request_interrupt(signum,frame):
        nonlocal interrupted
        interrupted=True
    previous={sig:signal.signal(sig,request_interrupt) for sig in [signal.SIGINT,signal.SIGTERM]}
    try:
        baseline_nodes=node.get_node_names_and_namespaces()
        baseline_endpoints={topic:[recorder_endpoint_record(info,node.get_publishers_info_by_topic(topic),qos_check_compatible)
            for info in node.get_subscriptions_info_by_topic(topic)] for topic in RAW_TOPICS}
        write_json(session/'recorder_baseline.json',dict(schema='mapping_recorder_baseline_v1',
            observed_unix_ns=time.time_ns(),nodes=baseline_nodes,endpoints=baseline_endpoints,
            recorder_node_name=DEFAULT_RECORDER_NODE,unique_node_name=False),exclusive=True)
        binding=RecorderEndpointBinding(baseline_nodes,baseline_endpoints)
        command=[sys.executable,str(Path(__file__).resolve()),'_bag-child',str(session),'--compression',compression,
            '--cache-size-mib',str(cache_size_mib)]
        child,identity=spawn_owned(session,'recorder',command)
        last_report=0; announced=False
        while child.poll() is None:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last_report>=.5:
                endpoint_inventory={}
                for topic in RAW_TOPICS:
                    endpoints=node.get_subscriptions_info_by_topic(topic)
                    publishers=node.get_publishers_info_by_topic(topic)
                    endpoint_inventory[topic]=[recorder_endpoint_record(info,publishers,qos_check_compatible) for info in endpoints]
                    accumulator.publisher_qos[topic]=[publisher_qos_report(info,raw_qos[topic],qos_check_compatible) for info in publishers]
                    for offered in accumulator.publisher_qos[topic]:
                        if offered['compatibility_code']==int(QoSCompatibility.ERROR):
                            observed=accumulator.topics.get(topic)
                            if observed is not None and 'publisher/probe QoS incompatible' not in observed['errors']:
                                observed['errors'].append('publisher/probe QoS incompatible')
                nodes=node.get_node_names_and_namespaces()
                bound=binding.update(nodes,endpoint_inventory)
                accumulator.subscriptions=bound['subscriptions']
                graph_evidence=dict(nodes=nodes,recorder_endpoints=endpoint_inventory,recorder_binding=bound,
                    recorder_node_name=DEFAULT_RECORDER_NODE,recorder_node_namespace='/',unique_node_name=False,
                    graph_observed_unix_ns=time.time_ns())
                if bound['bound'] and not (session/'recorder_binding.json').exists():
                    write_json(session/'recorder_binding.json',dict(bound,bound_unix_ns=time.time_ns(),
                        recorder_identity=identity,baseline_sha256=sha256(session/'recorder_baseline.json'),
                        endpoints=endpoint_inventory),exclusive=True)
                if bound['errors'] and not (session/'recorder_binding_failure.json').exists():
                    write_json(session/'recorder_binding_failure.json',dict(graph_evidence,observed_unix_ns=time.time_ns()),exclusive=True)
                for error in bound['errors']:
                    for observed in accumulator.topics.values():
                        if error not in observed['errors']: observed['errors'].append(error)
                accumulator.tf_static_publishers_seen |= bool(node.get_publishers_info_by_topic('/tf_static'))
                report=accumulator.report(); report['recorder_identity']=identity
                report.update(graph_evidence)
                write_json(session/'raw_input_report.json',report)
                if not announced and all(accumulator.subscriptions.values()) and all(accumulator.topics.get(t,{}).get('count',0)>=2 for t in RAW_TOPICS):
                    write_json(session/'recording_observed.json',dict(observed_unix_ns=time.time_ns(),recorder=identity,
                        binding=bound,note='Exclusive default recorder/new fixed endpoint GIDs discovered and raw messages observed. Not unique node name, cryptographic PID binding, or lossless proof.'))
                    print('RAW_RECORDING_OBSERVED: same-session subscriptions + both raw messages; start still requires strict contract.',flush=True)
                    announced=True
                last_report=time.monotonic()
            if interrupted:
                interrupted=False
                current=read_json(session/'processes.json')
                mapping_alive=any(process_matches(current.get(role,{})) for role in NODE_NAMES)
                if mapping_alive and not (session/'save_success.json').is_file():
                    print('REFUSING Ctrl-C while mapping unsaved: save successfully in another terminal, then stop; recorder continues.',file=sys.stderr,flush=True)
                elif process_matches(identity): signal_owned(identity,signal.SIGINT)
        child.wait()
        report=accumulator.report(); report['recorder_identity']=identity
        report.update(graph_evidence)
        report['recording_closed']=True; report['bag_exit_code']=child.returncode
        write_json(session/'raw_input_report.json',report)
        write_json(session/'record_closed.json',dict(exit_code=child.returncode,closed_unix_ns=time.time_ns(),
            metadata_present=(session/'bag/metadata.yaml').is_file(),flush_scope='SIGINT/normal rosbag exit; no fsync durability guarantee'))
        return child.returncode
    finally:
        if child is not None and child.poll() is None and identity and process_matches(identity):
            signal_owned(identity,signal.SIGINT)
            try: child.wait(timeout=60)
            except subprocess.TimeoutExpired: print('Recorder still flushing; no SIGKILL issued.',file=sys.stderr)
        for sig,handler in previous.items(): signal.signal(sig,handler)
        node.destroy_node(); rclpy.shutdown()


def fresh_bundle_target(session):
    session=Path(session).absolute()
    target=session/'bundles/map_bundle'
    for path in [target,*target.parents]:
        if path.is_symlink(): raise ValueError('save target/ancestor is symlink: '+str(path))
    if target.exists(): raise ValueError('map_bundle exists; writer could backup/overwrite it; new session required')
    if not target.parent.is_dir() or target.parent.resolve().parent!=session.resolve():
        raise ValueError('save target must be fresh strict child of this session/bundles')
    return target


@mapping_operation
def save_session(session,timeout):
    session,metadata=load_session(session)
    if not runtime_commit_report(session)['ready']: raise ValueError('no valid runtime INIT_COMMIT/frame metadata; save refused')
    state=read_json(session/'processes.json')
    if not all(process_matches(state.get(role,{})) for role in NODE_NAMES): raise ValueError('owned mapping nodes not all alive')
    if (session/'save_success.json').exists(): raise ValueError('session already saved; no overwrite')
    target=fresh_bundle_target(session)
    degradation=capture_degradation(session)
    rclpy,node=ros_context('mapping_capture_save_'+metadata['uuid'],session)
    from a2w_fastlio_msgs.srv import SaveMapBundle
    from a2w_fastlio_msgs.msg import RegistrationStatus
    latest={}; all_status=[]
    def receive(message):
        item=dict(stage=message.stage,reason=message.reason,accepted=bool(message.accepted))
        if message.stage=='backend': latest.update(item)
        all_status.append(item)
    subscription=node.create_subscription(RegistrationStatus,'/mapping/registration_status',receive,10)
    client=node.create_client(SaveMapBundle,'/mapping/save_map_bundle')
    response_data={}; end=time.monotonic()+timeout
    try:
        if not client.wait_for_service(timeout_sec=min(15,timeout)): raise ValueError('owned backend save service unavailable')
        fresh_bundle_target(session)
        request=SaveMapBundle.Request(); request.output_path='map_bundle'
        future=client.call_async(request)
        while rclpy.ok() and not future.done() and time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.1)
        if not future.done(): raise ValueError('save response timeout; backend may still be draining; do NOT stop')
        response=future.result()
        response_data=dict(success=bool(response.success),message=response.message,bundle_uuid=response.bundle_uuid,
            keyframe_count=int(response.keyframe_count),resolved_path=response.resolved_path)
        write_json(session/'save_response.json',response_data)
        if response.success:
            while time.monotonic()<end:
                try: validate_save_response(response_data,latest); break
                except ValueError: rclpy.spin_once(node,timeout_sec=.1)
        drained=validate_save_response(response_data,latest)
        if Path(response.resolved_path).resolve() != target.resolve(): raise ValueError('save response path escaped exact session map_bundle')
        checked=verify_bundle(session/'bundles/map_bundle',response.bundle_uuid,int(response.keyframe_count))
        degradation.extend(error for error in capture_degradation(session) if error not in degradation)
        result=dict(success=True,response=response_data,backend_status=dict(latest),bundle=checked,saved_unix_ns=time.time_ns(),
            drain=drained,**condition_labels(metadata),
            capture_degraded=bool(degradation),capture_degraded_reasons=degradation,
            source_provenance=provenance(metadata),**capture_evidence_labels(metadata.get('synthetic_test',False)))
        write_json(session/'save_success.json',result,exclusive=True)
        if metadata.get('experimental_initials_authorized') is True:
            write_json(session/'bundles/experimental_initial_conditions.json',dict(
                **condition_labels(metadata),authorization=read_json(session/POLICY_PATH),
                bundle_uuid=response_data['bundle_uuid'],interface_sha256=sha256(session/'interface.json')),exclusive=True)
        return result
    finally:
        write_json(session/'save_observed_status.json',dict(statuses=all_status,last_backend=latest,response=response_data))
        node.destroy_node(); rclpy.shutdown()


def read_bag_messages(bag):
    """Read serialized bag data without spinning a node or publishing anything."""
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if metadata.get('compression_format'):
        import rosbag2_py
        reader=rosbag2_py.SequentialCompressionReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        types={entry.name:get_message(entry.type) for entry in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic,serialized,record_ns=reader.read_next()
            yield topic,types[topic],deserialize_message(serialized,types[topic]),record_ns
    else:
        from contextlib import closing
        for path in sqlite_files(bag):
            with closing(sqlite3.connect('file:'+str(path.resolve())+'?mode=ro',uri=True)) as connection:
                topics={id_:(name,get_message(type_)) for id_,name,type_ in connection.execute('SELECT id,name,type FROM topics')}
                for id_,record_ns,serialized in connection.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
                    topic,type_=topics[id_]
                    yield topic,type_,deserialize_message(serialized,type_),record_ns


def recording_quality(closed,record_ranges,live,frontend_started_ns=None,recorder_log=None):
    errors=[]; coverage={}
    log_report=recorder_log_report(recorder_log) if recorder_log is not None else dict(checked=False)
    errors.extend(log_report.get('errors',[]))
    if closed.get('exit_code')!=0 or closed.get('metadata_present') is not True:
        errors.append('recorder did not report exit0 with flushed metadata')
    for topic in RAW_TOPICS:
        observed=live.get('topics',{}).get(topic,{})
        interval=record_ranges.get(topic)
        if not interval:
            errors.append(topic+' missing record time range'); continue
        first=interval['first_record_ns']; last=interval['last_record_ns']
        coverage[topic]=dict(interval,last_probe_receipt_ns=observed.get('last_receipt_unix_ns'),
            tail_gap_to_probe_ns=observed.get('last_receipt_unix_ns',last)-last,
            bag_minus_probe_count_proxy=interval.get('count',0)-observed.get('count',0))
        if frontend_started_ns is not None and first>frontend_started_ns:
            errors.append(topic+' bag begins after frontend start')
        if observed.get('last_receipt_unix_ns',last)-last>MAX_CLOCK_SKEW_NS:
            errors.append(topic+' bag tail is >2s behind last actual probe receipt')
    return dict(checks_passed=not errors,errors=errors,coverage=coverage,recorder_log=log_report,tolerance_ns=MAX_CLOCK_SKEW_NS,
        lossless_proven=False,note='Record/header gaps, ranges and probe count differences are coverage proxies only, not driver/DDS/storage loss proof.')


def verify_session(session):
    session,metadata=load_session(session)
    ensure_ros_environment(session)
    state=read_json(session/'processes.json')
    if any(process_matches(state.get(role,{})) for role in [*NODE_NAMES,'recorder','record_manager']):
        raise ValueError('verify requires all owned processes stopped and bag flushed')
    bag=session/'bag'
    if not (bag/'metadata.yaml').is_file(): raise ValueError('bag metadata missing (record not flushed)')
    inventory=sqlite_inventory(bag)
    observation=InputAccumulator(session); counts={}; headers={}; statuses=[]; record_ranges={}
    for topic,type_,message,record_ns in read_bag_messages(bag):
        counts[topic]=counts.get(topic,0)+1
        interval=record_ranges.setdefault(topic,dict(first_record_ns=record_ns,last_record_ns=record_ns,max_record_gap_ns=0,count=0))
        interval['max_record_gap_ns']=max(interval['max_record_gap_ns'],record_ns-interval['last_record_ns'])
        interval.update(first_record_ns=min(record_ns,interval['first_record_ns']),last_record_ns=record_ns,count=interval['count']+1)
        if topic in RAW_TOPICS: observe_raw(observation,topic,message,record_ns)
        for stamp in message_stamps(topic,message):
            item=headers.setdefault(topic,dict(first_stamp_ns=stamp,last_stamp_ns=stamp,count=0,max_gap_ns=0,nonincreasing=0))
            if item['count']:
                gap=stamp-item['last_stamp_ns']; item['max_gap_ns']=max(item['max_gap_ns'],gap)
                if gap<=0: item['nonincreasing']+=1
            item.update(last_stamp_ns=stamp,count=item['count']+1)
        if topic in ['/tf','/tf_static']:
            headers.setdefault(topic,{})['last_transforms']=[dict(parent=t.header.frame_id,child=t.child_frame_id,stamp_ns=stamp_ns(t.header)) for t in message.transforms]
        if topic=='/mapping/registration_status' and message.stage=='backend': statuses.append(dict(stage=message.stage,reason=message.reason))
    missing=[topic for topic in TOPICS if counts.get(topic,0)==0]
    if not inventory['storage_files']:
        inventory.update(sqlite_counts_available=False,missing_topics=missing,
            note='Compressed storage: counts obtained by sequential compression reader; direct sqlite inventory unavailable.')
    live=read_json(session/'raw_input_report.json') if (session/'raw_input_report.json').exists() else {}
    closed=read_json(session/'record_closed.json') if (session/'record_closed.json').exists() else {}
    started=read_json(session/'started.json') if (session/'started.json').exists() else {}
    quality=recording_quality(closed,record_ranges,live,started.get('started_unix_ns'),session/'logs/recorder.log')
    result=dict(schema='mapping_capture_verification_v1',session=str(session),topic_counts=counts,missing_topics=missing,
        **condition_labels(metadata),
        source_provenance=provenance(metadata),synthetic_test=metadata.get('synthetic_test',False),**capture_evidence_labels(metadata.get('synthetic_test',False)),
        storage=inventory,headers=headers,record_ranges=record_ranges,record_closed=closed,
        recording_quality=quality,raw_input=observation.report(),
        tf_static=dict(missing='/tf_static' in missing,publisher_ever_observed=live.get('tf_static_publishers_seen'),
            explanation='Explicitly missing; never fabricate transforms. Publisher-ever evidence is observational only.' if '/tf_static' in missing else 'Recorded static transforms present.'),
        runtime=runtime_shutdown(session/'runtime_state.csv',session/'logs/frontend.log') if (session/'runtime_state.csv').exists() else dict(valid=False,errors=['frontend runtime not created (raw-only session)']),
        last_backend_status=statuses[-1] if statuses else None,lossless_proven=False,map_gravity_alignment=False,
        time_scope='POST_LIDAR_UPDATE scan fields inherit frontend double-second round-trip, not original header-exact ns; INIT_COMMIT triple stamp is exact accepted IMU.',
        note='Counts, header gaps, shared internal geometry and received messages do not prove no upstream/DDS/storage loss or physical accuracy.')
    if (session/'save_success.json').exists():
        saved=read_json(session/'save_success.json')
        result['bundle']=verify_bundle(session/'bundles/map_bundle',saved['response']['bundle_uuid'],saved['response']['keyframe_count'])
        if saved.get('capture_degraded'): quality['errors'].append('save preserved map with degraded capture: '+str(saved.get('capture_degraded_reasons')))
    if started:
        if not result['runtime']['valid']: quality['errors'].extend(result['runtime']['errors'])
        if 'bundle' not in result: quality['errors'].append('mapping session lacks confirmed saved MapBundle')
        for topic in ['/Odometry','/tf','/mapping/registration_status']:
            if topic in missing: quality['errors'].append('mapping output missing from bag: '+topic)
        if not (session/'pre_frontend_bag_evidence.json').is_file(): quality['errors'].append('pre-frontend committed input evidence missing')
    if '/tf_static' in missing and live.get('tf_static_publishers_seen') is True:
        quality['errors'].append('tf_static publisher observed but no static message recorded')
    for topic,observed in observation.topics.items():
        quality['errors'].extend(topic+': '+error for error in observed.get('errors',[]))
    quality['checks_passed']=not quality['errors']
    result['capture_degraded']=not quality['checks_passed']
    write_json(session/'verification.json',result)
    return result


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest='command',required=True)
    template = commands.add_parser('interface-template'); template.add_argument('--output',type=Path)
    init = commands.add_parser('init'); init.add_argument('name'); init.add_argument('--interface',type=Path)
    init.add_argument('--sessions-root',type=Path,default=ROOT/'captures'); init.add_argument('--build-root',type=Path,default=build_root())
    init.add_argument('--synthetic-test',action='store_true',help='Explicit localhost-only nonzero-domain test; NEVER real hardware evidence')
    init.add_argument('--base-config',type=Path,help='Explicit existing JT128 calibrated config; contract SHA binds it and mapping R/T are preserved')
    for name in ['check','start','ready','save','stop','verify','record','shell','_bag-child']:
        command = commands.add_parser(name); command.add_argument('session',type=Path)
        if name in ['ready','save']: command.add_argument('--timeout',type=float,default=0 if name=='ready' else 600)
        if name=='stop': command.add_argument('--discard-unsaved',action='store_true'); command.add_argument('--timeout',type=float,default=60)
        if name in ['record','_bag-child']:
            command.add_argument('--compression',choices=['none','zstd'],default='none')
            command.add_argument('--cache-size-mib',type=int,default=16,help='Bounded 1..256 MiB; rosbag double buffer can use twice this')
        if name=='shell': command.add_argument('argv',nargs=argparse.REMAINDER)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command=='interface-template':
            result = interface_template()
            if args.output: write_json(args.output,result,exclusive=True)
        elif args.command=='init':
            result = dict(session=str(init_session(args.name,args.sessions_root,args.build_root,args.interface,
                base=args.base_config or DEFAULT_BASE,synthetic_test=args.synthetic_test)),map_gravity_alignment=False,
                synthetic_test=args.synthetic_test,**capture_evidence_labels(args.synthetic_test))
        elif args.command=='check':
            result = check_session(args.session)
            print(json.dumps(result,indent=2)); return 0 if result['start_allowed'] else 2
        elif args.command=='start': result = start_session(args.session)
        elif args.command=='ready':
            result = ready_session(args.session,args.timeout)
            print(json.dumps(result,indent=2)); return 0 if result['ready'] else 2
        elif args.command=='stop': result = stop_session(args.session,args.discard_unsaved,args.timeout)
        elif args.command=='_bag-child': return bag_child(args.session,args.compression,args.cache_size_mib)
        elif args.command=='record': return record_session(args.session,args.compression,args.cache_size_mib)
        elif args.command=='save': result = save_session(args.session,args.timeout)
        elif args.command=='verify':
            result = verify_session(args.session)
            print(json.dumps(result,indent=2,allow_nan=False)); return 2 if result['capture_degraded'] else 0
        elif args.command=='shell':
            session,metadata = load_session(args.session)
            command = args.argv[1:] if args.argv[:1]==['--'] else args.argv
            if not command: raise ValueError('shell requires -- COMMAND; no implicit interactive shell')
            return subprocess.run(command,env=session_env(session)).returncode
        print(json.dumps(result,indent=2,allow_nan=False)); return 0
    except (ValueError,OSError,KeyError,subprocess.SubprocessError,yaml.YAMLError) as error:
        print(json.dumps(dict(error=str(error),map_gravity_alignment=False)),file=sys.stderr); return 2


if __name__=='__main__': sys.exit(main())
