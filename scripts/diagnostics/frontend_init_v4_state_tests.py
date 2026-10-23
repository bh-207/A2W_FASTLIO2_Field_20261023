#!/usr/bin/env python3
"""Build/run real Process state-contract RED or GREEN in a new isolated output; no ROS."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]
NEGATIVE=['reused_rotated_state','reused_yaw_state','preserved_ba_residual','last_accel_residual',
 'nonfinite_position','nonfinite_ba','invalid_state_quaternion','nonfinite_ext_translation',
 'nonproper_ext_scale','nonproper_ext_reflection','nonfinite_covariance','asymmetric_covariance',
 'negative_covariance','singular_covariance','nonfinite_acc_noise','negative_gyro_noise',
 'negative_bias_noise','negative_gyro_bias_noise']
OUTPUT_RED=['stale_output_waiting','stale_output_rejected','stale_output_commit_scan']
POSITIVE=['fresh_unchanged','off_unchanged','off_reused_rotated_legacy','off_sentinel_legacy']
CONFIGURED=['configured_mean_tolerance','configured_last_tolerance']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(isolation,output,phase):
    isolation,output=Path(isolation).resolve(),Path(output).resolve();source=isolation/'source'
    if output.exists() or source in output.parents or output in source.parents:raise ValueError('new nonoverlapping output required')
    manifest=json.loads((isolation/'isolation_manifest.json').read_text())
    for name,digest in manifest['patched_hashes'].items():
        if sha(source/name)!=digest:raise ValueError('isolated source identity mismatch '+name)
    output.mkdir(parents=True)
    test=ROOT/'patches/fast_lio_v4/test_frontend_init_v4_state_consistency.cpp'
    shutil.copyfile(test,output/'test_source_snapshot.cpp')
    identity=dict(phase=phase,source_base_commit=manifest['base_commit'],patch_sha256=manifest['patch_sha256'],
        source_hashes={name:sha(source/name) for name in ['src/IMU_Processing.hpp','include/frontend_init_v4.hpp','CMakeLists.txt']},
        test_source_sha256=sha(test),input_provenance='synthetic literal200Hz static state fixtures in hashed test source',
        ros_started=False,ros_published=False)
    state_header=source/'include/frontend_init_state_v4.hpp'
    if state_header.exists():identity['source_hashes']['include/frontend_init_state_v4.hpp']=sha(state_header)
    frozen_patch=ROOT/'patches/fast_lio_v4/0001-frontend-init-quality-units.patch'
    if sha(frozen_patch)==manifest['patch_sha256']:shutil.copyfile(frozen_patch,output/'input_frozen_patch.patch')
    (output/'identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    commands=[['cmake','-S',str(source),'-B',str(output/'build'),'-DCMAKE_BUILD_TYPE=Release',
      '-DCMAKE_INSTALL_PREFIX='+str(output/'unused_install'),'-DFRONTEND_INIT_V4_BUILD_HARNESS=ON',
      '-DFRONTEND_INIT_V4_HARNESS_SOURCE='+str(test),'-DPython3_EXECUTABLE=/usr/bin/python3','-DPYTHON_EXECUTABLE=/usr/bin/python3'],
      ['cmake','--build',str(output/'build'),'--target','frontend_init_v4_harness','--parallel','2']]
    for command,name in zip(commands,['configure','compile']):
        with (output/(name+'.log')).open('x') as log:
            status=subprocess.run(['bash','--noprofile','--norc','-c','source "$1"; shift; exec "$@"','state_tests',
                '/opt/ros/humble/setup.bash',*command],stdout=log,stderr=subprocess.STDOUT).returncode
        if status:raise RuntimeError(name+' failed; retain log')
    binary=output/'build/frontend_init_v4_harness';identity['binary_sha256']=sha(binary)
    rows=[]
    for case in [*POSITIVE,*NEGATIVE,*OUTPUT_RED,*CONFIGURED]:
        execution=subprocess.run([str(binary),case,str(output/case)],capture_output=True,text=True)
        (output/(case+'.log')).write_text(execution.stdout+execution.stderr)
        quality=output/case/'quality.csv';events=list(csv.DictReader(quality.open())) if quality.exists() else []
        initialized=any(r['stage']=='initialized' for r in events)
        expected_red=phase=='red' and case in [*NEGATIVE,*OUTPUT_RED]
        if expected_red:
            matched=execution.returncode==1 and (('STATE GUARD MISSING' in execution.stderr and initialized) or
                (case in OUTPUT_RED and 'OUTPUT GUARD MISSING' in execution.stderr))
        else:matched=execution.returncode==0
        checks=output/case/'quality.csv.state_checks.csv'
        reasons=[]
        if checks.exists():reasons=sorted({r['reason'] for r in csv.DictReader(checks.open()) if r['accepted']=='0'})
        if phase=='green' and case in ['preserved_ba_residual','stale_output_rejected']:
            matched=matched and 'state_mean_residual_above_threshold' in reasons
        if phase=='green' and case=='last_accel_residual':matched=matched and 'state_last_residual_above_threshold' in reasons
        if phase=='green' and case in ['reused_rotated_state','reused_yaw_state']:
            matched=matched and 'state_requires_fresh_identity_rotation_restart' in reasons
        state_meta=output/case/'quality.csv.state_checks.metadata.json'
        if phase=='green' and case in CONFIGURED:
            values=json.loads(state_meta.read_text())
            key='maximum_mean_accel_residual_m_s2' if case=='configured_mean_tolerance' else 'maximum_last_accel_residual_m_s2'
            matched=matched and values[key]==(2. if case=='configured_mean_tolerance' else 1.)
        rows.append(dict(case=case,exit_code=execution.returncode,expected_red=expected_red,expected_observation_matched=matched,
            initialized_trace=initialized,state_rejection_reasons=reasons))
    result={**identity,'build_commands':commands,'all_expected_observations':all(r['expected_observation_matched'] for r in rows),'cases':rows}
    (output/'suite.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('isolation',type=Path);p.add_argument('output',type=Path);p.add_argument('phase',choices=['red','green']);a=p.parse_args()
    report=run(a.isolation,a.output,a.phase);print(json.dumps(report,indent=2));raise SystemExit(0 if report['all_expected_observations'] else 2)
