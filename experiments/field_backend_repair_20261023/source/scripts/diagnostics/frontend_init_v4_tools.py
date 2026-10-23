#!/usr/bin/env python3
"""Prepare/build isolated FAST-LIO; review LI-Init results. Never starts ROS."""
import argparse
import hashlib
import importlib.util
import io
import json
import math
import re
import shutil
from pathlib import Path
import subprocess
import sys
import tarfile
from datetime import datetime, timezone

ROOT=Path(__file__).resolve().parents[2]
BASE='16e97cdcc4260b9ba6241518a19ad8384224ba48'
PATCH=ROOT/'patches/fast_lio_v4/0001-frontend-init-quality-units.patch'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def extract_source_archive(data,copy):
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        members=archive.getmembers()
        for member in members:
            target=(copy/member.name).resolve()
            if copy not in target.parents and target!=copy:raise ValueError('unsafe archive path')
            if member.issym() or member.islnk():raise ValueError('unexpected source archive link')
        if hasattr(tarfile,'data_filter'):archive.extractall(copy,members=members,filter='data')
        else:archive.extractall(copy,members=members)

def prepare(source,output):
    source,output=Path(source).resolve(),Path(output).resolve()
    if output==source or source in output.parents or output in source.parents:
        raise ValueError('isolated output must not overlap original FAST-LIO source')
    if output.exists():raise ValueError('output exists; refusing overwrite')
    head=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    if head!=BASE:raise ValueError('unexpected base commit '+head)
    dirty=subprocess.check_output(['git','-C',str(source),'status','--porcelain'],text=True)
    if dirty:raise ValueError('source tree dirty; do not silently rebuild from untracked modifications')
    if not PATCH.is_file():raise ValueError('root repository tracked patch is missing')
    data=subprocess.check_output(['git','-C',str(source),'archive','--format=tar','HEAD'])
    output.mkdir(parents=True)
    copy=output/'source';copy.mkdir()
    extract_source_archive(data,copy)
    nested='include/ikd-Tree'
    tree_record=subprocess.check_output(['git','-C',str(source),'ls-tree','HEAD',nested],text=True).split()
    nested_commit=subprocess.check_output(['git','-C',str(source/nested),'rev-parse','HEAD'],text=True).strip()
    if len(tree_record)<3 or tree_record[1]!='commit' or tree_record[2]!=nested_commit:
        raise ValueError('nested ikd-Tree revision mismatch')
    if subprocess.check_output(['git','-C',str(source/nested),'status','--porcelain'],text=True):
        raise ValueError('nested ikd-Tree dirty')
    extract_source_archive(subprocess.check_output(['git','-C',str(source/nested),'archive','--format=tar','HEAD']),copy/nested)
    check=subprocess.run(['patch','--dry-run','--batch','--fuzz=0','-p1','-i',str(PATCH)],cwd=copy,capture_output=True,text=True)
    (output/'patch_dry_run.log').write_text(check.stdout+check.stderr)
    if check.returncode:raise ValueError('base anchored patch check failed; preserve output for review')
    applied=subprocess.run(['patch','--batch','--fuzz=0','-p1','-i',str(PATCH)],cwd=copy,capture_output=True,text=True)
    (output/'patch_apply.log').write_text(applied.stdout+applied.stderr)
    if applied.returncode:raise ValueError('patch application failed; preserve negative evidence')
    original={name:sha(source/name) for name in ['src/IMU_Processing.hpp','src/laserMapping.cpp','CMakeLists.txt']}
    manifest=dict(schema='frontend_init_v4_isolation_v1',created_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=head,nested_submodule_commits={nested:nested_commit},original_source=str(source),original_worktree_status=dirty,original_hashes=original,
        patch_path=str(PATCH.relative_to(ROOT)),patch_sha256=sha(PATCH),isolated_source=str(copy),
        patched_hashes={name:sha(copy/name) for name in [*original,'include/frontend_init_v4.hpp','include/frontend_init_state_v4.hpp','include/frontend_runtime_state_v4.hpp'] if (copy/name).is_file()},
        default_quality_gate_enabled=False,default_explicit_interface_enabled=False,
        ros_started=False,ros_published=False,production_config_written=False,main_install_changed=False)
    (output/'isolation_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def build(output,resume=False):
    output=Path(output).resolve();manifest=json.loads((output/'isolation_manifest.json').read_text())
    source=output/'source';build_dir=output/'build'
    if build_dir.exists() and not resume:raise ValueError('build directory exists; use new isolation or explicit --resume (preserves attempt logs)')
    if manifest['patch_sha256']!=sha(PATCH):raise ValueError('root patch changed since preparation; use new isolation')
    for name,digest in manifest['patched_hashes'].items():
        if sha(source/name)!=digest:raise ValueError('isolated patched source changed '+name)
    harness=ROOT/'scripts/diagnostics/frontend_init_v4_harness.cpp'
    if not harness.is_file():raise ValueError('actual Process harness missing')
    commands=[['cmake','-S',str(source),'-B',str(build_dir),'-DCMAKE_BUILD_TYPE=Release',
        '-DCMAKE_INSTALL_PREFIX='+str(output/'install'),'-DFRONTEND_INIT_V4_BUILD_HARNESS=ON',
        '-DFRONTEND_INIT_V4_HARNESS_SOURCE='+str(harness),'-DPYTHON_EXECUTABLE=/usr/bin/python3','-DPython3_EXECUTABLE=/usr/bin/python3'],
        ['cmake','--build',str(build_dir),'--target','fastlio_mapping','frontend_init_v4_harness','--parallel','2']]
    logs=[]
    attempt=1
    while (output/f'configure_attempt_{attempt}.log').exists():attempt+=1
    for index,command in enumerate(commands):
        log=output/(('configure' if index==0 else 'compile')+f'_attempt_{attempt}.log')
        with log.open('x') as stream:
            result=subprocess.run(['bash','--noprofile','--norc','-c','source "$1"; shift; exec "$@"','frontend_v4',
                '/opt/ros/humble/setup.bash',*command],stdout=stream,stderr=subprocess.STDOUT)
        logs.append(dict(command=command,exit_code=result.returncode,log=str(log)))
        if result.returncode:
            (output/f'build_failure_attempt_{attempt}.json').write_text(json.dumps(logs,indent=2)+'\n')
            raise ValueError('isolated build failed; inspect '+str(log))
    for name,digest in manifest['original_hashes'].items():
        if sha(Path(manifest['original_source'])/name)!=digest:raise ValueError('original source unexpectedly changed')
    result=dict(schema='frontend_init_v4_build_v1',base_commit=BASE,patch_sha256=sha(PATCH),
        harness_source_sha256=sha(harness),commands=logs,ros_started=False,ros_published=False,
        binaries={name:dict(path=str(build_dir/name),sha256=sha(build_dir/name)) for name in ['fastlio_mapping','frontend_init_v4_harness']})
    (output/'build_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def review_external(data):
    """Structural checks plus explicitly attested evidence, NOT calibration verification."""
    if data.get('source_provenance') not in ['synthetic','real']:
        raise ValueError('explicit synthetic/real result provenance required')
    if data.get('verified') is not True:
        raise ValueError('unverified result cannot export even an experimental configuration')
    for key in ['unit_evidence','frame_evidence','temporal_stability_evidence','excitation_evidence']:
        if not data.get(key) or data[key]=='unknown':raise ValueError('missing review evidence '+key)
    if data.get('imu_gyro_unit') not in ['rad/s','deg/s'] or data.get('imu_accel_unit') not in ['m/s^2','g']:
        raise ValueError('explicit actual source units required; do not infer from acceleration norm')
    if data.get('offset_definition')!='t_imu_raw_minus_t_lidar_same_event':
        raise ValueError('unknown or opposite offset sign; explicit same-event convention required')
    try:
        r=[[float(v) for v in row] for row in data['R_I_L']];t=[float(v) for v in data['t_I_L']];offset=float(data['offset_s'])
    except (KeyError,TypeError,ValueError) as error:raise ValueError('invalid T_I_L/offset') from error
    if len(r)!=3 or any(len(row)!=3 for row in r) or len(t)!=3 or not all(math.isfinite(v) for row in r for v in row) or not all(math.isfinite(v) for v in [*t,offset]):
        raise ValueError('finite3x3 proper rotation,translation3 and offset required')
    orth=max(abs(sum(r[k][i]*r[k][j] for k in range(3))-(i==j)) for i in range(3) for j in range(3))
    det=(r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]))
    if orth>1e-6 or abs(det-1)>1e-6:raise ValueError('R_I_L is not a proper rotation; do not silently project/scale')
    return dict(schema='li_init_review_v1',review_status='structural_checks_and_caller_attestation_only',
        production_accepted=False,physical_transform='T_I_L',R_I_L=r,t_I_L=t,time_offset_lidar_to_imu=offset,
        time_application='aligned_imu_stamp=raw_imu_stamp-offset; exactly once',source_provenance=data['source_provenance'],
        imu_gyro_unit=data['imu_gyro_unit'],imu_accel_unit=data['imu_accel_unit'],
        evidence={k:data[k] for k in ['unit_evidence','frame_evidence','temporal_stability_evidence','excitation_evidence']},
        warnings=['No actual upstream calibration execution or device verification is certified by this parser.',
            'Do not import LI-Init world gravity/IMU bias as an arbitrary FAST-LIO world alignment.',
            'Printed upstream6-digit matrices may fail strict proper-rotation validation; retain negative result, obtain high-precision reviewed calibration.',
            'Time offset may change with power-on clock origins; static mounting alone does not certify temporal stability.'])


def export_external(result_path,output):
    result_path,output=Path(result_path).resolve(),Path(output).resolve()
    if output.exists() or output in result_path.parents or result_path in output.parents:
        raise ValueError('new separate external-review output required')
    output.mkdir(parents=True)
    data=json.loads(result_path.read_text())
    audit=dict(input_path=str(result_path),input_sha256=sha(result_path),production_changed=False,upstream_executed_by_this_tool=False)
    try:review=review_external(data)
    except ValueError as error:
        (output/'rejected.json').write_text(json.dumps({**audit,'status':'REJECTED','reason':str(error)},indent=2)+'\n')
        raise
    review.update(audit)
    (output/'review.json').write_text(json.dumps(review,indent=2)+'\n')
    parameters={'common.imu_gyr_unit':'rad' if review['imu_gyro_unit']=='rad/s' else 'deg',
        'common.time_sync_en':False,'common.time_offset_lidar_to_imu':review['time_offset_lidar_to_imu'],
        'mapping.extrinsic_R':[v for row in review['R_I_L'] for v in row],'mapping.extrinsic_T':review['t_I_L'],
        'frontend_init_v4.input_gyro_unit':review['imu_gyro_unit'],'frontend_init_v4.input_accel_unit':review['imu_accel_unit'],
        'frontend_init_v4.input_interface_enabled':False,'frontend_init_v4.quality_gate_enabled':False}
    (output/'EXPERIMENT_ONLY_NOT_PRODUCTION_ACCEPTED.yaml').write_text(
        '# Human review required. NEVER automatically sourced/loaded by this tool. Both new features remain default-off.\n'+
        json.dumps({'/**':{'ros__parameters':parameters}},indent=2)+'\n')
    return review

def parse_official_text(text):
    """Official result file import, last completed Refinement only; no verification claim."""
    if 'Refinement result:' not in text:raise ValueError('completed Refinement result missing; initial-only result not accepted')
    block=text.rsplit('Refinement result:',1)[1]
    offsets=re.findall(r'Time Lag IMU to LiDAR \(second\)\s*=\s*([-+0-9.eE]+)',block)
    label='Homogeneous Transformation Matrix from LiDAR to IMU:'
    if len(offsets)!=1 or block.count(label)!=1:raise ValueError('ambiguous or incomplete refined time/matrix result')
    lines=[line.strip() for line in block.split(label,1)[1].splitlines() if line.strip()]
    try:matrix=[[float(x) for x in line.split()] for line in lines[:4]]
    except ValueError as error:raise ValueError('invalid official matrix') from error
    if len(matrix)!=4 or any(len(row)!=4 for row in matrix) or any(not math.isfinite(x) for row in matrix for x in row) or matrix[3]!=[0.,0.,0.,1.]:
        raise ValueError('invalid homogeneous4x4 matrix; do not guess Euler convention')
    return dict(schema='li_init_import_UNVERIFIED_v1',source_provenance='unknown',verified=False,
        R_I_L=[row[:3] for row in matrix[:3]],t_I_L=[row[3] for row in matrix[:3]],offset_s=float(offsets[0]),
        offset_definition='t_imu_raw_minus_t_lidar_same_event',imu_gyro_unit='unknown',imu_accel_unit='unknown',
        unit_evidence='',frame_evidence='',temporal_stability_evidence='',excitation_evidence='',
        warning='Imported official final refined matrix/time only; supply reviewed provenance/evidence independently, never infer device units')

def external_plan(output):
    output=Path(output).resolve()
    if output.exists():raise ValueError('new external plan output required')
    output.mkdir(parents=True)
    result=dict(schema='li_init_independent_entry_plan_v1',upstream='https://github.com/hku-mars/LiDAR_IMU_Init',
        upstream_revision='UNPINNED: record exact git rev-parse HEAD before any future execution',
        official_sources=['https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/README.md',
            'https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/src/laserMapping.cpp'],
        upstream_run_status='NOT_RUN',production_accepted=False,ros_started=False,ros_published=False,
        local_prerequisites={name:shutil.which(name) for name in ['roscore','catkin_make']},
        unavailable_real_data='1016 Bundle XYZI is not raw timed LiDAR/IMU; no new real capture authorized',
        manual_isolated_sequence=['Obtain/pin official source in separate ROS1 workspace/container (never main install).',
            'Verify LiDAR PointCloud2 field/types/time unit compatibility; supply raw timed LiDAR and IMU with rad/s gyro and explicitly evidenced acceleration unit.',
            'After separately authorized replay, retain full excitation/refinement logs, input hashes, clock/reset identities and result/Initialization_result.txt.',
            'Use external-import; independently review exact T_I_L, source units, excitation, repeatability and total fixed offset sign.',
            'Use external-review to write experimental-only config; never auto-load or accept into production.'],
        acceptance_checks=['No silent ROS1/ROS2 point-layout assumption','Sufficient multi-axis rotational/translational excitation',
            'Completed final Refinement result, not first initialization print','Independent repeatability and residual/held-out checks',
            'Physical mounting/frame check','Same-event clock sign and power-cycle temporal stability'],
        limitations=['Static gate cannot estimate full extrinsic/time calibration','No upstream algorithm or real device result verified here'])
    (output/'entry_plan.json').write_text(json.dumps(result,indent=2)+'\n');return result

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    apply=sub.add_parser('prepare');apply.add_argument('output',type=Path);apply.add_argument('--source',type=Path,default=ROOT/'src/FAST_LIO_Hesai')
    compile_=sub.add_parser('build');compile_.add_argument('output',type=Path);compile_.add_argument('--resume',action='store_true')
    review=sub.add_parser('external-review');review.add_argument('result_json',type=Path);review.add_argument('output',type=Path)
    plan=sub.add_parser('external-plan');plan.add_argument('output',type=Path)
    import_=sub.add_parser('external-import');import_.add_argument('official_result',type=Path);import_.add_argument('output',type=Path)
    args=parser.parse_args(argv)
    try:
        if args.command=='prepare':result=prepare(args.source,args.output)
        elif args.command=='build':result=build(args.output,args.resume)
        elif args.command=='external-plan':result=external_plan(args.output)
        elif args.command=='external-import':
            if args.output.exists():raise ValueError('output exists; refusing overwrite')
            result=parse_official_text(args.official_result.read_text());result['input_sha256']=sha(args.official_result)
            args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
        else:result=export_external(args.result_json,args.output)
        print(json.dumps(result,indent=2));return 0
    except (OSError,ValueError,subprocess.SubprocessError) as error:
        print('frontend_init_v4: '+str(error),file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
