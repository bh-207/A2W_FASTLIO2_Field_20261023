#!/usr/bin/env python3
"""Archive real no-ROS bounded writer tests; refuses existing output."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[2]
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def run(output):
    output=Path(output).resolve()
    if output.exists():raise ValueError('new runtime writer evidence output required')
    output.mkdir(parents=True)
    header=ROOT/'patches/fast_lio_v4/frontend_runtime_state_v4.hpp'
    test=ROOT/'patches/fast_lio_v4/test_frontend_runtime_state_v4.cpp'
    binary=output/'writer_test'
    command=['g++','-std=c++17','-O2','-pthread','-I',str(header.parent),str(test),'-o',str(binary)]
    build=subprocess.run(command,capture_output=True,text=True)
    (output/'compile.log').write_text(build.stdout+build.stderr)
    identity=dict(schema='frontend_runtime_state_v4_writer_tests_v1',source_provenance='synthetic',ros_started=False,
        source_header_sha256=sha(header),test_source_sha256=sha(test),root_patch_sha256=sha(ROOT/'patches/fast_lio_v4/0001-frontend-init-quality-units.patch'),
        build_command=command,build_exit_code=build.returncode,cases=[])
    (output/'source_header_snapshot.hpp').write_bytes(header.read_bytes())
    (output/'test_source_snapshot.cpp').write_bytes(test.read_bytes())
    if build.returncode:
        (output/'result.json').write_text(json.dumps(identity,indent=2)+'\n');return identity
    identity['binary_sha256']=sha(binary)
    for case in ['disabled','golden','overflow','write_error','live_init']:
        folder=output/case;folder.mkdir();path=folder/'state.csv'
        execution=subprocess.run([str(binary),case,str(path)],capture_output=True,text=True)
        (folder/'execution.log').write_text(execution.stdout+execution.stderr)
        item=dict(case=case,exit_code=execution.returncode,pass_marker='PASS '+case in execution.stdout)
        status=Path(str(path)+'.status.json')
        if status.is_file():item['actual_status']=json.loads(status.read_text())
        identity['cases'].append(item)
    identity['overall_pass']=all(item['exit_code']==0 and item['pass_marker'] for item in identity['cases'])
    (output/'result.json').write_text(json.dumps(identity,indent=2)+'\n');return identity

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path);args=parser.parse_args()
    result=run(args.output);print(json.dumps(result,indent=2));raise SystemExit(0 if result.get('overall_pass') else 1)
