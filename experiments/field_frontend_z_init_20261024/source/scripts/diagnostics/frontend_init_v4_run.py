#!/usr/bin/env python3
"""Durable no-ROS actual Process/ESKF/Preprocess mechanism validation."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

CASES=['tilted_static','moving_then_static','moving_never_static','late_bad_group','late_bad_input_nan','late_bad_input_stamp','positive_backtrack_buffered','off']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(isolation,fixture,output):
    isolation,fixture,output=map(lambda p:Path(p).resolve(),[isolation,fixture,output])
    if output.exists() or isolation in output.parents or output in isolation.parents or fixture in output.parents or output in fixture.parents:
        raise ValueError('new separate output required; no source/input overwrite')
    binary=isolation/'build/frontend_init_v4_harness'
    manifest=json.loads((isolation/'build_manifest.json').read_text())
    if sha(binary)!=manifest['binaries']['frontend_init_v4_harness']['sha256']:raise ValueError('binary/build provenance mismatch')
    fixture_meta=json.loads((fixture/'manifest.json').read_text())
    if fixture_meta['source_provenance']!='synthetic':raise ValueError('unexpected fixture provenance')
    if fixture_meta.get('scans')!=30:raise ValueError('mechanism frozen case requires30 scans; shorter exports are inspection-only')
    if not fixture_meta.get('exported_sha256'):raise ValueError('exported input integrity inventory missing')
    for name,digest in fixture_meta['exported_sha256'].items():
        path=(fixture/name).resolve()
        if path.parent!=fixture or sha(path)!=digest:raise ValueError('exported input integrity mismatch '+name)
    output.mkdir(parents=True)
    reports=[]
    for case in [*CASES,'frozen_terrain']:
        command=[str(binary),case,str(output/case)]+([str(fixture)] if case=='frozen_terrain' else [])
        with (output/(case+'.log')).open('x') as log:completed=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        report=dict(case=case,exit_code=completed.returncode,command=command)
        path=output/case/'result.json'
        if path.exists():report['result']=json.loads(path.read_text());report['result_sha256']=sha(path)
        reports.append(report)
        # Preserve all cases, including negative mechanism expectations and unexpected failures.
    result=dict(schema='frontend_init_v4_mechanism_suite_v1',source_provenance='synthetic',
        ros_started=False,ros_published=False,production_changed=False,source_base_commit=manifest['base_commit'],
        patch_sha256=manifest['patch_sha256'],binary_sha256=sha(binary),fixture_manifest_sha256=sha(fixture/'manifest.json'),
        overall_pass=all(r['exit_code']==0 and r.get('result',{}).get('mechanism_checks_passed') for r in reports),cases=reports,
        scope='Actual patched ImuProcess::Process, ESKF prediction and timed-point preprocess/deskew. No laser measurement/map/registration update, no ROS transport, no real-device proof.')
    (output/'suite.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('isolation',type=Path);p.add_argument('fixture',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    report=run(a.isolation,a.fixture,a.output);print(json.dumps(report,indent=2));raise SystemExit(0 if report['overall_pass'] else 2)
