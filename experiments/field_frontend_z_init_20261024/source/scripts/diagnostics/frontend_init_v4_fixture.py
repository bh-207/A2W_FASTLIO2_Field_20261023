#!/usr/bin/env python3
"""Export frozen synthetic raw IMU + XYZI/ring/absolute-time points; no ROS."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
DEFAULT_SOURCE=ROOT/'artifacts/terrain_extrinsic_1016_v1/inputs_v2/flat'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def export(source,output,scans=30):
    source,output=Path(source).resolve(),Path(output).resolve()
    if output.exists() or source in output.parents or output in source.parents:raise ValueError('new separate output required')
    if not 1<=scans<=30:raise ValueError('lightweight startup prefix only: scans1..30')
    identity=json.loads((source/'identity.json').read_text())
    if identity['scope']!='INDEPENDENT_SYNTHETIC_NOT_REAL_CAPTURE_REPLAY':raise ValueError('unexpected provenance')
    files=['imu_clock_truth.npz']+[f'scan_{i:04d}.npz' for i in range(scans)]
    hashes={name:sha(source/name) for name in files}
    for name,digest in hashes.items():
        if identity['files'].get(name)!=digest:raise ValueError('frozen sensor hash mismatch '+name)
    output.mkdir(parents=True)
    with np.load(source/'imu_clock_truth.npz',allow_pickle=False) as raw:
        # Never access p/R (independent truth); select unchanged raw sensor arrays.
        t,a,g=raw['t'],raw['accel'],raw['gyro']
        with (output/'imu.csv').open('x') as f:
            writer=csv.writer(f);writer.writerow(['stamp_s','ax','ay','az','gx','gy','gz'])
            for i in np.flatnonzero(t<=identity['epoch']+scans/10+.01):writer.writerow([format(float(x),'.17g') for x in [t[i],*a[i],*g[i]]])
    for i in range(scans):
        with np.load(source/f'scan_{i:04d}.npz',allow_pickle=False) as raw:
            cloud=raw['cloud']
            with (output/f'scan_{i:04d}.csv').open('x') as f:
                writer=csv.writer(f);writer.writerow(['x','y','z','intensity','ring','timestamp'])
                for p in cloud:writer.writerow([format(float(p[k]),'.17g') for k in ['x','y','z','intensity','ring','timestamp']])
    (output/'extrinsic.txt').write_text(' '.join(format(v,'.17g') for row in identity['R_IL'] for v in row)+' '+
        ' '.join(format(v,'.17g') for v in identity['T_IL'])+'\n')
    report=dict(schema='frontend_init_v4_frozen_raw_prefix_v1',source_provenance='synthetic',
        scope='Frozen terrain synthetic raw sensor arrays; NOT1016 raw recording, NOT164 driver evidence',
        source=str(source),identity_sha256=sha(source/'identity.json'),source_sha256=hashes,
        exported_sha256={p.name:sha(p) for p in sorted(output.iterdir()) if p.is_file()},
        truth_arrays_loaded=False,numeric_rescaling_applied=False,scans=scans,
        frames={'W':'camera_init(estimator),generator_world distinct','I':'body; input frame imu','L':'LiDAR'},
        units={'accel':'m/s^2','gyro':'rad/s','point_timestamp':'absolute seconds'},
        unit_evidence='terrain_model.py state_at: R^T(a-[0,0,-9.81])+BA, SO(3) angular derivative; freeze_inputs.py saves unchanged',
        time_evidence='same frozen generator epoch; declared offset0; double seconds converted to ns with llround, at most0.5ns rounding',
        production_changed=False,ros_started=False,ros_published=False)
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n');return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);p.add_argument('--source',type=Path,default=DEFAULT_SOURCE);p.add_argument('--scans',type=int,default=30);args=p.parse_args()
    print(json.dumps(export(args.source,args.output,args.scans),indent=2))
