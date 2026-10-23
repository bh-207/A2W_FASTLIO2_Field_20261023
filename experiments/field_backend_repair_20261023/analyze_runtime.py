#!/usr/bin/env python3
"""Closed-run replay pacing, process resource and IMU-trigger-to-POST evidence."""
import argparse
from bisect import bisect_left,bisect_right
from collections import defaultdict
import csv
import json
import os
from pathlib import Path
import numpy as np

def stats(values):
    a=np.asarray(values,dtype=float)
    return dict(count=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),maximum=float(a.max()),minimum=float(a.min())) if len(a) else dict(count=0)

def run(session,rate=1):
    session=Path(session)
    with (session/'published_raw.csv').open() as stream: pubs=list(csv.DictReader(stream))
    first=pubs[0];source0=int(first['bag_record_ns']);steady0=int(first['publish_steady_ns'])
    pacing=[(int(r['publish_steady_ns'])-steady0-(int(r['bag_record_ns'])-source0)/rate)/1e6 for r in pubs]
    imu=defaultdict(list)
    for r in pubs:
        if r['topic']=='/lidar_imu':imu[int(r['header_ns'])].append(int(r['publish_steady_ns']))
    with (session/'frontend_handoff.csv').open() as stream:
        post={int(r['stamp_ns']):int(r['steady_ns']) for r in csv.DictReader(stream) if r['stage']=='post_lidar_update'}
    latencies=[];missing=0;cloud_latencies=[];cloud_missing=0
    clouds=defaultdict(list)
    for r in pubs:
        if r['topic']=='/lidar_points':clouds[int(r['header_ns'])].append(int(r['publish_steady_ns']))
    cloud_stamps=sorted(clouds)
    with (session/'runtime_state.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['stage']!='POST_LIDAR_UPDATE':continue
            t=int(row['state_stamp_ns']);source=int(row['group_last_imu_ns'])
            candidates=[s for s in imu.get(source,[]) if t in post and s<=post[t]]
            if len(candidates)==1:latencies.append((post[t]-candidates[0])/1e6)
            else:missing+=1
            begin=int(row['scan_begin_ns']);matches=cloud_stamps[bisect_left(cloud_stamps,begin-1000):bisect_right(cloud_stamps,begin+1000)]
            if len(matches)==1 and len(clouds[matches[0]])==1 and t in post:
                cloud_latencies.append((post[t]-clouds[matches[0]][0])/1e6)
            else:cloud_missing+=1
    resources=[json.loads(line) for line in (session/'process_resources.jsonl').read_text().splitlines()]
    roles={};hz=os.sysconf('SC_CLK_TCK')
    for role in ['frontend','ingress','backend','recorder','field_observer']:
        records=[(r['steady_ns'],r['roles'][role]) for r in resources if role in r['roles'] and 'cpu_ticks' in r['roles'][role]]
        utilization=[(b['cpu_ticks']-a['cpu_ticks'])/hz/((tb-ta)/1e9)*100 for (ta,a),(tb,b) in zip(records,records[1:]) if tb>ta]
        roles[role]=dict(cpu_percent_one_core=stats(utilization),rss_MiB=stats([r['rss_bytes']/2**20 for _,r in records]))
    return dict(schema='closed_offline_runtime_evidence_v1',rate=rate,publisher_pacing_lag_ms=stats(pacing),
        last_group_IMU_publish_to_POST_ms=stats(latencies),post_latency_unmatched=missing,process_resources=roles,
        cloud_publish_return_to_POST_ms=stats(cloud_latencies),cloud_latency_unmatched=cloud_missing,cloud_begin_association_tolerance_ns=1000,
        scopes=['Latency starts at the publication of the last grouped IMU, not pure algorithm service time.',
                'Pacing includes Python Bag decode/publish and host scheduling; source stamps were preserved.',
                'CPU is percent of one logical core, from own /proc stat deltas; other desktop load is uncontrolled.',
                'Cloud latency includes input waiting and scheduling/estimation; it is not pure algorithm service time.',
                'Original-rate input completion, queue drain and log closure are checked separately.'])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    with a.output.open('x') as stream:json.dump(run(a.session),stream,indent=2);stream.write('\n')
