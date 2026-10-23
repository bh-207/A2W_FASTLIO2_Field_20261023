#!/usr/bin/env python3
"""Formal closed-run gate; helper result.valid alone is never input acceptance."""
import argparse
from collections import Counter
import csv
import json
import math
import sys
from pathlib import Path
from audit_inputs import run as audit,headers,compare
sys.path.insert(0,str(Path(__file__).resolve().parent/'source/scripts'))
from timestamp_accounting import account

def full_output_handoff(session):
    """Audit also the non-keyframe tail beyond the frozen Bundle cutoff."""
    session=Path(session);events={}
    for role in ['frontend','ingress']:
        with (session/(role+'_handoff.csv')).open() as stream:
            for row in csv.DictReader(stream):events.setdefault(row['stage'],[]).append(int(row['stamp_ns']))
    with (session/'runtime_state.csv').open() as stream:
        runtime=[int(r['state_stamp_ns']) for r in csv.DictReader(stream) if r['stage']=='POST_LIDAR_UPDATE']
    checks={'post_lidar_update':account(runtime,events.get('post_lidar_update',[]),0),
            'frontend_odom_publish_return':account(runtime,events.get('frontend_odom_publish_return',[]),1000)}
    published=events.get('frontend_odom_publish_return',[])
    for name in ['frontend_cloud_publish_return','ingress_odom_received','ingress_cloud_received','ingress_synchronized','keyframe_decision']:
        checks[name]=account(published,events.get(name,[]),0)
    return dict(valid=bool(runtime) and all(c['complete'] for c in checks.values()),stages=checks,
        scope='All closed runtime POSTs including tail beyond last saved keyframe; Bundle keyframe subset checked separately.')

def judge(result,accounting,cloud_recording,full_source=None):
    checks=dict(
        original_rate=isinstance(result.get('rate'),(int,float)) and math.isfinite(result['rate']) and result['rate']==1.,
        duration_scope_valid=isinstance(result.get('requested_duration_s'),(int,float)) and math.isfinite(result['requested_duration_s']) and result['requested_duration_s']>=0,
        no_runtime_exception=not result.get('error') and not result.get('cleanup_error'),
        all_actual_process_exits_zero=set(result.get('exit_codes',{}))=={'frontend','ingress','backend','recorder','field_observer'} and all(x==0 for x in result['exit_codes'].values()),
        ordered_stop_closed=result.get('ordered_stop',{}).get('success') is True,
        fixed_extrinsic_state=result.get('fixed_state_closed',{}).get('valid') is True,
        runtime_log_complete=result.get('runtime_shutdown',{}).get('valid') is True,
        handoff_complete=result.get('handoff',{}).get('valid') is True,
        all_post_output_handoff_exact=result.get('full_output_handoff',{}).get('valid') is True,
        recorder_no_reported_cache_loss=result.get('recorder_log',{}).get('checked') is True and not result['recorder_log'].get('loss_reported') and not result['recorder_log'].get('errors'),
        published_to_imu_callback_exact=accounting.get('published_vs_callback_full',{}).get('complete') is True,
        callback_to_recorded_imu_exact=accounting.get('callback_vs_recorded_full',{}).get('complete') is True,
        published_to_recorded_imu_exact=accounting.get('published_vs_recorded_full',{}).get('complete') is True,
        published_to_recorded_cloud_exact=cloud_recording.get('complete') is True,
        whole_source_coverage=(result.get('requested_duration_s')!=0 or bool(full_source) and all(v.get('complete') is True for v in full_source.values())))
    return dict(status='PASS_SCOPED_OFFLINE' if all(checks.values()) else 'FAIL',checks=checks,
        failed_checks=[k for k,v in checks.items() if not v],
        accepted_scope='Original-rate execution, IMU callback continuity, raw-topic recording and POST-to-Bundle handoff/exit only; no geometry or hardware certification.',
        unverified=['Raw cloud callback intake has no dedicated pre-processing-stage trace; POST misses cannot all be assigned to subscriber loss.',
                    'Live sensor clock/network gate, foreground record_manager and robot/ARM64/navigation are not exercised by this offline harness.'])

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path);a=p.parse_args();root=a.run.resolve();session=root/'session'
    result=json.loads((root/'result.json').read_text());accounting=audit(session)
    result['full_output_handoff']=full_output_handoff(session)
    with (session/'published_raw.csv').open() as stream:published=list(csv.DictReader(stream))
    clouds=Counter(int(r['header_ns']) for r in published if r['topic']=='/lidar_points')
    cloud_recording=compare(clouds,headers(session/'bag','/lidar_points'))
    full_source={}
    if result.get('requested_duration_s')==0:
        for topic in ['/lidar_imu','/lidar_points']:
            sent=Counter(int(r['header_ns']) for r in published if r['topic']==topic)
            full_source[topic]=compare(headers(result['source_bag'],topic),sent)
    acceptance=judge(result,accounting,cloud_recording,full_source)
    acceptance.update(schema='closed_offline_acceptance_v1',run=str(root),accounting=accounting,cloud_recording=cloud_recording,
        driver_helper_valid_is_not_total_acceptance=result.get('valid'),rate=result.get('rate'),published_counts=result.get('published_counts'),
        requested_duration_s=result.get('requested_duration_s'),source_record_span_s=result.get('source_record_span_s'),full_source_coverage=full_source,
        full_output_handoff=result['full_output_handoff'])
    with (a.output or root/'ACCEPTANCE.json').open('x') as stream:json.dump(acceptance,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(status=acceptance['status'],failed_checks=acceptance['failed_checks']),indent=2))
    return 0 if acceptance['status']=='PASS_SCOPED_OFFLINE' else 2

if __name__=='__main__':raise SystemExit(main())
