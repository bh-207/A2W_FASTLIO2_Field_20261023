#!/usr/bin/env python3
"""Cross-check extracted integer inputs and the archived field initialization."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def read_csv(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def rotate(q, v):
    x,y,z,w=q
    t=[2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])]
    return [v[0]+w*t[0]+y*t[2]-z*t[1],v[1]+w*t[1]+z*t[0]-x*t[2],v[2]+w*t[2]+x*t[1]-y*t[0]]


def analyze(extracted, archived):
    imu = read_csv(extracted/'imu_raw.csv')
    by_stamp = {int(r['header_ns']):r for r in imu}
    stamps = [int(r['header_ns']) for r in imu]
    times = [float(t)*1e-9 for t in stamps]
    init = [r for r in read_csv(archived/'init_quality.csv') if r['stage']=='initialized'][0]
    members = [int(x) for x in init['window_member_stamps_ns'].split(';') if x]
    matched = [by_stamp[t] for t in members if t in by_stamp]
    raw_mean = [sum(float(r[k]) for r in matched)/len(matched) for k in ['ax','ay','az']]
    archived_mean = [float(init[k]) for k in ['mean_ax','mean_ay','mean_az']]
    alpha=9.81/norm(archived_mean)
    ba=[float(init[k]) for k in ['bax','bay','baz']]
    gravity=[float(init[k]) for k in ['g_Wx','g_Wy','g_Wz']]
    q=[float(init[k]) for k in ['q_WIx','q_WIy','q_WIz','q_WIw']]
    internal_mean=[alpha*x for x in archived_mean]
    residual=[x+y for x,y in zip(rotate(q,[a-b for a,b in zip(internal_mean,ba)]),gravity)]
    state_checks=read_csv(archived/'init_quality.csv.state_checks.csv')[-1]
    last=by_stamp[int(init['last_stamp_ns'])]
    last_raw=[float(last[k]) for k in ['ax','ay','az']]
    last_residual=[x+y for x,y in zip(rotate(q,[alpha*a-b for a,b in zip(last_raw,ba)]),gravity)]
    runtime=read_csv(archived/'runtime_state.csv')
    coverage=[]
    for row in read_csv(extracted/'cloud_tail_samples.csv'):
        end=float(row['predicted_scan_end_double_s'])
        split=bisect.bisect_right(times,end)
        before=stamps[split-1] if split else None
        after=stamps[split] if split<len(stamps) else None
        coverage.append(dict(raw_scan_id=int(row['raw_scan_id']),source_header_ns=int(row['header_ns']),
                      estimated_scan_end_s=end,last_imu_at_or_before_end_ns=before,next_imu_after_end_ns=after,
                      extrapolation_from_last_selected_imu_s=end-times[split-1] if split else None,
                      future_buffer_support_after_end=after is not None))
    gaps=[stamps[i]-stamps[i-1] for i in range(1,len(stamps))]
    report=dict(schema='imu_domain_and_time_contract_audit_v1',
      source_extraction=str(extracted),archived_session=str(archived),
      archived_initialization=dict(label='old live frontend window, not the new complete-consumption window',
        first_stamp_ns=int(init['first_stamp_ns']),last_stamp_ns=int(init['last_stamp_ns']),
        member_count=len(members),exact_original_bag_matches=len(matched),
        raw_mean_si=archived_mean,recomputed_raw_mean_si=raw_mean,
        maximum_mean_difference=max(abs(x-y) for x,y in zip(raw_mean,archived_mean)),
        raw_mean_norm=norm(archived_mean),alpha=alpha,alpha_squared=alpha*alpha,
        internal_mean=internal_mean,ba_internal=ba,g_world=gravity,g_norm=norm(gravity),
        mean_world_acc_residual=residual,mean_residual_norm=norm(residual),
        mean_residual_matches_existing_guard=abs(norm(residual)-float(state_checks['mean_norm']))<1e-12,
        last_raw_acc=last_raw,last_world_acc_residual=last_residual,last_residual_norm=norm(last_residual),
        last_residual_matches_existing_guard=abs(norm(last_residual)-float(state_checks['last_norm']))<1e-12,
        initial_covariance_diagonal=dict(pos=[1,1,1],rot=[1,1,1],ext_rot=[1e-5]*3,
          ext_t=[1e-5]*3,vel=[1]*3,bg=[1e-4]*3,ba_internal=[1e-3]*3,gravity_S2_tangent=[1e-5]*2)),
      domain_contract=dict(raw_input='archived source-declared SI: a_raw m/s^2 and gyro rad/s',
        acceleration_prediction='in.acc = alpha * 0.5*(a_raw_head+a_raw_tail)',
        dynamics='a_world = R_WI*(alpha*a_raw - ba_internal) + g_world',
        alpha='9.81/norm(accepted raw-SI mean_acc); frozen at initialization',
        ba_internal='subtracted after alpha; effective internal input bias in m/s^2',
        ba_raw_equivalent='ba_internal/alpha for a deterministic alpha; interpretation only',
        covariance_conversion='Pba_internal = alpha^2*Pba_raw; cross blocks get alpha once',
        acc_s_last='world linear acceleration after orientation and gravity, not raw accelerometer',
        gravity_norm=9.809,normalization_numerator=9.81,
        Q='configured internal-domain diagonal; measured raw variance is not used',
        Q_discretization='IKFoM adds (dt*G)*Q*(dt*G)^T, not Q*dt'),
      timestamp_contract=dict(raw_header='integer sec*1e9+nanosec from original message',
        point_zero='original absolute double timestamp of point 0',
        curvature='float32((point_time-point0_time)*1000) in milliseconds',
        scan_begin='rclcpp::Time(header).seconds(), epoch double',
        scan_end='scan_begin + last retained point curvature/1000, with source mean fallback',
        sync='wait for a buffered IMU >= scan_end; consume only IMU <= scan_end; future sample stays buffered',
        propagation='prepend last_imu, skip tails before previous scan end, predict intervals, extrapolate to scan_end',
        offset='aligned_IMU_ns = raw_IMU_ns - configured_offset exactly once, current offset=0',
        raw_gap_action='preserved; no message inserted, no offset changed',
        archived_init_log_scan_end_minus_state_stamp_ns=int(runtime[0]['scan_end_ns'])-int(runtime[0]['state_stamp_ns']),
        archived_INIT_log_warning='state_stamp logs accepted last IMU, while predictor boundary is set to scan_end'),
      source_gap_counts=dict(imu_over7_5ms=sum(x>7_500_000 for x in gaps),
        imu_over10ms=sum(x>10_000_000 for x in gaps),imu_over20ms=sum(x>20_000_000 for x in gaps),
        imu_max_gap_ns=max(gaps),imu_nonpositive_gap_count=sum(x<=0 for x in gaps)),
      sampled_scan_end_coverage=coverage,
      archived_runtime_states=dict(rows=len(runtime),
        final_ba_internal=[float(runtime[-1]['ba_I_'+a]) for a in 'xyz'],
        final_position=[float(runtime[-1]['pos_O_I_'+a]) for a in 'xyz'],
        interpretation='existing estimator output, not physical bias or position truth'),
      validation_scope='read-only original Bag small fields plus CSV arithmetic; no build, ROS runtime, replay or hardware test')
    out=extracted/'imu_domain_summary.json'
    if out.exists():raise FileExistsError(out)
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    assert len(imu)==166023 and len(by_stamp)==166023
    assert len(matched)==len(members)==220
    assert report['archived_initialization']['maximum_mean_difference']<1e-12
    assert report['archived_initialization']['mean_residual_matches_existing_guard']
    assert report['archived_initialization']['last_residual_matches_existing_guard']
    assert len(coverage)==10 and all(r['future_buffer_support_after_end'] for r in coverage)
    print(json.dumps(dict(output=str(out),alpha=alpha,mean_residual_norm=norm(residual),
       last_residual_norm=norm(last_residual),exact_window_matches=len(matched),validation='PASS')))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--extracted',type=Path,required=True);p.add_argument('--archived',type=Path,required=True)
    a=p.parse_args();analyze(a.extracted.resolve(),a.archived.resolve())
