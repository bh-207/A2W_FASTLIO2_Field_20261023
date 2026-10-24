#!/usr/bin/env python3
"""Integer/multiplicity audit of saved CSVs; this tool never opens a Bag.

Source CSVs come from the one-time read-only extraction. Raw callbacks use
1-based per-topic IDs, whereas consumed_raw/post_index use 0-based source IDs.
Every timestamp is parsed with int(), never through float seconds.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path


OUTCOMES = ['first_scan_skip', 'initialization_skip', 'empty_cloud_skip',
            'tree_initialization', 'few_features_skip', 'post_lidar_update']
EXPLANATIONS = {
    'first_scan_skip': 'Existing timer first-frame branch: synchronized IMUs are consumed, but Process is not called; establish first_lidar_time.',
    'initialization_skip': 'Process feeds the real initialization gate; no deskew cloud/laser update is released while initialization is pending.',
    'empty_cloud_skip': 'Process returns an empty cloud after initialization commit in this run; no laser update. Must be checked against the actual INIT_COMMIT record.',
    'tree_initialization': 'Process propagates/deskews and the existing map tree is built; this branch returns before the first laser update.',
    'few_features_skip': 'Process propagated/deskewed, but the existing minimum-feature branch skipped laser update; this is not a POST.',
    'post_lidar_update': 'Actual iterative laser update completed; a fresh unvoxelized deskew body cloud was exported.',
}


def rows(path):
    with path.open(newline='') as stream:
        yield from csv.DictReader(stream)


def load_json(path):
    return json.loads(path.read_text())


def counter_delta(expected, actual):
    """Retain multiplicity, including repeated header stamps; never compare sets."""
    missing, unexpected = expected-actual, actual-expected
    return missing, unexpected


def write_rows(path, columns, data):
    with path.open('x', newline='') as stream:
        writer=csv.DictWriter(stream,columns);writer.writeheader();writer.writerows(data)


def audit(run, original, output):
    run=run.resolve();original=original.resolve();output=output.resolve()
    if output.exists():raise FileExistsError('fresh audit output directory required: '+str(output))
    output.mkdir(parents=True)
    failures=[];comparisons=[];difference_rows=[]
    def require(condition, name, details=None):
        if not condition:failures.append(dict(check=name, details=details))
    def compare(name, expected, actual):
        missing,unexpected=counter_delta(expected,actual)
        result=dict(name=name,expected_count=sum(expected.values()),actual_count=sum(actual.values()),
                    missing_count=sum(missing.values()),unexpected_count=sum(unexpected.values()),
                    exact_integer_multiplicity_match=not missing and not unexpected)
        comparisons.append(result)
        require(result['exact_integer_multiplicity_match'],name,result)
        for direction,counter in [('missing',missing),('unexpected',unexpected)]:
            for identity,multiplicity in counter.items():
                difference_rows.append(dict(comparison=name,direction=direction,
                    identity=json.dumps(identity,separators=(',',':')),multiplicity=multiplicity))
        return result
    imu=list(rows(original/'imu_raw.csv'));cloud=list(rows(original/'cloud_timing.csv'))
    source={'/lidar_imu':imu,'/lidar_points':cloud}
    source_ids={'/lidar_imu':'raw_imu_id','/lidar_points':'raw_scan_id'}
    source_identity={topic:Counter((int(r[source_ids[topic]])+1,int(r['header_ns'])) for r in data)
                     for topic,data in source.items()}
    expected_order=sorted((int(r['sensor_input_order']),topic,int(r[source_ids[topic]]),
                           int(r['record_ns']),int(r['header_ns']))
                          for topic,data in source.items() for r in data)
    expected_order=[record[1:] for record in expected_order]
    consumed=defaultdict(Counter);sequence_mismatches=[];consumed_count=0
    for i,row in enumerate(rows(run/'reconstruction/consumed_raw.csv')):
        record=(row['topic'],int(row['id']),int(row['record_ns']),int(row['header_ns']))
        consumed[row['topic']][(record[1]+1,record[3])]+=1
        if i>=len(expected_order) or record!=expected_order[i]:
            if len(sequence_mismatches)<32:sequence_mismatches.append(dict(position=i,actual=record,
                expected=expected_order[i] if i<len(expected_order) else None))
        consumed_count+=1
    require(consumed_count==len(expected_order) and not sequence_mismatches,
            'exact_sensor_record_order_and_integer_record/header_stamps',sequence_mismatches)
    for topic in source:compare('source_to_consumed_'+topic,source_identity[topic],consumed[topic])
    metadata=load_json(run/'session/init_quality.csv.metadata.json')
    offset=int(metadata['input']['imu_minus_lidar_offset_ns'])
    expected_aligned=Counter((identity,stamp-offset) for identity,stamp in source_identity['/lidar_imu'].elements())
    counters=defaultdict(Counter);stage_counts=Counter();group_meta={};group_imu=defaultdict(list)
    outcome_events=defaultdict(list);cloud_event_groups={};max_group=0
    for row in rows(run/'session/frontend_input_stages.csv'):
        stage=row['stage'];identity=int(row['id']);stamp=int(row['stamp_ns']);group=int(row['group'])
        stage_counts[stage]+=1;counters[stage][(identity,stamp)]+=1
        if stage=='sync_group':
            require(group not in group_meta,'unique_sync_group',group)
            group_meta[group]=dict(callback_id=identity,source_scan_id=identity-1,source_header_ns=stamp,
                first_imu_ns=int(row['a']),last_imu_ns=int(row['b']))
            max_group=max(max_group,group)
        elif stage=='imu_synced':group_imu[group].append((identity,stamp))
        if stage in OUTCOMES:outcome_events[group].append(stage)
        if stage in ['cloud_synced','compute_begin','compute_end',*OUTCOMES]:
            cloud_event_groups.setdefault(stage,Counter())[(identity,stamp,group)]+=1
    for topic,stage in [('/lidar_imu','imu_raw'),('/lidar_points','cloud_raw')]:
        compare('source_to_callback_'+topic,source_identity[topic],counters[stage])
    for stage in ['imu_adapted','imu_logged','imu_buffered']:
        compare('source_aligned_to_'+stage,expected_aligned,counters[stage])
    for stage in ['cloud_preprocessed','cloud_buffered','cloud_synced','sync_group','compute_begin','compute_end']:
        compare('source_to_'+stage,source_identity['/lidar_points'],counters[stage])
    compare('cloud_synced_to_compute_begin_group_identity',cloud_event_groups['cloud_synced'],cloud_event_groups['compute_begin'])
    compare('cloud_synced_to_compute_end_group_identity',cloud_event_groups['cloud_synced'],cloud_event_groups['compute_end'])
    outcome_counter=Counter()
    for name in OUTCOMES:outcome_counter.update(cloud_event_groups.get(name,{}))
    compare('cloud_synced_to_exactly_one_terminal_outcome',cloud_event_groups['cloud_synced'],outcome_counter)
    require(all(len(outcome_events[g])==1 for g in group_meta),'one_outcome_per_sync_group')
    require(set(group_meta)==set(range(1,len(cloud)+1)),'contiguous_groups_1_to_cloud_count')
    for stage in ['imu_rejected','imu_cleared','cloud_cleared']:
        require(stage_counts[stage]==0,'no_'+stage,stage_counts[stage])
    _,unexpected_sync=counter_delta(counters['imu_buffered'],counters['imu_synced'])
    require(not unexpected_sync,'no_unbuffered_or_repeated_imu_sync',list(unexpected_sync.items()))
    terminal=counters['imu_buffered']-counters['imu_synced']
    status=load_json(run/'session/frontend_processing_status.json')
    consumption=load_json(run/'reconstruction/consumption_summary.json')
    require(status['closed'] and status['flush_ok'] and not status['computing'],'integrity_trace_closed_healthy')
    for name in ['trace_overflow','status_write_errors','lidar_buffer_size']:
        require(status[name]==0,'terminal_status_'+name,status[name])
    # CSV emits no rows for zero-valued stages; Counter equality retains zeros correctly.
    require(Counter(status['counts'])==stage_counts,'status_counts_equal_saved_integer_ledger',
            {'status':status['counts'],'csv':dict(stage_counts)})
    require(sum(terminal.values())==status['imu_buffer_size']==consumption['terminal_imu'],
            'terminal_buffer_count_exact',{'unconsumed':sum(terminal.values()),'status':status['imu_buffer_size'],'summary':consumption['terminal_imu']})
    require(consumption['pending_cloud']==0 and consumption['exact_expected_source_counts'] and consumption['full_requested'],
            'complete_requested_and_cloud_buffer_drained')
    # All input.csv rows stay in the raw/SI adapter domain, before internal alpha.
    adapter=Counter();raw_adapter=Counter();adapter_errors=[];adapter_count=0
    for i,row in enumerate(rows(run/'session/input.csv')):
        raw_stamp=int(row['raw_stamp_ns']);aligned=int(row['aligned_stamp_ns'])
        raw_adapter[(i+1,raw_stamp)]+=1;adapter[(i+1,aligned)]+=1;adapter_count+=1
        ok=(row['status']=='adapted' and int(row['conversion_count'])==1 and
            int(row['time_adjustment_count'])==1 and aligned==raw_stamp-offset)
        if i<len(imu):
            for suffix,key in [('ax','ax'),('ay','ay'),('az','az'),('gx','gx'),('gy','gy'),('gz','gz')]:
                ok=ok and float(row['raw_'+suffix])==float(imu[i][key])
                # This input contract is already SI, so adapting must preserve values.
                ok=ok and float(row['si_'+suffix])==float(row['raw_'+suffix])
        else:ok=False
        if not ok and len(adapter_errors)<32:adapter_errors.append(i)
    compare('source_to_input_csv_raw',source_identity['/lidar_imu'],raw_adapter)
    compare('source_aligned_to_input_csv',expected_aligned,adapter)
    require(not adapter_errors,'input_csv_raw_SI_values_once_conversion',adapter_errors)
    require(metadata['input']['gyro_unit']=='rad/s' and metadata['input']['accel_unit']=='m/s^2',
            'declared_SI_adapter_identity_scope')
    preprocess=Counter();point_counts={}
    for row in rows(run/'reconstruction/preprocess_identity.csv'):
        identity=int(row['source_scan_id'])+1;header=int(row['header_ns']);preprocess[(identity,header)]+=1
        point_counts[identity-1]=int(row['point_count'])
    compare('source_to_preprocess_identity',source_identity['/lidar_points'],preprocess)
    post=Counter();scan_ends={};post_rows=[]
    for row in rows(run/'reconstruction/post_index.csv'):
        source_id=int(row['source_scan_id']);header=int(row['source_header_stamp_ns'])
        post[(source_id+1,header)]+=1;scan_ends[source_id]=int(row['scan_end_ns']);post_rows.append(row)
    compare('post_stage_to_post_index',counters['post_lidar_update'],post)
    require(sum(post.values())==consumption['post'],'post_index_count_equals_closed_summary')
    z=defaultdict(Counter);scan_bounds={}
    for row in rows(run/'reconstruction/z_stages.csv'):
        source_id=int(row['source_scan_id']);header=int(row['source_header_stamp_ns'])
        z[row['stage']][(source_id+1,header)]+=1
        scan_bounds[source_id]=(int(row['scan_begin_ns']),int(row['scan_end_ns']))
    compare('sync_to_PRE_or_first_scan_skip',counters['cloud_synced'],z['PRE_PROPAGATION']+counters['first_scan_skip'])
    compare('PRE_to_POSTPROP_or_INIT_PENDING',z['PRE_PROPAGATION'],z['POST_PROPAGATION']+z['INIT_PENDING'])
    compare('init_pending_to_initialization_skip',z['INIT_PENDING'],counters['initialization_skip'])
    compare('post_stage_to_z_stages_POST',counters['post_lidar_update'],z['POST_LIDAR_UPDATE'])
    compare('POSTPROP_to_empty_tree_few_or_POST',z['POST_PROPAGATION'],
            counters['empty_cloud_skip']+counters['tree_initialization']+counters['few_features_skip']+counters['post_lidar_update'])
    init_commit=[r for r in rows(run/'session/runtime_state.csv') if r['stage']=='INIT_COMMIT']
    require(len(init_commit)==1,'one_actual_INIT_COMMIT',len(init_commit))
    init_scan_ids=[]
    if len(init_commit)==1:
        begin,end=int(init_commit[0]['scan_begin_ns']),int(init_commit[0]['scan_end_ns'])
        init_scan_ids=[sid for sid,bounds in scan_bounds.items() if bounds==(begin,end)]
        require(len(init_scan_ids)==1,'unique_INIT_COMMIT_scan_bounds',init_scan_ids)
    empty_ids=[identity-1 for identity,_ in counters['empty_cloud_skip'].elements()]
    require(empty_ids==init_scan_ids,'empty_output_is_exact_initialization_commit_scan',
            {'empty_source_ids':empty_ids,'init_source_ids':init_scan_ids})
    group_records=[];outcome_imu_counts=Counter();outcome_scan_ids=defaultdict(list)
    for group,meta in sorted(group_meta.items()):
        members=group_imu[group];outcome=outcome_events[group][0] if len(outcome_events[group])==1 else 'INVALID'
        expected_first=members[0][1] if members else 0;expected_last=members[-1][1] if members else 0
        require(meta['first_imu_ns']==expected_first and meta['last_imu_ns']==expected_last,
                'group_first_last_matches_integer_imu_members',group)
        outcome_imu_counts[outcome]+=len(members);outcome_scan_ids[outcome].append(meta['source_scan_id'])
        bounds=scan_bounds.get(meta['source_scan_id'],('',''))
        group_records.append(dict(group=group,**meta,scan_begin_ns=bounds[0],scan_end_ns=bounds[1],
            synced_imu_count=len(members),outcome=outcome,preprocess_point_count=point_counts.get(meta['source_scan_id'],'')))
    last_source_id=len(cloud)-1
    require(last_source_id in scan_bounds,'final_source_scan_has_actual_scan_end')
    final_scan_end=scan_bounds.get(last_source_id,(0,0))[1]
    require(final_scan_end>0,'positive_final_scan_end')
    expected_terminal=Counter((identity,raw_stamp-offset)
       for identity,raw_stamp in source_identity['/lidar_imu'].elements() if raw_stamp-offset>final_scan_end)
    compare('terminal_buffer_equals_original_IMUs_after_final_scan_end',expected_terminal,terminal)
    if terminal:
        sorted_terminal=sorted(terminal.elements())
        require(status['imu_buffer_front_ns']==sorted_terminal[0][1] and status['imu_buffer_back_ns']==sorted_terminal[-1][1],
                'terminal_buffer_exact_front_back',sorted_terminal[:1]+sorted_terminal[-1:])
    terminal_records=[dict(raw_imu_id=identity-1,callback_id=identity,raw_header_ns=aligned+offset,
       aligned_header_ns=aligned,after_final_scan_end_ns=aligned-final_scan_end,multiplicity=count)
       for (identity,aligned),count in sorted(terminal.items())]
    write_rows(output/'terminal_imu.csv',['raw_imu_id','callback_id','raw_header_ns','aligned_header_ns','after_final_scan_end_ns','multiplicity'],terminal_records)
    write_rows(output/'group_accounting.csv',list(group_records[0]),group_records)
    write_rows(output/'integer_stage_differences.csv',['comparison','direction','identity','multiplicity'],difference_rows)
    outcome_summary={name:dict(scans=stage_counts[name],synced_imu_samples=outcome_imu_counts[name],
       source_scan_ids=outcome_scan_ids[name] if name!='post_lidar_update' else None,
       first_source_scan_id=min(outcome_scan_ids[name]) if outcome_scan_ids[name] else None,
       last_source_scan_id=max(outcome_scan_ids[name]) if outcome_scan_ids[name] else None,
       explanation=EXPLANATIONS[name]) for name in OUTCOMES}
    report=dict(schema='integer_frontend_consumption_stage_audit_v1',status='PASS' if not failures else 'FAIL',
       run=str(run),original_extracted_input=str(original),bag_opened=False,parsed_stamp_type='integer_only',
       source_imu=len(imu),source_cloud=len(cloud),all_sensor_record_order_exact=not sequence_mismatches and consumed_count==len(expected_order),
       imu_minus_lidar_offset_ns=offset,comparisons=comparisons,stage_counts=dict(stage_counts),outcomes=outcome_summary,
       cloud_sync_equals_outcomes=sum(stage_counts[n] for n in OUTCOMES)==len(cloud),
       IMU_sync_is_not_equal_to_IMU_predict_use='first scan skips Process, initialization groups feed the gate, later groups propagate/deskew',
       terminal_imu=dict(count=sum(terminal.values()),final_source_scan_id=last_source_id,final_scan_end_ns=final_scan_end,
         first_aligned_ns=min((t for _,t in terminal),default=None),last_aligned_ns=max((t for _,t in terminal),default=None),
         no_pending_cloud=consumption['pending_cloud']==0,explanation='Original messages after the final scan end remain buffered; no later cloud exists to consume them. They were delivered/adapted/logged/buffered, not lost or fabricated.'),
       initialization_commit_source_scan_ids=init_scan_ids,trace_closed=bool(status['closed']),trace_flush_ok=bool(status['flush_ok']),
       failures=failures,limitations=['Source extraction covers saved Bag only, not device upstream delivery.',
         'Exact offline consumption is not original-rate/DDS/hardware acceptance.',
         'No claim that every source IMU produces a laser update; real stage skips are listed.',
         'Timestamp/source-gap completeness does not validate physical R/T/time-offset calibration.'])
    (output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    zh=[run.name+' 整数身份/重数阶段审计：'+report['status'],
        f'仅读取已有CSV/JSON；原Bag未打开。source IMU={len(imu)}，cloud={len(cloud)}。',
        f'callback/adapt/log/buffer完整性及全部{len(comparisons)}项重数对比见summary.json；差异明细见integer_stage_differences.csv。',
        '同步cloud每帧恰好一个合法终结阶段：']
    for name in OUTCOMES:zh.append(f'  {name}: cloud={stage_counts[name]}，该组同步IMU={outcome_imu_counts[name]}，source_scan_ids={outcome_summary[name]["source_scan_ids"] if name!="post_lidar_update" else "见group_accounting.csv"}')
    zh.extend([f'最后扫描source_scan_id={last_source_id}，scan_end_ns={final_scan_end}。',
      f'末尾{sum(terminal.values())}条IMU均已callback/adapt/log/buffer，时间严格晚于最后扫描结束，无后续cloud，故合法留在buffer；不是丢输入。',
      'first_scan_skip组已同步，但原算法未调用Process；initialization_skip组供初始化门；初始化commit返回空云；tree_initialization组传播/去畸变后建树；这些均不是POST。',
      '不把166003条sync IMU宣称为166003条独立predict：初始化和跨扫描边界由既有Process处理。',
      '顺序离线输入完整不等于原速、DDS或现场验收；缺口与物理标定状态未改变。'])
    (output/'SUMMARY_ZH.txt').write_text('\n'.join(zh)+'\n')
    print(json.dumps(dict(status=report['status'],output=str(output),comparisons=len(comparisons),failures=len(failures),
       imu=len(imu),cloud=len(cloud),sync_imu=stage_counts['imu_synced'],post=stage_counts['post_lidar_update'],terminal=sum(terminal.values()))))
    return not failures


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--original',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();raise SystemExit(0 if audit(a.run,a.original,a.output) else 1)
