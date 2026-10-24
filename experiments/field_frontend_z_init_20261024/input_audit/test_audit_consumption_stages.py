#!/usr/bin/env python3
"""Small CSV fixtures test accounting, real skips, terminal suffix and loss detection."""
from collections import Counter
import csv
import json
from pathlib import Path
import tempfile
import unittest
import audit_consumption_stages as audit


def write_csv(path, columns, values):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,columns);writer.writeheader();writer.writerows(values)


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))


class AuditTest(unittest.TestCase):
    def fixture(self,root):
        run=root/'run';original=root/'original';run.mkdir();original.mkdir()
        base=1792724411400161982 # Far beyond 2^53: integer identities must survive CSV.
        values=dict(ax='0.1',ay='-0.2',az='9.81',gx='0.001',gy='-0.002',gz='0.003')
        imu=[dict(raw_imu_id=i,header_ns=base+i*100+90,record_ns=base+i*100+100,
                  sensor_input_order=2*i,**values) for i in range(5)]
        cloud=[dict(raw_scan_id=i,header_ns=base+i*100+50,record_ns=base+i*100+60,
                    sensor_input_order=2*i+1) for i in range(4)]
        write_csv(original/'imu_raw.csv',list(imu[0]),imu)
        write_csv(original/'cloud_timing.csv',list(cloud[0]),cloud)
        consumed=[]
        for topic,identity,data in [('/lidar_imu','raw_imu_id',imu),('/lidar_points','raw_scan_id',cloud)]:
            for row in data:consumed.append(dict(topic=topic,id=row[identity],record_ns=row['record_ns'],header_ns=row['header_ns'],order=row['sensor_input_order']))
        consumed.sort(key=lambda r:r['order'])
        for row in consumed:del row['order']
        write_csv(run/'reconstruction/consumed_raw.csv',list(consumed[0]),consumed)
        events=[]
        def event(stage,identity,stamp,group=0,a=0,b=0):
            events.append(dict(stage=stage,stamp_ns=stamp,steady_ns=len(events),id=identity,group=group,a=a,b=b))
        for i,row in enumerate(imu):
            for stage in ['imu_raw','imu_adapted','imu_logged','imu_buffered']:event(stage,i+1,row['header_ns'])
            if i<4:event('imu_synced',i+1,row['header_ns'],i+1)
        outcomes=['first_scan_skip','empty_cloud_skip','tree_initialization','post_lidar_update']
        for i,row in enumerate(cloud):
            for stage in ['cloud_raw','cloud_preprocessed','cloud_buffered']:event(stage,i+1,row['header_ns'])
            for stage in ['cloud_synced','compute_begin',outcomes[i],'compute_end']:event(stage,i+1,row['header_ns'],i+1)
            event('sync_group',i+1,row['header_ns'],i+1,imu[i]['header_ns'],imu[i]['header_ns'])
        write_csv(run/'session/frontend_input_stages.csv',list(events[0]),events)
        counts=Counter(r['stage'] for r in events)
        write_json(run/'session/frontend_processing_status.json',dict(closed=True,flush_ok=True,computing=False,
            trace_overflow=0,status_write_errors=0,lidar_buffer_size=0,imu_buffer_size=1,
            imu_buffer_front_ns=imu[-1]['header_ns'],imu_buffer_back_ns=imu[-1]['header_ns'],counts=counts))
        write_json(run/'reconstruction/consumption_summary.json',dict(terminal_imu=1,pending_cloud=0,
            exact_expected_source_counts=True,full_requested=True,post=1))
        write_json(run/'session/init_quality.csv.metadata.json',dict(input=dict(imu_minus_lidar_offset_ns=0,gyro_unit='rad/s',accel_unit='m/s^2')))
        adapters=[]
        for row in imu:
            adapted=dict(status='adapted',raw_stamp_ns=row['header_ns'],aligned_stamp_ns=row['header_ns'],conversion_count=1,time_adjustment_count=1)
            for key,value in values.items():adapted['raw_'+key]=adapted['si_'+key]=value
            adapters.append(adapted)
        write_csv(run/'session/input.csv',list(adapters[0]),adapters)
        preprocess=[dict(source_scan_id=i,header_ns=r['header_ns'],point_count=3) for i,r in enumerate(cloud)]
        write_csv(run/'reconstruction/preprocess_identity.csv',list(preprocess[0]),preprocess)
        post=dict(source_scan_id=3,source_header_stamp_ns=cloud[3]['header_ns'],scan_begin_ns=cloud[3]['header_ns'],scan_end_ns=base+395)
        write_csv(run/'reconstruction/post_index.csv',list(post),[post])
        z=[]
        for i in range(1,4):
            row=dict(source_scan_id=i,source_header_stamp_ns=cloud[i]['header_ns'],scan_begin_ns=cloud[i]['header_ns'],scan_end_ns=base+i*100+95)
            for stage in ['PRE_PROPAGATION','POST_PROPAGATION']+(['POST_LIDAR_UPDATE'] if i==3 else []):z.append(dict(stage=stage,**row))
        write_csv(run/'reconstruction/z_stages.csv',list(z[0]),z)
        write_csv(run/'session/runtime_state.csv',['stage','scan_begin_ns','scan_end_ns'],
                  [dict(stage='INIT_COMMIT',scan_begin_ns=cloud[1]['header_ns'],scan_end_ns=base+195)])
        return run,original,events

    def test_integer_stamp_multiplicity_is_not_a_set(self):
        stamp=1792724411400161982
        missing,unexpected=audit.counter_delta(Counter({stamp:2}),Counter({stamp:1}))
        self.assertEqual(missing,Counter({stamp:1}));self.assertFalse(unexpected)
        missing,unexpected=audit.counter_delta(Counter({stamp:1}),Counter({stamp+1:1}))
        self.assertEqual(missing,Counter({stamp:1}));self.assertEqual(unexpected,Counter({stamp+1:1}))
        self.assertEqual(float(stamp),float(stamp+1)) # Float parsing would conceal this mismatch.

    def test_valid_skips_and_terminal_are_not_loss(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run,source,_=self.fixture(root)
            self.assertTrue(audit.audit(run,source,root/'report'))
            report=audit.load_json(root/'report/summary.json')
            self.assertEqual(report['terminal_imu']['count'],1)
            self.assertEqual(report['initialization_commit_source_scan_ids'],[1])
            self.assertEqual(report['outcomes']['first_scan_skip']['synced_imu_samples'],1)
            self.assertEqual(report['outcomes']['post_lidar_update']['scans'],1)

    def test_missing_adapter_event_is_detected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);run,source,events=self.fixture(root)
            events=[r for r in events if not (r['stage']=='imu_adapted' and r['id']==2)]
            write_csv(run/'session/frontend_input_stages.csv',list(events[0]),events)
            self.assertFalse(audit.audit(run,source,root/'report'))
            report=audit.load_json(root/'report/summary.json')
            bad=[r for r in report['comparisons'] if r['name']=='source_aligned_to_imu_adapted'][0]
            self.assertEqual(bad['missing_count'],1);self.assertEqual(bad['unexpected_count'],0)


if __name__=='__main__':unittest.main()
