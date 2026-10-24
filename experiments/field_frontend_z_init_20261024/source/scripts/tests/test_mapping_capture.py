"""Controlled capture safety contracts; no ROS nodes or driver are started."""
import copy
import csv
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))


class CaptureContracts(unittest.TestCase):
    def setUp(self):
        import mapping_capture_lib as capture
        self.c = capture
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.base = self.root / 'jt128.yaml'
        self.base.write_text('/**:\n  ros__parameters:\n    common:\n      lid_topic: /lidar_points\n      imu_topic: /lidar_imu\n      time_sync_en: false\n      time_offset_lidar_to_imu: 0.0\n    mapping:\n      extrinsic_T: [0, 0, 0]\n      extrinsic_R: [1,0,0,0,1,0,0,0,1]\n')

    def tearDown(self):
        self.tmp.cleanup()

    def interface(self):
        evidence = {'source': 'driver/source.cpp', 'version': 'commit-123',
                    'config': 'deployment-config', 'sha256': 'a' * 64}
        return {'schema': 'mapping_input_interface_v1', 'source_provenance': 'real',
                'gyro_unit': 'rad/s', 'accel_unit': 'm/s^2',
                'imu_frame': 'hesai_lidar', 'lidar_frame': 'hesai_lidar',
                'imu_minus_lidar_offset_ns': 0,
                'offset_definition': 't_imu_raw_minus_t_lidar_same_event',
                'gyro_unit_evidence': evidence, 'accel_unit_evidence': evidence,
                'time_offset_evidence': evidence, 'frame_evidence': evidence,
                'extrinsic_evidence': dict(evidence, base_config_sha256=self.c.sha256(self.base))}

    def test_unknown_template_lists_missing_evidence_but_is_serializable(self):
        data = self.c.interface_template()
        json.dumps(data)
        errors = self.c.interface_errors(data, self.base)
        self.assertTrue(any('gyro_unit' in e for e in errors))
        self.assertTrue(any('time_offset_evidence' in e for e in errors))

    def test_units_require_explicit_evidence_and_original_offset(self):
        data = self.interface()
        self.assertEqual([], self.c.interface_errors(data, self.base))
        for field, value in [('gyro_unit', 'auto'), ('accel_unit', 'auto'),
                             ('imu_minus_lidar_offset_ns', 1), ('source_provenance', 'synthetic')]:
            invalid = copy.deepcopy(data); invalid[field] = value
            self.assertTrue(self.c.interface_errors(invalid, self.base), field)
        invalid = copy.deepcopy(data); invalid['gyro_unit_evidence']['sha256'] = 'unknown'
        self.assertTrue(self.c.interface_errors(invalid, self.base))

    def test_snapshot_strict_frontend_preserves_extrinsic_and_disables_auto(self):
        params = self.c.frontend_parameters(self.base, self.interface(), self.root)
        self.assertEqual('rad', params['common']['imu_gyr_unit'])
        self.assertIs(False, params['common']['time_sync_en'])
        self.assertEqual([0, 0, 0], params['mapping']['extrinsic_T'])
        self.assertTrue(params['frontend_init_v4']['quality_gate_enabled'])
        self.assertEqual(1024, params['frontend_init_v4']['runtime_state_log_capacity'])
        self.assertEqual('real', params['frontend_init_v4']['source_provenance'])

    def test_enhanced_backend_is_rejected_even_nested_or_dotted(self):
        for params in [{'loop_pipeline': {'revisit_verification_enabled': True}},
                       {'loop_pipeline.revisit_verification_enabled': True},
                       {'quatro': {'feature_cache_capacity': 16}}]:
            with self.assertRaises(ValueError): self.c.assert_conservative(params)
        self.c.assert_conservative({'loop_pipeline': {'revisit_verification_enabled': False}})

    def test_record_command_exact_topics_and_bounded_compression(self):
        command = self.c.record_command(self.root, 'zstd')
        self.assertNotIn('-a', command)
        self.assertEqual(1, command.count('/lidar_points'))
        self.assertIn('536870912', command)
        self.assertIn('300', command)
        self.assertIn('--compression-threads', command)
        self.assertEqual('16777216',command[command.index('--max-cache-size')+1])
        self.assertEqual('resilient',command[command.index('--storage-preset-profile')+1])
        qos = self.c.qos_overrides()
        self.assertEqual('transient_local', qos['/tf_static']['durability'])
        self.assertEqual(1, qos['/tf_static']['depth'])
        self.assertEqual('best_effort', qos['/lidar_imu']['reliability'])
        self.assertEqual(32,qos['/lidar_points']['depth'])
        self.assertEqual(100,qos['/tf']['depth'])

    def test_init_commit_not_odometry_and_gravity_angle(self):
        path = self.root / 'runtime_state.csv'
        columns = ['stage','g_O_x','g_O_y','g_O_z','init_stamp_consistent','state_stamp_ns',
                   'accepted_window_last_imu_ns','group_last_imu_ns']
        with path.open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns); writer.writeheader()
            writer.writerow(dict(zip(columns, ['POST_LIDAR_UPDATE',0,0,-9.81,1,42,42,42])))
        self.assertFalse(self.c.runtime_ready(path)['ready'])
        with path.open('a') as stream:
            csv.writer(stream).writerow(['INIT_COMMIT',0,0,-9.81,1,43,43,43])
        result = self.c.runtime_ready(path)
        self.assertTrue(result['ready'])
        self.assertEqual(0.0, result['anti_g_to_camera_init_z_rad'])
        self.assertFalse(result['map_gravity_alignment'])

    def test_init_commit_requires_all_three_exact_source_stamps(self):
        path = self.root/'runtime_state.csv'
        path.write_text('stage,g_O_x,g_O_y,g_O_z,init_stamp_consistent,state_stamp_ns,accepted_window_last_imu_ns,group_last_imu_ns\n'
                        'INIT_COMMIT,0,0,-9.81,1,100,100,101\n')
        self.assertFalse(self.c.runtime_ready(path)['ready'])

    def test_runtime_shutdown_counts_must_match_actual_rows(self):
        path = self.root/'runtime_state.csv'; path.write_text('stage\nINIT_COMMIT\nPOST_LIDAR_UPDATE\n')
        status = dict(closed=True,flush_ok=True,dropped=0,write_errors=0,enqueued=2,written=2,attempted=2)
        self.c.write_json(str(path)+'.status.json',status)
        self.assertTrue(self.c.runtime_shutdown(path)['valid'])
        status['written']=1; self.c.write_json(str(path)+'.status.json',status)
        self.assertFalse(self.c.runtime_shutdown(path)['valid'])

    def test_observer_does_not_infer_units_and_flags_stamp_reversal(self):
        accumulator = self.c.InputAccumulator(self.root)
        fields=[dict(name=n,datatype=d,count=1,offset=0) for n,d in
                [('x',7),('y',7),('z',7),('intensity',7),('ring',4),('timestamp',8)]]
        accumulator.observe('/lidar_points',100,'frame',100,fields=fields)
        accumulator.observe('/lidar_points',99,'frame',101,fields=fields)
        data=accumulator.report()
        self.assertEqual(2,data['topics']['/lidar_points']['count'])
        self.assertTrue(data['topics']['/lidar_points']['errors'])
        self.assertNotIn('gyro_unit',data)

    def test_invalid_gravity_or_uncommitted_stamp_never_ready(self):
        path = self.root / 'runtime_state.csv'
        for gravity, consistent in [('nan', 1), ('0',1), ('-9.81',0)]:
            path.write_text('stage,g_O_x,g_O_y,g_O_z,init_stamp_consistent\n'
                            f'INIT_COMMIT,0,0,{gravity},{consistent}\n')
            self.assertFalse(self.c.runtime_ready(path)['ready'])

    def test_manifest_requires_all_core_files_and_confined_unique_hashes(self):
        bundle = self.root / 'bundle'; bundle.mkdir()
        files = {'metadata.yaml': 'schema: a2w_map_bundle_v1\nbundle_uuid: id1\nkeyframe_count: 1\ndescriptor_count: 1\n',
                 'optimized_poses.csv': 'id,tx,ty,tz,qx,qy,qz,qw\n0,0,0,0,0,0,0,1\n',
                 'keyframes/index.csv': 'id,stamp_ns,cloud,tx,ty,tz,qx,qy,qz,qw\n0,1,keyframes/clouds/000000.pcd,0,0,0,0,0,0,1\n',
                 'keyframes/clouds/000000.pcd':'cloud',
                 'config/mapping_effective.yaml':'x: 1', 'config/registration.yaml':'x: 1',
                 'config/scan_context.yaml':'x: 1',
                 'descriptors/scan_context.bin': 'test', 'global_map.pcd': 'pcd'}
        for name, value in files.items():
            p = bundle / name; p.parent.mkdir(parents=True, exist_ok=True); p.write_text(value)
        manifest = bundle / 'manifest.sha256'
        manifest.write_text(''.join(f'{self.c.sha256(bundle/name)}  {name}\n' for name in sorted(files)))
        self.assertEqual(1, self.c.verify_bundle(bundle, 'id1', 1)['keyframe_count'])
        original=manifest.read_text(); manifest.write_text(''.join(line+'\n' for line in original.splitlines() if not line.endswith('keyframes/clouds/000000.pcd')))
        with self.assertRaises(ValueError): self.c.verify_bundle(bundle,'id1',1)
        manifest.write_text(original)
        (bundle / 'optimized_poses.csv').write_text('corrupt')
        with self.assertRaises(ValueError): self.c.verify_bundle(bundle, 'id1', 1)
        manifest.write_text('a'*64+'  ../escape\n')
        with self.assertRaises(ValueError): self.c.verify_bundle(bundle)

    def test_process_identity_rejects_pid_reuse_and_wrong_command(self):
        identity = self.c.process_identity(os.getpid())
        self.assertTrue(self.c.process_matches(identity))
        wrong = dict(identity, start_ticks=identity['start_ticks']+1)
        self.assertFalse(self.c.process_matches(wrong))
        wrong = dict(identity, cmdline=['unrelated'])
        self.assertFalse(self.c.process_matches(wrong))

    def test_input_report_requires_reception_fields_frames_and_clock(self):
        report = {'session': str(self.root), 'updated_unix_ns': 100,
                  'recorder_subscriptions': {'/lidar_points': True, '/lidar_imu': True},
                  'topics': {'/lidar_points': {'count': 2, 'frame_id': 'hesai_lidar',
                             'stamp_ns': 1000, 'last_receipt_unix_ns':100,'clock_skew_ns': 0, 'errors': [],
                             'fields': [{'name': n, 'datatype': d, 'count': 1} for n,d in
                                        [('x',7),('y',7),('z',7),('intensity',7),('ring',4),('timestamp',8)]]},
                             '/lidar_imu': {'count': 3, 'frame_id': 'hesai_lidar', 'stamp_ns':1000,
                                            'last_receipt_unix_ns':100,'clock_skew_ns': 0,'errors': []}}}
        self.assertEqual([], self.c.input_report_errors(report,self.interface(),self.root,100))
        report['topics']['/lidar_points']['fields'].pop()
        self.assertTrue(self.c.input_report_errors(report,self.interface(),self.root,100))
        report['topics']['/lidar_imu']['clock_skew_ns'] = 10_000_000_000
        self.assertTrue(any('clock' in x for x in self.c.input_report_errors(report,self.interface(),self.root,100)))

    def test_fresh_report_cannot_hide_stopped_raw_stream(self):
        accumulator=self.c.InputAccumulator(self.root)
        accumulator.observe('/lidar_imu',100,'hesai_lidar',100)
        report=accumulator.report(); report['updated_unix_ns']=10_000_000_000
        errors=self.c.input_report_errors(report,self.interface(),self.root,10_000_000_000)
        self.assertTrue(any('reception stale' in error for error in errors))

    def test_sqlite_inventory_counts_splits_without_fabricating_tf_static(self):
        bag = self.root / 'bag'; bag.mkdir()
        for i in range(2):
            connection = sqlite3.connect(bag/f'bag_{i}.db3')
            connection.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);'
                                     'CREATE TABLE messages(topic_id INTEGER,timestamp INTEGER,data BLOB);')
            connection.execute('INSERT INTO topics VALUES(1,?,?)',('/lidar_imu','sensor_msgs/msg/Imu'))
            connection.execute('INSERT INTO messages VALUES(1,?,?)',(100+i,b'cdr'))
            connection.commit(); connection.close()
        result = self.c.sqlite_inventory(bag)
        self.assertEqual(2,result['topics']['/lidar_imu']['count'])
        self.assertIn('/tf_static',result['missing_topics'])
        self.assertFalse(result['lossless_proven'])

    def test_session_creation_is_exclusive_and_unknown_allows_raw_only(self):
        import mapping_capture as cli
        session = cli.init_session('capture-one', self.root/'captures', self.root/'absent-build',
                                   None, base=self.base)
        self.assertTrue((session/'session.json').is_file())
        self.assertTrue((session/'snapshots/frontend.yaml').is_file())
        status = cli.check_session(session)
        self.assertTrue(status['record_allowed'])
        self.assertFalse(status['start_allowed'])
        with self.assertRaises((ValueError, FileExistsError)):
            cli.init_session('capture-one',self.root/'captures',self.root/'absent-build',None,base=self.base)
        with self.assertRaises(ValueError): cli.init_session('../escape',self.root,self.root,None,base=self.base)

    def test_stop_refuses_unsaved_running_session_without_signalling(self):
        import mapping_capture as cli
        session = cli.init_session('unsaved',self.root/'captures',self.root/'build',None,base=self.base)
        state = {'frontend': self.c.process_identity(os.getpid())}
        self.c.write_json(session/'processes.json',state)
        with self.assertRaises(ValueError): cli.stop_session(session)
        self.assertTrue(self.c.process_matches(state['frontend']))

    def test_check_detects_snapshot_tamper_even_when_contract_unknown(self):
        import mapping_capture as cli
        session = cli.init_session('tamper',self.root/'captures',self.root/'build',None,base=self.base)
        (session/'snapshots/frontend.yaml').write_text('modified')
        self.assertTrue(any('snapshot' in x for x in cli.check_session(session)['errors']))

    def test_binary_hash_and_resolved_library_identity_are_rechecked(self):
        import mapping_capture as cli
        exe = self.root/'frontend'; exe.write_bytes(b'binary')
        lib = self.root/'lib.so'; lib.write_bytes(b'lib')
        identity = {'path': str(exe),'sha256':self.c.sha256(exe),
                    'dsos':[{'path':str(lib),'sha256':self.c.sha256(lib)}]}
        self.assertEqual([],cli.file_identity_errors(identity))
        lib.write_bytes(b'replaced')
        self.assertTrue(cli.file_identity_errors(identity))

    def test_save_evidence_must_be_success_with_pending_zero_and_valid_manifest(self):
        import mapping_capture as cli
        with self.assertRaises(ValueError):
            cli.validate_save_response({'success':False}, {'stage':'backend','reason':'pending=0 active=none state=saved'})
        with self.assertRaises(ValueError):
            cli.validate_save_response({'success':True}, {'stage':'backend','reason':'pending=1 active=none state=saved'})
        with self.assertRaises(ValueError):
            cli.validate_save_response({'success':True}, {'stage':'loop','reason':'pending=0 active=none state=saved'})
        cli.validate_save_response({'success':True}, {'stage':'backend','reason':'received=3 processed=3 pending=0 active=none state=saved'})

    def test_point_parser_reports_all_fields_and_does_not_change_bytes(self):
        import mapping_capture as cli
        fields=[SimpleNamespace(name=n,datatype=d,count=1,offset=o) for n,d,o in
                [('x',7,0),('y',7,4),('z',7,8),('intensity',7,12),('ring',4,16),('timestamp',8,24)]]
        data=struct.pack('<ffffH6xd',1,2,3,4,8,100.0)+struct.pack('<ffffH6xd',2,3,4,5,9,100.1)
        cloud=SimpleNamespace(fields=fields,point_step=32,row_step=64,width=2,height=1,data=data,is_bigendian=False)
        result=cli.pointcloud_summary(cloud)
        self.assertEqual(6,len(result['fields']))
        self.assertAlmostEqual(.1,result['point_timestamp_span_s'])
        self.assertEqual(data,cloud.data)
        cloud.fields=fields[:-1]
        self.assertTrue(cli.pointcloud_summary(cloud)['errors'])

    def test_cli_help_and_interface_template_need_no_ros(self):
        import subprocess
        result=subprocess.run([sys.executable,str(SCRIPTS/'mapping_capture.py'),'interface-template'],capture_output=True,text=True)
        self.assertEqual(0,result.returncode,result.stderr)
        self.assertEqual('unknown',json.loads(result.stdout)['gyro_unit'])

    def test_malformed_evidence_is_reported_not_unhandled(self):
        contract=self.interface(); contract['extrinsic_evidence']=None
        self.assertTrue(self.c.interface_errors(contract,self.base))

    def test_explicit_frontend_manifest_path_supports_verified_isolation(self):
        import mapping_capture as cli
        controlled=self.root/'controlled'; controlled.mkdir()
        install=controlled/'backend_install'; install.mkdir()
        isolation=self.root/'verified_isolation'; isolation.mkdir()
        exe=isolation/'frontend'; exe.write_bytes(b'front')
        frontend=dict(path=str(exe),sha256=self.c.sha256(exe))
        self.c.write_json(isolation/'build_manifest.json',{'binaries':{'fastlio_mapping':frontend}})
        binaries={'frontend':frontend}
        for role,name in [('backend','mapping_backend_node'),('ingress','mapping_ingress_node')]:
            path=install/'a2w_fastlio_mapping/lib/a2w_fastlio_mapping'/name
            path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(role.encode())
            binaries[role]=dict(path=str(path),sha256=self.c.sha256(path))
        self.c.write_json(controlled/'build_identity.json',dict(frontend_binary=str(exe),frontend_isolation=str(isolation),
            frontend_build_manifest=str(isolation/'build_manifest.json'),backend_install=str(install),binaries=binaries))
        # ldd is the external boundary; real path selection/hashing/manifest checks remain active.
        with patch.object(cli,'binary_identity',side_effect=lambda path,digest,role,prefix:dict(path=str(path),sha256=digest,dsos=[])):
            result=cli.capture_build_identity(controlled)
        self.assertEqual(str(exe),result['binaries']['frontend']['path'])

    def test_stale_or_different_recorder_report_is_rejected(self):
        import mapping_capture as cli
        state={'recorder':{'pid':12,'start_ticks':3},'record_manager':{'pid':13}}
        report={'recorder_identity':{'pid':14,'start_ticks':3}}
        with self.assertRaises(ValueError): cli.assert_same_recorder_report(state,report)
        report['recorder_identity']=state['recorder']
        with self.assertRaises(ValueError): cli.assert_same_recorder_report(state,report)
        report['recorder_binding']=dict(bound=True,identity_strategy='exclusive_default_node_new_fixed_endpoint_gids',
            node_name='rosbag2_recorder',node_namespace='/',gids={'/lidar_points':'a'*48,'/lidar_imu':'b'*48},errors=[])
        cli.assert_same_recorder_report(state,report)

    def test_parser_rejects_point_field_outside_step(self):
        import mapping_capture as cli
        fields=[SimpleNamespace(name=n,datatype=d,count=1,offset=o) for n,d,o in
                [('x',7,100),('y',7,4),('z',7,8),('intensity',7,12),('ring',4,16),('timestamp',8,24)]]
        cloud=SimpleNamespace(fields=fields,point_step=32,row_step=32,width=1,height=1,data=b'\0'*32,is_bigendian=False)
        self.assertTrue(cli.pointcloud_summary(cloud)['errors'])

    def test_save_target_rejects_symlink_ancestors_and_existing_directory(self):
        import mapping_capture as cli
        session=self.root/'session'; session.mkdir(); (session/'bundles').mkdir()
        self.assertEqual(session/'bundles/map_bundle',cli.fresh_bundle_target(session))
        (session/'bundles/map_bundle').mkdir()
        with self.assertRaises(ValueError): cli.fresh_bundle_target(session)
        other=self.root/'other'; other.mkdir()
        linked=self.root/'linked'; linked.mkdir(); (linked/'bundles').symlink_to(other,target_is_directory=True)
        with self.assertRaises(ValueError): cli.fresh_bundle_target(linked)

    def test_runtime_log_error_overrides_apparently_clean_status(self):
        path=self.root/'runtime_state.csv'; path.write_text('stage\nINIT_COMMIT\n')
        self.c.write_json(str(path)+'.status.json',dict(closed=True,flush_ok=True,dropped=0,write_errors=0,enqueued=1,written=1,attempted=1))
        log=self.root/'frontend.log'
        log.write_text('[frontend_runtime_v4] STATUS_WRITE_ERROR\n[frontend_runtime_v4] CLOSED attempted=1 enqueued=1 written=1 dropped=0 write_errors=0 flush_ok=1\n')
        self.assertFalse(self.c.runtime_shutdown(path,log)['valid'])

    def test_frontend_trace_paths_stay_inside_session(self):
        params=self.c.frontend_parameters(self.base,self.interface(),self.root)
        self.assertEqual(str(self.root/'input.csv'),params['frontend_init_v4']['input_trace_path'])

    def test_bag_child_uses_actual_ros2cli_argv_contract_without_claiming_unique_name(self):
        import mapping_capture as cli
        from types import ModuleType
        session=cli.init_session('child',self.root/'captures',self.root/'build',None,base=self.base)
        record=ModuleType('ros2bag.verb.record'); record.NODE_NAME_PREFIX='default'; record.RecordVerb=object
        ros2bag=ModuleType('ros2bag'); verb=ModuleType('ros2bag.verb'); verb.record=record; ros2bag.verb=verb
        ros2cli=ModuleType('ros2cli'); cli_module=ModuleType('ros2cli.cli')
        def actual_signature(*,argv):
            self.assertEqual(['bag','record'],argv[:2]); self.assertIn('/lidar_points',argv)
            return 0
        cli_module.main=actual_signature; ros2cli.cli=cli_module
        rosbag=ModuleType('rosbag2_py'); rosbag.RecordOptions=lambda:SimpleNamespace(node_prefix='')
        modules={'ros2bag':ros2bag,'ros2bag.verb':verb,'ros2bag.verb.record':record,
                 'ros2cli':ros2cli,'ros2cli.cli':cli_module,'rosbag2_py':rosbag}
        with patch.dict(sys.modules,modules): self.assertEqual(0,cli.bag_child(session,'none'))
        self.assertEqual('default',record.NODE_NAME_PREFIX)

    def test_nonfinite_imu_report_is_serializable_but_invalid(self):
        import mapping_capture as cli
        accumulator=self.c.InputAccumulator(self.root)
        message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0),frame_id='imu'),
            angular_velocity=SimpleNamespace(x=float('nan'),y=0,z=0),linear_acceleration=SimpleNamespace(x=0,y=0,z=9.81))
        cli.observe_raw(accumulator,'/lidar_imu',message,1_000_000_000)
        self.assertTrue(accumulator.topics['/lidar_imu']['errors'])
        json.dumps(accumulator.report(),allow_nan=False)

    def test_stop_discard_never_signals_a_foreign_command_even_with_pid_identity(self):
        import mapping_capture as cli
        session=cli.init_session('foreign',self.root/'captures',self.root/'build',None,base=self.base)
        foreign=dict(self.c.process_identity(os.getpid()),owner_session=str(session),role='frontend')
        self.c.write_json(session/'processes.json',{'frontend':foreign})
        with patch.object(cli,'signal_owned',side_effect=AssertionError('foreign process would be signalled')):
            with self.assertRaises(ValueError): cli.stop_session(session,discard_unsaved=True,timeout=0)

    def test_named_node_collision_is_fail_closed(self):
        import mapping_capture as cli
        with self.assertRaises(ValueError): cli.assert_no_node_collisions([('laser_mapping','/')])
        cli.assert_no_node_collisions([('unrelated','/'),('laser_mapping','/other')])

    def test_mapping_operations_cannot_overlap_across_two_cli_processes(self):
        import mapping_capture as cli
        session=cli.init_session('locked',self.root/'captures',self.root/'build',None,base=self.base)
        import fcntl
        with (session/'.mapping_operation.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError): cli.stop_session(session,discard_unsaved=True,timeout=0)

    def test_ready_requires_runtime_metadata_to_confirm_frame_contract(self):
        import mapping_capture as cli
        session=cli.init_session('metadata',self.root/'captures',self.root/'build',None,base=self.base)
        path=session/'runtime_state.csv'
        path.write_text('stage,g_O_x,g_O_y,g_O_z,init_stamp_consistent,state_stamp_ns,accepted_window_last_imu_ns,group_last_imu_ns\n'
                        'INIT_COMMIT,0,0,-9.81,1,100,100,100\n')
        with patch.object(cli,'process_matches',return_value=True),patch.object(cli,'dump_parameters',return_value={}):
            self.assertFalse(cli.ready_session(session)['ready'])

    def test_humble_qos_result_is_a_plain_tuple(self):
        import mapping_capture as cli
        info=SimpleNamespace(node_name='driver',node_namespace='/',qos_profile=SimpleNamespace(reliability=1,durability=2,depth=4))
        row=cli.publisher_qos_report(info,object(),lambda pub,sub:(2,'incompatible'))
        self.assertEqual(2,row['compatibility_code'])
        self.assertEqual('incompatible',row['compatibility_reason'])

    def test_bag_stamp_extraction_includes_tf_and_registration_status(self):
        import mapping_capture as cli
        stamp=SimpleNamespace(sec=2,nanosec=3)
        status=SimpleNamespace(stamp=stamp)
        self.assertEqual([2_000_000_003],cli.message_stamps('/mapping/registration_status',status))
        transform=SimpleNamespace(header=SimpleNamespace(stamp=stamp))
        self.assertEqual([2_000_000_003],cli.message_stamps('/tf',SimpleNamespace(transforms=[transform])))

    def test_synthetic_mode_requires_explicit_localhost_nonzero_domain_no_profile(self):
        import mapping_capture as cli
        with patch.dict(os.environ,{},clear=True): self.assertTrue(cli.synthetic_environment_errors())
        env={'ROS_LOCALHOST_ONLY':'1','ROS_DOMAIN_ID':'57'}
        with patch.dict(os.environ,env,clear=True): self.assertEqual([],cli.synthetic_environment_errors())
        with patch.dict(os.environ,dict(env,CYCLONEDDS_URI='file:///external.xml'),clear=True):
            self.assertTrue(cli.synthetic_environment_errors())
        with patch.dict(os.environ,env,clear=True):
            session=cli.init_session('synthetic',self.root/'captures',self.root/'build',None,base=self.base,synthetic_test=True)
        self.assertTrue(self.c.read_json(session/'session.json')['synthetic_test'])
        self.assertEqual('synthetic',self.c.parameters(session/'snapshots/frontend.yaml')['frontend_init_v4']['source_provenance'])

    def test_ros_environment_is_set_in_current_process_not_only_children(self):
        import mapping_capture as cli
        session=cli.init_session('env',self.root/'captures',self.root/'build',None,base=self.base)
        with patch.dict(os.environ,{},clear=True):
            cli.ensure_ros_environment(session)
            self.assertEqual(str(session/'ros_logs'),os.environ['ROS_LOG_DIR'])
            self.assertEqual(str(session/'ros_home'),os.environ['ROS_HOME'])

    def test_pre_frontend_requires_committed_rows_not_probe_receipt(self):
        bag=self.root/'bag'; bag.mkdir(); path=bag/'capture_0.db3'
        connection=sqlite3.connect(path)
        connection.execute('PRAGMA journal_mode=WAL')
        connection.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);'
            'CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        for index,topic,type_ in [(1,'/lidar_points','sensor_msgs/msg/PointCloud2'),(2,'/lidar_imu','sensor_msgs/msg/Imu')]:
            connection.execute('INSERT INTO topics VALUES(?,?,?)',(index,topic,type_))
        connection.commit()
        data=b'\0\1\0\0'+struct.pack('<iII',1,2,4)+b'imu\0'
        for index in [1,2]: connection.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',(index,1_000_000_003,data))
        self.assertFalse(self.c.committed_raw_evidence(bag,2_000_000_000)['valid'])
        connection.commit()
        result=self.c.committed_raw_evidence(bag,2_000_000_000)
        self.assertTrue(result['valid'])
        self.assertEqual(1_000_000_002,result['topics']['/lidar_imu']['source_stamp_ns'])
        self.assertEqual(1_000_000_003,result['topics']['/lidar_imu']['record_stamp_ns'])
        connection.close()

    def test_ready_is_false_when_recorder_or_manager_has_died(self):
        import mapping_capture as cli
        session=cli.init_session('dead_bag',self.root/'captures',self.root/'build',None,base=self.base)
        with patch.object(cli,'runtime_commit_report',return_value={'ready':True,'map_gravity_alignment':False}),patch.object(cli,'dump_parameters',return_value={}):
            with patch.object(cli,'process_matches',return_value=True):
                self.assertFalse(cli.ready_session(session)['ready'])

    def test_recording_quality_checks_exit_and_time_coverage_not_just_topic_nonzero(self):
        import mapping_capture as cli
        record_ranges={'/lidar_points':{'first_record_ns':100,'last_record_ns':200},
                       '/lidar_imu':{'first_record_ns':100,'last_record_ns':200}}
        live={'topics':{'/lidar_points':{'last_receipt_unix_ns':200},'/lidar_imu':{'last_receipt_unix_ns':200}}}
        clean=cli.recording_quality({'exit_code':0,'metadata_present':True},record_ranges,live,150)
        self.assertTrue(clean['checks_passed'])
        self.assertFalse(cli.recording_quality({'exit_code':1,'metadata_present':True},record_ranges,live,150)['checks_passed'])
        self.assertFalse(cli.recording_quality({'exit_code':0,'metadata_present':True},record_ranges,live,99)['checks_passed'])
        live['topics']['/lidar_imu']['last_receipt_unix_ns']=10_000_000_000
        self.assertFalse(cli.recording_quality({'exit_code':0,'metadata_present':True},record_ranges,live,150)['checks_passed'])

    def test_backend_message_typesupport_is_isolated_but_not_common_library(self):
        import mapping_capture as cli
        install=self.root/'backend_install'
        libraries=[]
        for package,name in [('a2w_fastlio_common','liba2w_registration.so'),
                             ('a2w_fastlio_common','liba2w_algorithm_suite.so'),
                             ('a2w_fastlio_common','liba2w_runtime_contracts.so'),
                             ('a2w_fastlio_msgs','liba2w_fastlio_msgs__rosidl_typesupport_cpp.so')]:
            path=install/package/'lib'/name; path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(name.encode())
            libraries.append(f'{name} => {path} (0x1234)')
        executable=self.root/'backend'; executable.write_bytes(b'bin')
        with patch.object(cli,'run',return_value=SimpleNamespace(stdout='\n'.join(libraries))):
            entry=cli.binary_identity(executable,self.c.sha256(executable),'backend',install)
        self.assertEqual(4,len(entry['dsos']))

    def test_split_read_order_uses_metadata_not_lexicographic_filename(self):
        bag=self.root/'bag'; bag.mkdir()
        for name in ['bag_2.db3','bag_10.db3']: (bag/name).write_bytes(b'')
        (bag/'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths: [bag_2.db3, bag_10.db3]\n')
        self.assertEqual(['bag_2.db3','bag_10.db3'],[p.name for p in self.c.sqlite_files(bag)])

    def test_backend_base_snapshots_are_exact_source_bytes(self):
        import mapping_capture as cli
        session=cli.init_session('exact_yaml',self.root/'captures',self.root/'build',None,base=self.base)
        for name,source in cli.CONFIG_SOURCES.items():
            self.assertEqual(source.read_bytes(),(session/'snapshots'/name).read_bytes())

    def test_actual_humble_lazy_parser_isolates_bag_child_argv_without_ros_start(self):
        """Regression: BagCommand.add_arguments drops argv; nested parser reads sys.argv."""
        import mapping_capture as cli
        import subprocess
        session=cli.init_session('parse_only',self.root/'captures',self.root/'build',None,base=self.base)
        code='''import sys
sys.path.insert(0,sys.argv[1])
import mapping_capture as capture
import ros2bag.verb.record as record
session=sys.argv[2]
option=sys.argv[3]
def forbidden(*args,**kwargs): raise AssertionError("parse-only test must never start recorder")
record.RecordVerb.main=forbidden
capture.record_command=lambda *args: ["ros2","bag","record",option]
sys.argv=["mapping_capture.py","_bag-child",session]
capture.bag_child(session,"none")
'''
        for option,expected in [('--help',0),('--mapping-capture-invalid',2)]:
            with self.subTest(option=option):
                command=['bash','--noprofile','--norc','-c',
                    'source /opt/ros/humble/setup.bash; exec /usr/bin/python3 -c "$1" "$2" "$3" "$4"',
                    'parse_only',code,str(SCRIPTS),str(session),option]
                result=subprocess.run(command,capture_output=True,text=True,timeout=20)
                self.assertEqual(expected,result.returncode,result.stdout+result.stderr)
                self.assertNotIn("invalid choice: '_bag-child'",result.stderr)
                if option=='--help': self.assertIn('--qos-profile-overrides-path',result.stdout)
                else: self.assertIn('--mapping-capture-invalid',result.stderr)

    def test_base_raw_topics_and_time_sync_must_match_capture_contract(self):
        import yaml
        document=yaml.safe_load(self.base.read_text())
        for key,value in [('lid_topic','/other_points'),('imu_topic','/other_imu'),('time_sync_en',True)]:
            changed=copy.deepcopy(document); changed['/**']['ros__parameters']['common'][key]=value
            with self.subTest(key=key):
                self.base.write_text(yaml.safe_dump(changed))
                self.assertTrue(self.c.interface_errors(self.interface(),self.base))
        self.base.write_text(yaml.safe_dump(document))

    def test_side_output_overrides_are_recorded_without_changing_mapping_or_filters(self):
        import mapping_capture as cli
        import yaml
        document=yaml.safe_load(self.base.read_text()); original=document['/**']['ros__parameters']
        original.update(pcd_save={'pcd_save_en':True,'leaf_size':.37},trajectory_save={'tum_en':True},
                        runtime_pos_log_enable=True,point_filter_num=6,filter_size_surf=.5)
        self.base.write_text(yaml.safe_dump(document))
        contract_path=self.root/'interface.json'; self.c.write_json(contract_path,self.interface())
        session=cli.init_session('overrides',self.root/'captures',self.root/'build',contract_path,base=self.base)
        params=self.c.parameters(session/'snapshots/frontend.yaml')
        self.assertIs(False,params['pcd_save']['pcd_save_en'])
        self.assertIs(False,params['trajectory_save']['tum_en'])
        self.assertIs(False,params['runtime_pos_log_enable'])
        self.assertEqual(original['mapping'],params['mapping'])
        self.assertEqual(6,params['point_filter_num']); self.assertEqual(.5,params['filter_size_surf'])
        self.assertEqual(.37,params['pcd_save']['leaf_size'])
        changes=self.c.read_json(session/'snapshots/frontend_overrides.json')['changes']
        self.assertEqual({'original':True,'snapshot':False},changes['pcd_save.pcd_save_en'])

    def test_real_init_cli_accepts_explicit_calibrated_base_profile(self):
        import mapping_capture as cli
        args=cli.parser().parse_args(['init','real','--base-config',str(self.base)])
        self.assertFalse(args.synthetic_test)
        with patch.object(cli,'init_session',return_value=self.root/'real') as initialize:
            self.assertEqual(0,cli.main(['init','real','--base-config',str(self.base)]))
            self.assertEqual(self.base,initialize.call_args.kwargs['base'])

    def test_actual_rosbag_writer_concurrent_sqlite_reader_never_busy_in_capture_profile(self):
        """Real Humble storage plugin, no rclpy init, node, DDS or Recorder."""
        import subprocess
        code='''import json,sys,sqlite3,threading,time
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from mapping_capture_lib import record_command,committed_raw_evidence
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu,PointCloud2
bag=Path(sys.argv[2]); mode=sys.argv[3]
command=record_command(bag.parent)
preset=(command[command.index('--storage-preset-profile')+1] if '--storage-preset-profile' in command else '') if mode=='capture' else ''
writer=rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3',max_cache_size=0,storage_preset_profile=preset),rosbag2_py.ConverterOptions('',''))
topics=[('/lidar_points','sensor_msgs/msg/PointCloud2',PointCloud2()),('/lidar_imu','sensor_msgs/msg/Imu',Imu())]
for topic,type_,msg in topics:
    writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=type_,serialization_format='cdr'))
    msg.header.stamp.sec=1; msg.header.frame_id='sensor'
    writer.write(topic,serialize_message(msg),1000000000)
file=next(bag.glob('*.db3')); reader=sqlite3.connect('file:'+str(file)+'?mode=ro',uri=True,timeout=.2)
journal=reader.execute('PRAGMA journal_mode').fetchone()[0]
reader.execute('BEGIN'); cursor=reader.execute('SELECT count(*) FROM messages'); cursor.fetchone(); cursor.close()
stop=threading.Event(); polls=[]; poll_errors=[]
def poll():
    while not stop.is_set():
        try: polls.append(committed_raw_evidence(bag,3000000000))
        except Exception as error: poll_errors.append(str(error))
        time.sleep(.0005)
thread=threading.Thread(target=poll); thread.start()
failure=None; wrote=0
try:
    for index in range(200):
        topic,type_,msg=topics[index%2]
        writer.write(topic,serialize_message(msg),1000000001+index)
        wrote+=1; time.sleep(.0005)
except Exception as error: failure=str(error)
finally:
    stop.set(); thread.join(); reader.rollback(); reader.close(); writer.close()
connection=sqlite3.connect('file:'+str(file)+'?mode=ro',uri=True)
count=connection.execute('SELECT count(*) FROM messages').fetchone()[0]; connection.close()
print('AUDIT_JSON='+json.dumps(dict(mode=mode,preset=preset,journal=journal,wrote=wrote,count=count,failure=failure,polls=len(polls),poll_errors=poll_errors,valid_polls=sum(x['valid'] for x in polls))))
'''
        for mode in ['default_negative','capture']:
            with self.subTest(mode=mode):
                command=['bash','--noprofile','--norc','-c',
                    'source /opt/ros/humble/setup.bash; exec /usr/bin/python3 -c "$1" "$2" "$3" "$4"',
                    'sqlite_probe',code,str(SCRIPTS),str(self.root/mode),mode]
                result=subprocess.run(command,capture_output=True,text=True,timeout=30)
                self.assertEqual(0,result.returncode,result.stdout+result.stderr)
                lines=[line for line in result.stdout.splitlines() if line.startswith('AUDIT_JSON=')]
                self.assertEqual(1,len(lines),result.stdout+result.stderr)
                row=json.loads(lines[0].split('=',1)[1])
                if mode=='default_negative':
                    self.assertIsNotNone(row['failure'],row)
                    self.assertIn('locked',row['failure'])
                else:
                    self.assertEqual('wal',row['journal'],row)
                    self.assertIsNone(row['failure'],row)
                    self.assertEqual(202,row['count'],row)
                    self.assertGreater(row['polls'],0,row)
                    self.assertGreater(row['valid_polls'],0,row)
                    self.assertEqual([],row['poll_errors'],row)

    def test_live_sqlite_guard_waits_for_wal_initialization_without_read_lock(self):
        bag=self.root/'not_ready'; bag.mkdir()
        connection=sqlite3.connect(bag/'capture.db3')
        connection.execute('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT)'); connection.close()
        with patch.object(self.c.sqlite3,'connect',side_effect=AssertionError('must not race writer journal initialization')):
            evidence=self.c.committed_raw_evidence(bag)
        self.assertFalse(evidence['valid'])
        self.assertTrue(any('WAL' in error for error in evidence['errors']))

    def endpoint(self,gid,node='rosbag2_recorder',namespace='/'):
        return dict(node_name=node,node_namespace=namespace,gid=gid*48)

    def test_recorder_binding_refuses_existing_default_recorder(self):
        with self.assertRaises(ValueError): self.c.RecorderEndpointBinding([('rosbag2_recorder','/')],{})
        with self.assertRaises(ValueError): self.c.RecorderEndpointBinding([],
            {'/lidar_points':[self.endpoint('a')]})

    def test_recorder_binding_requires_unique_new_two_raw_gids_and_never_trusts_prefix(self):
        binding=self.c.RecorderEndpointBinding([],{'/lidar_points':[],'/lidar_imu':[]})
        spoof={'/lidar_points':[self.endpoint('a','mapping_capture_fake')],
               '/lidar_imu':[self.endpoint('b','mapping_capture_fake')]}
        self.assertFalse(binding.update([('mapping_capture_fake','/')],spoof)['bound'])
        actual={'/lidar_points':[self.endpoint('a')],'/lidar_imu':[self.endpoint('b')]}
        result=binding.update([('rosbag2_recorder','/')],actual)
        self.assertTrue(result['bound'])
        self.assertFalse(result['unique_node_name'])
        self.assertEqual('a'*48,result['gids']['/lidar_points'])

    def test_recorder_binding_rejects_second_recorder_extra_gid_and_drift(self):
        actual={'/lidar_points':[self.endpoint('a')],'/lidar_imu':[self.endpoint('b')]}
        for changed_nodes,changed_endpoints in [
            ([('rosbag2_recorder','/'),('rosbag2_recorder','/')],actual),
            ([('rosbag2_recorder','/')],dict(actual,**{'/lidar_points':[self.endpoint('a'),self.endpoint('c')]})),
            ([('rosbag2_recorder','/')],dict(actual,**{'/lidar_imu':[self.endpoint('c')]}))]:
            with self.subTest(nodes=changed_nodes,endpoints=changed_endpoints):
                binding=self.c.RecorderEndpointBinding([],{})
                self.assertTrue(binding.update([('rosbag2_recorder','/')],actual)['bound'])
                self.assertFalse(binding.update(changed_nodes,changed_endpoints)['bound'])
                self.assertFalse(binding.update([('rosbag2_recorder','/')],actual)['bound'],'never silently rebind after ambiguity/drift')

    def test_startup_summary_is_pure_complete_and_does_not_claim_gravity_alignment(self):
        import mapping_capture as cli
        session=self.root/'summary'
        frontend=self.c.frontend_parameters(self.base,self.interface(),session)
        identity={'source':{'git_head':'head123','git_dirty':True},'snapshots':{'snapshots/frontend.yaml':'f'*64},
            'base_source':str(self.base),'base_sha256':self.c.sha256(self.base),
            'backend_runtime_parameters':{'loop_pipeline':{'revisit_verification_enabled':False},'bundle_root':str(session/'bundles')},
            'build':{'manifests':{'build_manifest.json':'m'*64},'frontend_source_manifest':{'base_commit':'base123','patch_sha256':'p'*64},
                     'binaries':{role:{'path':'/controlled/'+role,'sha256':role+'hash','dsos':[{'path':'/lib/system.so','sha256':'sohash'}]} for role in ['frontend','backend','ingress']}}}
        metadata={'synthetic_test':True,'uuid':'sessionid','ros_domain_id':'57','ros_localhost_only':'1'}
        before=copy.deepcopy(identity)
        result=self.c.startup_summary(identity,metadata,frontend,session,self.interface())
        self.assertEqual(before,identity)
        self.assertEqual('/controlled/frontend',result['binaries']['frontend']['path'])
        self.assertEqual('p'*64,result['versions']['frontend_patch_sha256'])
        self.assertEqual('rad/s',result['input_contract']['gyro_unit'])
        self.assertEqual('/lidar_points',result['topics']['raw_lidar'])
        self.assertTrue(result['gates']['quality_gate_enabled'])
        self.assertFalse(result['gates']['backend_revisit_verification_enabled'])
        self.assertEqual(str(session/'ros_logs'),result['paths']['ROS_LOG_DIR'])
        self.assertEqual(str(session/'snapshots/frontend.yaml'),result['config_snapshots'][0]['path'])
        self.assertFalse(result['map_gravity_alignment'])
        self.assertFalse(result['real_hardware_evidence'])
        json.dumps(result,allow_nan=False)

    def test_recorder_binding_rejects_preexisting_gid_and_missing_bound_endpoint(self):
        baseline={'/lidar_points':[self.endpoint('a','other_observer')]}
        current={'/lidar_points':[self.endpoint('a')],'/lidar_imu':[self.endpoint('b')]}
        binding=self.c.RecorderEndpointBinding([],baseline)
        self.assertFalse(binding.update([('rosbag2_recorder','/')],current)['bound'])
        binding=self.c.RecorderEndpointBinding([],{})
        self.assertTrue(binding.update([('rosbag2_recorder','/')],current)['bound'])
        self.assertFalse(binding.update([('rosbag2_recorder','/')],{'/lidar_points':current['/lidar_points']})['bound'])

    def test_empty_real_requested_session_never_claims_hardware_evidence(self):
        import mapping_capture as cli
        session=cli.init_session('unverified_real',self.root/'captures',self.root/'build',None,base=self.base)
        metadata=self.c.read_json(session/'session.json')
        self.assertEqual('real_requested',metadata['capture_mode'])
        self.assertFalse(metadata['real_hardware_evidence'])
        self.assertEqual('unverified',metadata['hardware_evidence_status'])
        self.assertFalse(cli.check_session(session)['start_allowed'])

    def test_startup_environment_distinguishes_init_declaration_from_actual_start(self):
        frontend=self.c.frontend_parameters(self.base,self.interface(),self.root)
        metadata={'synthetic_test':False,'ros_domain_id':'0','ros_localhost_only':'0'}
        actual={'ROS_DOMAIN_ID':'57','ROS_LOCALHOST_ONLY':'1','ROS_HOME':'/session/ros_home'}
        summary=self.c.startup_summary({},metadata,frontend,self.root,self.interface(),actual_start_environment=actual)
        self.assertEqual('0',summary['ros_environment']['declared_at_init']['ROS_DOMAIN_ID'])
        self.assertEqual('57',summary['ros_environment']['actual_at_start']['ROS_DOMAIN_ID'])
        self.assertEqual('real_requested',summary['capture_mode'])
        self.assertFalse(summary['real_hardware_evidence'])
        self.assertEqual('unverified',summary['hardware_evidence_status'])
        actual['ROS_DOMAIN_ID']='changed'
        self.assertEqual('57',summary['ros_environment']['actual_at_start']['ROS_DOMAIN_ID'])


if __name__ == '__main__': unittest.main()
