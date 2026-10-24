import copy
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent


class FieldContracts(unittest.TestCase):
    def api(self):
        self.assertTrue((HERE/'field.py').is_file(), 'independent field entry required')
        spec = importlib.util.spec_from_file_location('field_test_api', HERE/'field.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_policy_exact_values_and_default_off(self):
        m = self.api()
        params = m.lib.parameters(HERE/'fixed_extrinsic.yaml')
        source = m.lib.read_json(m.INTERFACE_SOURCE)
        with self.assertRaises(ValueError):
            m.authorize(source, HERE/'fixed_extrinsic.yaml', params, 'a'*64, 'user', False)
        contract, policy = m.authorize(source, HERE/'fixed_extrinsic.yaml', params, 'a'*64, 'user', True)
        self.assertEqual([], m.policy_errors(policy, contract, params, 'a'*64))
        self.assertFalse(policy['physical_calibration_verified'])
        for key, val in [('extrinsic_est_en', True), ('extrinsic_T', [0, 0, 0]),
                         ('extrinsic_T', [0.00862, -0.00914, -0.03916])]:
            bad = copy.deepcopy(params)
            bad['mapping'][key] = val
            self.assertTrue(m.policy_errors(policy, contract, bad, 'a'*64))
        self.assertTrue(m.policy_errors(policy, contract, params, 'b'*64))

    def test_derived_parameters_preserve_fixed_state_and_input_gates(self):
        m = self.api()
        p = m.frontend_parameters(HERE/'fixed_extrinsic.yaml', m.lib.read_json(m.INTERFACE_SOURCE), '/tmp/session')
        self.assertFalse(p['mapping']['extrinsic_est_en'])
        self.assertEqual([-0.00862, 0.00914, 0.03916], p['mapping']['extrinsic_T'])
        self.assertTrue(p['frontend_init_v4']['quality_gate_enabled'])
        self.assertEqual(100, p['field_capture']['imu_depth'])
        self.assertFalse(p['use_sim_time'])
        self.assertNotIn('joint_ground', p)

    def test_actual_state_changes_cannot_hide_behind_parameter_false(self):
        m = self.api()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'runtime_state.csv'
            columns = ['seq', 'stage', 't_I_L_x', 't_I_L_y', 't_I_L_z',
                       'q_I_L_x', 'q_I_L_y', 'q_I_L_z', 'q_I_L_w']
            row = dict(zip(columns, [0, 'INIT_COMMIT', -.00862, .00914, .03916, 0, 0, 0, 1]))
            def write(rows):
                with path.open('w') as f:
                    writer = csv.DictWriter(f, fieldnames=columns)
                    writer.writeheader(); writer.writerows(rows)
            write([row, dict(row, seq=1, stage='POST_LIDAR_UPDATE')])
            self.assertTrue(m.fixed_state_report(path)['valid'])
            write([row, dict(row, seq=1, stage='POST_LIDAR_UPDATE', t_I_L_z=.040)])
            self.assertFalse(m.fixed_state_report(path)['valid'])
            write([])
            self.assertFalse(m.fixed_state_report(path)['valid'])

    def test_live_partial_row_is_pending_but_closed_partial_is_failure(self):
        m = self.api()
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'runtime.csv'
            p.write_text('seq,stage,t_I_L_x,t_I_L_y,t_I_L_z,q_I_L_x,q_I_L_y,q_I_L_z,q_I_L_w\n'
                         '0,INIT_COMMIT,-0.00862,0.00914,0.03916,0,0,0,1\n'
                         '1,POST_LIDAR_UPDATE,-0.00862,0.00914,0.03916,0,0,0,1\n'
                         '2,POST_LIDAR_UPDATE,-0.00862,')
            self.assertTrue(m.fixed_state_report(p)['valid'])
            self.assertTrue(m.fixed_state_report(p)['partial_row_pending'])
            self.assertFalse(m.fixed_state_report(p,closed=True)['valid'])

    def test_matches_require_actual_counts_identities_and_freshness(self):
        m = self.api()
        observer = 'field_capture_observer_abc'
        records = {role: dict(schema='publisher_matched_edges_v1', steady_ns=100,
                             ready=True, edges=[dict(topic=t, matched_count=2, subscribers=[node, observer])
                             for t in topics])
                   for role, topics, node in [
                       ('frontend', ['/Odometry', '/cloud_registered_body'], 'mapping_ingress_node'),
                       ('ingress', ['/mapping/keyframe_odom', '/mapping/keyframe_cloud'], 'mapping_backend_node')]}
        records['frontend']['edges'][0].update(matched_count=3,subscribers=['mapping_ingress_node',observer,'rosbag2_recorder'])
        self.assertTrue(m.matches_valid(records, observer, 101))
        bad = copy.deepcopy(records); bad['frontend']['edges'][0]['matched_count'] = 0
        self.assertFalse(m.matches_valid(bad, observer, 101))
        self.assertFalse(m.matches_valid(records, observer, 10_000_000_101))

    def test_recorder_uses_real_128mib_and_only_six_topics(self):
        m = self.api()
        args = m.lib.record_command('/tmp/session', 'none', m.CACHE_MIB)
        self.assertEqual('134217728', args[args.index('--max-cache-size')+1])
        self.assertNotIn('--compression-mode', args)
        self.assertEqual(m.lib.TOPICS, args[-6:])
        self.assertIn('--max-bag-size', args)

    def test_only_backend_actual_commit_audit_is_displayed(self):
        m = self.api()
        self.assertTrue(m.committed_line('[INFO] [LoopCommitted] current=31 candidate=0'))
        self.assertFalse(m.committed_line('[回环成功] accepted=true 约束待图优化提交'))
        self.assertFalse(m.committed_line('LoopCommittedFake candidate'))

    def test_ready_requires_actual_backend_receiving_and_processed(self):
        m=self.api()
        self.assertTrue(m.backend_receiving(dict(reason='received=2 processed=2 pending=0 active=none state=receiving',steady_ns=100),101))
        self.assertFalse(m.backend_receiving(dict(reason='received=2 processed=0 pending=2 active=0 state=receiving',steady_ns=100),101))
        self.assertFalse(m.backend_receiving(dict(reason='received=2 processed=2 pending=0 active=none state=failed',steady_ns=100),101))
        self.assertFalse(m.backend_receiving(dict(reason='received=2 processed=2 pending=0 active=none state=receiving',steady_ns=100),20_000_000_101))

    def test_smoke_failure_cannot_be_reported_as_success(self):
        import offline_smoke as smoke
        good=dict(exit_codes={'frontend':0,'ingress':0,'backend':0,'recorder':0,'field_observer':0},
                  fixed_state_closed=dict(valid=True,post=187),runtime_shutdown=dict(valid=True),
                  handoff=dict(valid=True),recorder_log=dict(checked=True,loss_reported=False,errors=[]),backend_receiving=True)
        self.assertTrue(smoke.smoke_passed(good))
        for key,bad in [('handoff',dict(valid=False)),('recorder_log',dict(checked=True,loss_reported=True,errors=['lost'])),
                        ('fixed_state_closed',dict(valid=True,post=1)),('backend_receiving',False)]:
            changed=copy.deepcopy(good); changed[key]=bad
            self.assertFalse(smoke.smoke_passed(changed))


if __name__ == '__main__':
    unittest.main()
