"""Behavioral checks for recording plans and timestamp-aware trial evaluation."""
import importlib.util
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]


def module():
    spec = importlib.util.spec_from_file_location('trial_metrics', ROOT / 'scripts/diagnostics/trial_metrics.py')
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def pose(x=0., y=0., yaw=0.):
    return NS(position=NS(x=x, y=y, z=0.),
              orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))


def header(stamp, frame='map'):
    return NS(stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9), frame_id=frame)


def tf(stamp, parent, child, p):
    return NS(header=header(stamp, parent), child_frame_id=child,
              transform=NS(translation=p.position, rotation=p.orientation))


class TrialMetricsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = module()

    def test_record_plan_only_starts_recorder_and_preserves_sensor_and_action_topics(self):
        with tempfile.TemporaryDirectory() as folder:
            plan = self.m.record_plan(Path(folder), 'avoidance', 60, ROOT)
        self.assertEqual(plan['argv'][:3], ['ros2', 'bag', 'record'])
        for topic in ['/lidar_points', '/lidar_imu', '/Odometry', '/cloud_registered_body',
                      '/rosout', '/tf_static', '/route_follower/raw_path',
                      '/route_follower/terminal_error_body', '/cmd_vel_guarded',
                      '/follow_path/_action/feedback', '/collision_monitor_state']:
            self.assertIn(topic, plan['topics'])
        self.assertNotIn('launch', plan['argv'])
        self.assertEqual(plan['qos']['/tf_static']['durability'], 'transient_local')
        self.assertEqual(plan['qos']['/cloud_registered_body']['reliability'], 'best_effort')
        self.assertEqual(plan['qos']['/plan']['durability'], 'volatile')

    def test_nonfinite_duration_and_unknown_scenario_are_rejected(self):
        for duration in [0, -1, math.nan, math.inf]:
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                self.m.record_plan(Path('/tmp/trial'), 'straight', duration, ROOT)
        with self.assertRaises(ValueError):
            self.m.record_plan(Path('/tmp/trial'), 'anything', 10, ROOT)

    def test_sensor_headers_and_receipt_intervals_are_distinct(self):
        evaluator = self.m.TrialEvaluator()
        for stamp, receipt in [(1_000_000_000, 1_030_000_000), (1_100_000_000, 1_250_000_000)]:
            evaluator.observe('/Odometry', 'nav_msgs/msg/Odometry',
                              NS(header=header(stamp, 'camera_init'), child_frame_id='body',
                                 pose=NS(pose=pose())), receipt)
        result = evaluator.summary()['topics']['/Odometry']
        self.assertAlmostEqual(result['header_interval_ms']['p50'], 100)
        self.assertAlmostEqual(result['receipt_interval_ms']['p50'], 220)
        self.assertAlmostEqual(result['receipt_minus_header_ms']['max'], 150)

    def test_terminal_error_uses_map_base_extrinsic_and_preserves_original_goal(self):
        evaluator = self.m.TrialEvaluator(goal=(5., 3., 0.))
        stamp = 1_000_000_000
        evaluator.observe('/tf_static', 'tf2_msgs/msg/TFMessage',
                          NS(transforms=[tf(0, 'body', 'base_link', pose(.4, 0., math.pi))]), stamp)
        evaluator.observe('/tf', 'tf2_msgs/msg/TFMessage',
                          NS(transforms=[tf(stamp, 'map', 'camera_init', pose(2., 1., math.pi/2)),
                                         tf(stamp, 'camera_init', 'body', pose(2., 1., 0.))]), stamp)
        evaluator.observe('/localization/pose', 'geometry_msgs/msg/PoseStamped',
                          NS(header=header(stamp), pose=pose(1., 3., math.pi/2)), stamp + 1)
        result = evaluator.summary()
        self.assertAlmostEqual(result['estimated_terminal']['distance_m'], math.hypot(4., .4))
        self.assertEqual(result['estimated_terminal']['goal_map'], [5., 3., 0.])
        self.assertIsNone(result['true_endpoint_error_m'])
        self.assertEqual(result['accuracy_validation'], 'independent_ground_truth_required')

    def test_tf_comparison_pairs_exact_odom_stamp_and_does_not_use_future_correction(self):
        evaluator = self.m.TrialEvaluator()
        evaluator.observe('/tf', 'tf2_msgs/msg/TFMessage', NS(transforms=[
            tf(1_000_000_000, 'camera_init', 'body', pose(1.)),
            tf(2_000_000_000, 'camera_init', 'body', pose(20.)),
            tf(1_000_000_000, 'map', 'camera_init', pose(5.)),
            tf(3_000_000_000, 'map', 'camera_init', pose(50.))]), 1_000_000_000)
        evaluator.observe('/localization/pose', 'geometry_msgs/msg/PoseStamped',
                          NS(header=header(1_000_000_000), pose=pose(6.)), 1_100_000_000)
        result = evaluator.summary()['tf_pose_consistency']
        self.assertEqual(result['exact_odom_pairs'], 0)
        self.assertEqual(result['exact_raw_tf_pairs'], 1)
        self.assertAlmostEqual(result['raw_tf_translation_residual_m']['max'], 0)

    def test_actual_odom_is_checked_and_later_raw_tf_cannot_rewrite_earlier_pose(self):
        e = self.m.TrialEvaluator()
        e.observe('/Odometry', '', NS(header=header(10, 'camera_init'),
                  child_frame_id='body', pose=NS(pose=pose(999.))), 10)
        e.observe('/tf', '', NS(transforms=[tf(10, 'map', 'camera_init', pose(5.))]), 10)
        e.observe('/localization/pose', '', NS(header=header(10), pose=pose(6.)), 20)
        e.observe('/tf', '', NS(transforms=[tf(10, 'camera_init', 'body', pose(1.))]), 30)
        r = e.summary()['tf_pose_consistency']
        self.assertEqual(r['exact_odom_pairs'], 1)
        self.assertAlmostEqual(r['translation_residual_m']['max'], 998.)
        self.assertEqual(r['exact_raw_tf_pairs'], 0)

    def test_nonfinite_velocity_and_unknown_motion_cannot_prove_safe_stop(self):
        for state, bad in [(1, math.nan), (255, 0.)]:
            e = self.m.TrialEvaluator()
            e.observe('/localization/status', '', NS(state=state, state_label='LOCALIZED' if state == 1 else 'UNKNOWN', reason=''), 1)
            e.observe('/cmd_vel_guarded', '', NS(linear=NS(x=.1, y=0.), angular=NS(z=0.)), 2)
            e.observe('/localization/status', '', NS(state=3, state_label='LOST', reason='fault'), 3)
            e.observe('/cmd_vel_guarded', '', NS(linear=NS(x=bad, y=0.), angular=NS(z=0.)), 4)
            r = e.summary()
            self.assertIsNot(r['guarded_velocity']['fault_stop_observed'], True)
            self.assertTrue(r['invalid_messages'])

    def test_known_moving_localized_then_fault_zero_is_observed_stop(self):
        e = self.m.TrialEvaluator()
        for stamp, state, speed in [(1, 1, .1), (3, 3, 0.)]:
            e.observe('/localization/status', '', NS(state=state, state_label='LOCALIZED' if state == 1 else 'LOST', reason=''), stamp)
            e.observe('/cmd_vel_guarded', '', NS(linear=NS(x=speed, y=0.), angular=NS(z=0.)), stamp+1)
        self.assertTrue(e.summary()['guarded_velocity']['fault_stop_observed'])

    def test_unknown_intermediate_state_or_already_stopped_does_not_prove_fault_stop(self):
        for intermediate in [(255, .1), (1, 0.)]:
            e = self.m.TrialEvaluator()
            for stamp, state, speed in [(1, 1, .1), (3, *intermediate), (5, 3, 0.)]:
                e.observe('/localization/status', '', NS(state=state, state_label=str(state), reason=''), stamp)
                e.observe('/cmd_vel_guarded', '', NS(linear=NS(x=speed, y=0.), angular=NS(z=0.)), stamp+1)
            self.assertIsNone(e.summary()['guarded_velocity']['fault_stop_observed'])

    def test_no_fit_infinite_sentinel_is_json_safe_and_keeps_invalid_stat(self):
        import json
        e = self.m.TrialEvaluator()
        e.observe('/localization/registration_status', '', NS(stamp=header(1).stamp,
                  accepted=False, stage='localization', reason='non_monotonic_frontend_frame',
                  fitness=math.inf, overlap=0., correspondence_count=0, elapsed_ms=0.), 2)
        json.dumps(self.m.json_safe(e.registrations), allow_nan=False)
        self.assertIsNone(self.m.json_safe(e.registrations)[0]['fitness'])
        self.assertEqual(e.summary()['normal_registration']['fitness']['invalid_count'], 1)
        self.assertEqual(e.registrations[0]['nonfinite_fields'], {'fitness': 'inf'})

    def test_no_nonzero_command_after_fault_is_counted_by_receipt_state(self):
        evaluator = self.m.TrialEvaluator()
        evaluator.observe('/localization/status', 'a2w_fastlio_msgs/msg/LocalizationStatus',
                          NS(state=3, state_label='LOST', reason='fault', stamp=header(1).stamp), 100)
        evaluator.observe('/cmd_vel_guarded', 'geometry_msgs/msg/Twist',
                          NS(linear=NS(x=.1, y=0., z=0.), angular=NS(x=0., y=0., z=0.)), 110)
        result = evaluator.summary()['guarded_velocity']
        self.assertEqual(result['fault_state_nonzero_count'], 1)
        self.assertFalse(result['fault_stop_observed'])

    def test_stationary_zero_bag_does_not_claim_moving_fault_stop_passed(self):
        evaluator = self.m.TrialEvaluator()
        evaluator.observe('/cmd_vel_guarded', 'geometry_msgs/msg/Twist',
                          NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.)), 100)
        result = evaluator.summary()['guarded_velocity']
        self.assertIsNone(result['fault_stop_observed'])

    def test_gicp_time_does_not_equal_pipeline_or_end_to_end_delay(self):
        evaluator = self.m.TrialEvaluator()
        evaluator.observe('/rosout', 'rcl_interfaces/msg/Log', NS(msg=
            'offline_match_stages source_stamp_ns=1000000000 wait_ms=80.0 preprocess_ms=0.1 '
            'fine_ms=20.0 worker_ms=40.0 result_wait_ms=30.0 end_to_end_status_ms=150.0 '
            'accepted=1 mode=fine_only reason=match_accepted'), 1_200_000_000)
        result = evaluator.summary()['match_stages_ms']
        self.assertEqual(result['fine_ms']['p50'], 20)
        self.assertEqual(result['worker_ms']['p50'], 40)
        self.assertEqual(result['end_to_end_status_ms']['p50'], 150)

    def test_empty_or_partial_data_never_claims_complete_capture(self):
        result = self.m.TrialEvaluator().summary()
        self.assertFalse(result['capture_complete'])
        self.assertIn('/cloud_registered_body', result['missing_core_topics'])

    def test_all_topics_with_only_future_map_tf_is_not_complete(self):
        e = self.m.TrialEvaluator()
        e.observe('/Odometry', '', NS(header=header(10, 'camera_init'), child_frame_id='body', pose=NS(pose=pose())), 10)
        for topic, frame in [('/lidar_points', 'lidar'), ('/lidar_imu', 'imu'), ('/cloud_registered_body', 'body')]:
            e.observe(topic, '', NS(header=header(10, frame), width=1, height=1), 10)
        e.observe('/localization/pose', '', NS(header=header(10), pose=pose()), 20)
        e.observe('/localization/status', '', NS(state=1, state_label='LOCALIZED', reason=''), 10)
        e.observe('/localization/registration_status', '', NS(stamp=header(10).stamp, accepted=True,
            stage='localization', reason='', fitness=0., overlap=1., correspondence_count=30, elapsed_ms=1.), 10)
        e.observe('/tf_static', '', NS(transforms=[tf(0, 'body', 'base_link', pose())]), 10)
        e.observe('/tf', '', NS(transforms=[tf(10, 'camera_init', 'body', pose()),
            tf(2_000_000_000, 'map', 'camera_init', pose())]), 10)
        r = e.summary()
        self.assertTrue(r['core_topics_present'])
        self.assertFalse(r['capture_complete'])
        self.assertIn('TF:map=>camera_init', r['missing_required_data'])

    def test_interpolated_percentile_and_invalid_samples_are_explicit(self):
        result = self.m.statistics([1., 3., math.nan, math.inf])
        self.assertEqual(result['p50'], 2.)
        self.assertEqual(result['invalid_count'], 2)

    def test_map_archive_preserves_manifest_clouds_and_rejects_hash_or_path_mismatch(self):
        import hashlib
        import sys
        sys.path.insert(0, str(ROOT/'scripts/diagnostics'))
        import localization_trial as trial
        with tempfile.TemporaryDirectory() as folder:
            src = Path(folder)/'src'; src.mkdir()
            cloud = src/'keyframes/0.pcd'; cloud.parent.mkdir(); cloud.write_bytes(b'real fixture')
            digest = hashlib.sha256(cloud.read_bytes()).hexdigest()
            (src/'manifest.sha256').write_text(digest+'  keyframes/0.pcd\n')
            trial.archive_bundle(src, Path(folder)/'copy')
            cloud.unlink()
            self.assertEqual((Path(folder)/'copy/keyframes/0.pcd').read_bytes(), b'real fixture')
            cloud.write_bytes(b'changed')
            with self.assertRaises(ValueError):
                trial.archive_bundle(src, Path(folder)/'wrong')
            (src/'manifest.sha256').write_text(digest+'  ../escape.pcd\n')
            with self.assertRaises(ValueError):
                trial.archive_bundle(src, Path(folder)/'unsafe')


if __name__ == '__main__':
    unittest.main()
