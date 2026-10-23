#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "a2w_navigation" / "runtime_monitoring.py"


def load_runtime_monitoring():
    if not MODULE_PATH.is_file():
        raise AssertionError("runtime_monitoring.py has not been implemented")
    spec = importlib.util.spec_from_file_location("runtime_monitoring", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RuntimeMonitoringTests(unittest.TestCase):
    def test_nav2_requires_motion_authorization_and_recovery_cannot_reuse_it(self):
        module = load_runtime_monitoring()
        policy = module.Nav2ActivationPolicy(enabled=True)
        policy.observe_localization(module.LOCALIZED_STATE)
        policy.observe_pose()
        self.assertIsNone(policy.take_lifecycle_request())
        policy.observe_motion_authorization(True)
        self.assertEqual(policy.take_lifecycle_request(), module.STARTUP_REQUEST)
        policy.complete_lifecycle_request(True)
        policy.observe_motion_authorization(False)
        self.assertFalse(policy.ready)
        self.assertEqual(policy.take_lifecycle_request(), module.PAUSE_REQUEST)
        policy.complete_lifecycle_request(True)
        self.assertIsNone(policy.take_lifecycle_request())
        policy.observe_motion_authorization(True)
        self.assertEqual(policy.take_lifecycle_request(), module.RESUME_REQUEST)

    def test_nav2_start_is_requested_only_after_localized_pose_is_available(self):
        module = load_runtime_monitoring()
        policy = module.Nav2ActivationPolicy(enabled=True)
        policy.observe_motion_authorization(True)

        policy.observe_localization(module.LOCALIZED_STATE)
        self.assertIsNone(policy.take_lifecycle_request())

        policy.observe_pose()
        self.assertEqual(policy.take_lifecycle_request(), module.STARTUP_REQUEST)
        self.assertIsNone(policy.take_lifecycle_request())

    def test_authorization_revoked_during_activation_must_pause_before_resume(self):
        module = load_runtime_monitoring()
        policy = module.Nav2ActivationPolicy(enabled=True)
        policy.observe_localization(module.LOCALIZED_STATE)
        policy.observe_pose()
        policy.observe_motion_authorization(True)
        self.assertEqual(policy.take_lifecycle_request(), module.STARTUP_REQUEST)
        policy.observe_motion_authorization(False)
        policy.observe_motion_authorization(True)
        policy.complete_lifecycle_request(True)
        self.assertFalse(policy.ready)
        self.assertEqual(policy.take_lifecycle_request(), module.PAUSE_REQUEST)
        policy.complete_lifecycle_request(True)
        self.assertEqual(policy.take_lifecycle_request(), module.RESUME_REQUEST)
        policy.complete_lifecycle_request(True)
        self.assertTrue(policy.ready)

    def test_failed_nav2_start_can_retry_but_success_is_latched(self):
        module = load_runtime_monitoring()
        policy = module.Nav2ActivationPolicy(enabled=True)
        policy.observe_motion_authorization(True)
        policy.observe_localization(module.LOCALIZED_STATE)
        policy.observe_pose()

        self.assertEqual(policy.take_lifecycle_request(), module.STARTUP_REQUEST)
        policy.complete_lifecycle_request(False)
        self.assertEqual(policy.take_lifecycle_request(), module.STARTUP_REQUEST)
        policy.complete_lifecycle_request(True)
        self.assertTrue(policy.ready)
        self.assertIsNone(policy.take_lifecycle_request())

    def test_nonlocalized_state_prevents_a_not_yet_started_nav2_request(self):
        module = load_runtime_monitoring()
        policy = module.Nav2ActivationPolicy(enabled=True)
        policy.observe_motion_authorization(True)
        policy.observe_pose()
        policy.observe_localization(module.DEGRADED_STATE)
        self.assertIsNone(policy.take_lifecycle_request())
        policy.observe_localization(module.LOCALIZED_STATE)
        self.assertIsNone(policy.take_lifecycle_request())
        policy.observe_pose()
        self.assertEqual(policy.take_lifecycle_request(), module.STARTUP_REQUEST)

    def test_degraded_state_pauses_active_nav2_before_it_can_resume(self):
        module = load_runtime_monitoring()
        policy = module.Nav2ActivationPolicy(enabled=True)
        policy.observe_motion_authorization(True)
        policy.observe_localization(module.LOCALIZED_STATE)
        policy.observe_pose()
        self.assertEqual(policy.take_lifecycle_request(), module.STARTUP_REQUEST)
        policy.complete_lifecycle_request(True)
        self.assertTrue(policy.ready)

        policy.observe_localization(module.DEGRADED_STATE)
        self.assertFalse(policy.ready)
        self.assertEqual(policy.take_lifecycle_request(), module.PAUSE_REQUEST)
        self.assertIsNone(policy.take_lifecycle_request())
        policy.complete_lifecycle_request(True)

        policy.observe_localization(module.LOCALIZED_STATE)
        self.assertIsNone(policy.take_lifecycle_request())
        policy.observe_pose()
        self.assertEqual(policy.take_lifecycle_request(), module.RESUME_REQUEST)
        policy.complete_lifecycle_request(True)
        self.assertTrue(policy.ready)

    def test_transient_degraded_state_cannot_skip_the_required_pause(self):
        module = load_runtime_monitoring()
        policy = module.Nav2ActivationPolicy(enabled=True)
        policy.observe_motion_authorization(True)
        policy.observe_localization(module.LOCALIZED_STATE)
        policy.observe_pose()
        self.assertEqual(policy.take_lifecycle_request(), module.STARTUP_REQUEST)
        policy.complete_lifecycle_request(True)

        policy.observe_localization(module.DEGRADED_STATE)
        policy.observe_localization(module.LOCALIZED_STATE)
        policy.observe_pose()
        self.assertEqual(policy.take_lifecycle_request(), module.PAUSE_REQUEST)
        policy.complete_lifecycle_request(True)
        self.assertEqual(policy.take_lifecycle_request(), module.RESUME_REQUEST)

    def test_costmap_cells_are_split_into_free_inflated_lethal_and_unknown(self):
        module = load_runtime_monitoring()
        counts = module.classify_costmap([-1, 0, 1, 50, 98, 99, 100])
        self.assertEqual(counts.unknown, 1)
        self.assertEqual(counts.free, 1)
        self.assertEqual(counts.inflated, 3)
        self.assertEqual(counts.lethal, 2)

    def test_zero_keepalive_is_sent_only_when_an_armed_command_stream_is_idle(self):
        module = load_runtime_monitoring()
        self.assertFalse(module.should_publish_zero_keepalive(
            armed=False, last_command_s=None, now_s=2.0, interval_s=0.1))
        self.assertTrue(module.should_publish_zero_keepalive(
            armed=True, last_command_s=None, now_s=2.0, interval_s=0.1))
        self.assertFalse(module.should_publish_zero_keepalive(
            armed=True, last_command_s=1.95, now_s=2.0, interval_s=0.1))
        self.assertTrue(module.should_publish_zero_keepalive(
            armed=True, last_command_s=1.8, now_s=2.0, interval_s=0.1))


if __name__ == "__main__":
    unittest.main()
