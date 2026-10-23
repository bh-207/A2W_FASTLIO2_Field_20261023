#!/usr/bin/env python3

import unittest

import a2w_navigation.guard_policy as guard_policy
from a2w_navigation.guard_policy import GuardPolicy, GuardPolicyConfig


class GuardPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = GuardPolicy(GuardPolicyConfig(
            status_timeout_s=1.0, max_forward=0.15,
            max_reverse=0.05, max_lateral=0.15, max_yaw=0.50))
        self.policy.update_navigation_ready(True, now_s=0.5)

    def test_starts_closed_and_cannot_arm_without_fresh_localized_status(self):
        self.assertFalse(self.policy.arm(now_s=0.0))
        self.assertEqual(self.policy.filter_command(0.1, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertTrue(self.policy.tick(now_s=2.0).publish_zero)

    def test_only_fresh_localized_state_can_be_explicitly_armed(self):
        event = self.policy.update_status(self.policy.LOCALIZED, now_s=1.0)
        self.assertFalse(event.cancel_goal)
        self.assertTrue(self.policy.arm(now_s=1.1))
        self.assertEqual(
            self.policy.filter_command(0.8, 0.2, -0.8, now_s=1.2),
            (0.15, 0.15, -0.50))

    def test_default_policy_allows_bounded_omnidirectional_motion(self):
        policy = GuardPolicy(GuardPolicyConfig())
        policy.update_navigation_ready(True, now_s=0.5)
        policy.update_status(policy.LOCALIZED, now_s=1.0)
        self.assertTrue(policy.arm(now_s=1.1))
        self.assertEqual(
            policy.filter_command(0.1, -0.2, 0.8, now_s=1.2),
            (0.1, -0.15, 0.50))

    def test_all_nonlocalized_states_close_and_latch_the_gate(self):
        for state in (
                self.policy.INITIALIZING, self.policy.DEGRADED,
                self.policy.LOST, self.policy.RELOCALIZING):
            with self.subTest(state=state):
                self.setUp()
                self.policy.update_status(self.policy.LOCALIZED, now_s=2.0)
                self.assertTrue(self.policy.arm(now_s=2.1))
                event = self.policy.update_status(state, now_s=2.2)
                self.assertTrue(event.publish_zero)
                self.assertFalse(self.policy.armed)
                self.assertTrue(event.cancel_goal)

    def test_localized_recovery_never_rearms_an_old_goal(self):
        self.policy.update_status(self.policy.LOCALIZED, now_s=1.0)
        self.policy.arm(now_s=1.1)
        self.policy.update_status(self.policy.DEGRADED, now_s=1.2)
        self.policy.update_status(self.policy.LOCALIZED, now_s=1.3)
        self.assertFalse(self.policy.armed)
        self.assertEqual(self.policy.filter_command(0.1, 0.0, 0.0, 1.4), (0.0, 0.0, 0.0))

    def test_leaving_localized_cancels_once_even_if_motion_was_not_armed(self):
        self.policy.update_status(self.policy.LOCALIZED, now_s=1.0)

        first = self.policy.update_status(self.policy.DEGRADED, now_s=1.1)
        repeated = self.policy.update_status(self.policy.DEGRADED, now_s=1.2)

        self.assertTrue(first.cancel_goal)
        self.assertFalse(repeated.cancel_goal)

    def test_pose_liveness_keeps_a_localized_gate_open_between_slow_matches(self):
        self.policy.update_status(self.policy.LOCALIZED, now_s=1.0)
        self.assertTrue(self.policy.arm(now_s=1.1))
        self.policy.update_liveness(now_s=1.8)
        event = self.policy.tick(now_s=2.5)
        self.assertFalse(event.publish_zero)
        self.assertTrue(self.policy.armed)

    def test_stale_localization_liveness_closes_gate_and_cancels_goal(self):
        self.policy.update_status(self.policy.LOCALIZED, now_s=1.0)
        self.policy.arm(now_s=1.1)
        event = self.policy.tick(now_s=2.01)
        self.assertTrue(event.publish_zero)
        self.assertTrue(event.cancel_goal)
        self.assertFalse(self.policy.armed)
        self.assertEqual(self.policy.reason, "localization_liveness_stale")

    def test_clock_rollback_closes_gate_cancels_and_requires_new_authorization(self):
        self.policy.update_status(self.policy.LOCALIZED, now_s=10.0)
        self.assertTrue(self.policy.arm(now_s=10.1))

        event = self.policy.tick(now_s=9.0)

        self.assertTrue(event.publish_zero)
        self.assertTrue(event.cancel_goal)
        self.assertFalse(self.policy.armed)
        self.assertFalse(self.policy.motion_authorized)
        self.assertEqual(
            self.policy.filter_command(0.1, 0.0, 0.0, 9.0), (0.0, 0.0, 0.0))
        self.assertFalse(self.policy.tick(now_s=9.1).cancel_goal)
        self.policy.update_liveness(now_s=9.2)
        self.policy.update_navigation_ready(True, now_s=9.3)
        self.assertFalse(self.policy.armed)
        self.assertFalse(self.policy.arm(now_s=9.4))
        self.policy.update_navigation_ready(False, now_s=9.5)
        self.assertTrue(self.policy.arm(now_s=9.6))
        self.policy.update_navigation_ready(True, now_s=9.7)
        self.assertEqual(
            self.policy.filter_command(0.1, 0.0, 0.0, 9.8), (0.1, 0.0, 0.0))

    def test_non_finite_command_closes_gate(self):
        self.policy.update_status(self.policy.LOCALIZED, now_s=1.0)
        self.policy.arm(now_s=1.1)
        self.assertEqual(
            self.policy.filter_command(float("inf"), 0.0, 0.0, 1.2),
            (0.0, 0.0, 0.0))
        self.assertFalse(self.policy.armed)

    def test_auto_arm_is_used_only_for_the_first_localized_transition(self):
        self.assertIn(
            "auto_arm_on_first_localized",
            GuardPolicyConfig.__dataclass_fields__)
        policy = GuardPolicy(GuardPolicyConfig(auto_arm_on_first_localized=True))
        policy.update_navigation_ready(True, now_s=0.5)

        policy.update_status(policy.INITIALIZING, now_s=1.0)
        self.assertFalse(policy.armed)
        policy.update_status(policy.LOCALIZED, now_s=1.1)
        self.assertFalse(policy.armed)
        policy.update_liveness(now_s=1.15)
        self.assertTrue(policy.armed)

        policy.update_status(policy.DEGRADED, now_s=1.2)
        self.assertFalse(policy.armed)
        policy.update_status(policy.LOCALIZED, now_s=1.3)
        self.assertFalse(policy.armed)

    def test_readiness_latch_reports_only_ready_and_paused_transitions(self):
        self.assertTrue(hasattr(guard_policy, "NavigationReadinessLatch"))
        latch = guard_policy.NavigationReadinessLatch()

        self.assertIsNone(latch.update(
            gate_armed=False, pose_available=False, hardware_output_enabled=False))
        self.assertIsNone(latch.update(
            gate_armed=True, pose_available=True, hardware_output_enabled=False))
        self.assertTrue(latch.update(
            gate_armed=True, pose_available=True, hardware_output_enabled=True))
        self.assertIsNone(latch.update(
            gate_armed=True, pose_available=True, hardware_output_enabled=True))
        self.assertFalse(latch.update(
            gate_armed=False, pose_available=True, hardware_output_enabled=True))
        self.assertIsNone(latch.update(
            gate_armed=False, pose_available=True, hardware_output_enabled=True))

    def test_auto_arm_waits_for_nav2_ready_even_with_fresh_localized_pose(self):
        policy = GuardPolicy(GuardPolicyConfig(auto_arm_on_first_localized=True))
        policy.update_status(policy.LOCALIZED, now_s=1.0)
        policy.update_liveness(now_s=1.1)
        self.assertFalse(policy.armed)
        self.assertEqual(policy.reason, "localized_waiting_for_nav2")

        event = policy.update_navigation_ready(True, now_s=1.15)
        self.assertFalse(event.publish_zero)
        self.assertTrue(policy.armed)

    def test_nav2_not_ready_closes_an_armed_gate(self):
        self.policy.update_status(self.policy.LOCALIZED, now_s=1.0)
        self.assertTrue(self.policy.arm(now_s=1.1))
        event = self.policy.update_navigation_ready(False, now_s=1.2)
        self.assertTrue(event.publish_zero)
        self.assertFalse(self.policy.armed)
        self.assertEqual(self.policy.reason, "nav2_not_ready")

    def test_authorization_can_precede_nav2_resume_but_velocity_stays_zero(self):
        policy = GuardPolicy(GuardPolicyConfig())
        policy.update_status(policy.LOCALIZED, 1.0)
        policy.update_liveness(1.1)
        self.assertTrue(policy.arm(1.2))
        self.assertTrue(policy.motion_authorized)
        self.assertFalse(policy.armed)
        self.assertEqual(policy.filter_command(.1, 0., .3, 1.3), (0., 0., 0.))
        policy.update_navigation_ready(True, 1.4)
        self.assertTrue(policy.armed)
        policy.update_status(policy.DEGRADED, 1.5)
        self.assertFalse(policy.motion_authorized)
        policy.update_status(policy.LOCALIZED, 1.6)
        policy.update_liveness(1.7)
        policy.update_navigation_ready(True, 1.8)
        self.assertFalse(policy.armed)

    def test_initial_auto_authorization_does_not_require_nav2_already_active(self):
        policy = GuardPolicy(GuardPolicyConfig(auto_arm_on_first_localized=True))
        policy.update_status(policy.LOCALIZED, 1.0)
        policy.update_liveness(1.1)
        self.assertTrue(policy.motion_authorized)
        self.assertFalse(policy.armed)
        policy.disarm()
        policy.update_liveness(1.2)
        self.assertFalse(policy.motion_authorized)

    def test_stale_pose_revokes_pending_authorization(self):
        policy = GuardPolicy(GuardPolicyConfig())
        policy.update_status(policy.LOCALIZED, 1.0)
        policy.arm(1.1)
        policy.tick(2.1)
        policy.update_navigation_ready(True, 2.2)
        self.assertFalse(policy.motion_authorized)
        self.assertFalse(policy.armed)

    def test_cannot_reenable_before_nav2_acknowledges_stop(self):
        self.policy.update_status(self.policy.LOCALIZED, 1.0)
        self.policy.arm(1.1)
        self.policy.disarm()
        self.assertFalse(self.policy.arm(1.2))
        self.assertFalse(self.policy.armed)
        self.policy.update_navigation_ready(False, 1.3)
        self.assertTrue(self.policy.arm(1.4))
        self.assertFalse(self.policy.armed)
        self.policy.update_navigation_ready(True, 1.5)
        self.assertTrue(self.policy.armed)


if __name__ == "__main__":
    unittest.main()
