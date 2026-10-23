#!/usr/bin/env python3

import json
import math
import unittest

from a2w_motion_bridge.motion_policy import (
    MotionPolicy,
    MotionPolicyConfig,
    motion_profile_requests,
)


class MotionPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = MotionPolicy(MotionPolicyConfig(
            max_forward=0.15, max_reverse=0.05, max_lateral=0.15,
            max_yaw=0.50, command_timeout_s=0.30))

    def test_starts_disabled_and_uses_verified_stopmove_api(self):
        self.assertFalse(self.policy.enabled)
        result = self.policy.command(0.1, 0.0, 0.0, now_s=1.0)
        self.assertEqual(result.api_id, 1003)
        self.assertEqual(result.parameter, "")

    def test_enabled_command_saturates_omnidirectional_motion(self):
        self.policy.set_enabled(True)
        result = self.policy.command(0.7, 0.4, -0.8, now_s=2.0)
        self.assertEqual(result.api_id, 1008)
        self.assertEqual(json.loads(result.parameter), {"x": 0.15, "y": 0.15, "z": -0.50})

    def test_default_policy_maps_bounded_lateral_velocity_to_unitree_y(self):
        policy = MotionPolicy(MotionPolicyConfig())
        policy.set_enabled(True)
        result = policy.command(0.1, -0.2, 0.8, now_s=2.0)
        self.assertEqual(
            json.loads(result.parameter),
            {"x": 0.1, "y": -0.15, "z": 0.5})

    def test_reverse_limit_is_independent(self):
        self.policy.set_enabled(True)
        result = self.policy.command(-0.7, 0.0, 0.0, now_s=2.0)
        self.assertEqual(json.loads(result.parameter)["x"], -0.05)

    def test_non_finite_input_closes_gate_and_returns_stopmove(self):
        self.policy.set_enabled(True)
        result = self.policy.command(math.nan, 0.0, 0.0, now_s=2.0)
        self.assertEqual(result.api_id, 1003)
        self.assertFalse(self.policy.enabled)
        self.assertEqual(self.policy.reason, "non_finite_command")

    def test_timeout_closes_gate_once_and_requires_reenable(self):
        self.policy.set_enabled(True)
        self.policy.command(0.1, 0.0, 0.0, now_s=2.0)
        self.assertIsNone(self.policy.watchdog(now_s=2.29))
        result = self.policy.watchdog(now_s=2.31)
        self.assertEqual(result.api_id, 1003)
        self.assertFalse(self.policy.enabled)
        self.assertEqual(self.policy.reason, "command_timeout")
        self.assertIsNone(self.policy.watchdog(now_s=3.0))

    def test_zero_command_stops_without_disabling_prepared_output(self):
        self.policy.set_enabled(True)
        result = self.policy.command(0.0, 0.0, 0.0, now_s=2.0)
        self.assertEqual(result.api_id, 1003)
        self.assertIsNone(self.policy.watchdog(now_s=3.0))
        self.assertTrue(self.policy.enabled)
        self.assertEqual(self.policy.reason, "zero_command")

    def test_disable_and_shutdown_return_stopmove(self):
        self.policy.set_enabled(True)
        self.assertEqual(self.policy.set_enabled(False).api_id, 1003)
        self.policy.set_enabled(True)
        self.assertEqual(self.policy.shutdown().api_id, 1003)
        self.assertFalse(self.policy.enabled)

    def test_explicit_start_enabled_configuration_prepares_hardware_output(self):
        self.assertIn("start_enabled", MotionPolicyConfig.__dataclass_fields__)
        policy = MotionPolicy(MotionPolicyConfig(start_enabled=True))
        self.assertTrue(policy.enabled)
        self.assertEqual(policy.reason, "startup_enabled")

    def test_legacy_minimum_parameters_do_not_amplify_small_commands(self):
        policy = MotionPolicy(MotionPolicyConfig(
            max_forward=0.15,
            max_reverse=0.05,
            max_lateral=0.15,
            max_yaw=0.50,
            min_effective_forward=0.04,
            min_effective_lateral=0.05,
            min_effective_yaw=0.08,
        ))
        policy.set_enabled(True)

        result = policy.command(0.01, -0.02, 0.03, now_s=2.0)

        self.assertEqual(
            json.loads(result.parameter),
            {"x": 0.01, "y": -0.02, "z": 0.03},
        )

    def test_terminal_lateral_boost_requires_fresh_signed_error(self):
        policy = MotionPolicy(MotionPolicyConfig(
            max_lateral=0.30, min_effective_lateral=0.30,
            start_enabled=True))
        raw = policy.command(0.04, -0.08, 0.03, now_s=1.0)
        self.assertEqual(json.loads(raw.parameter), {"x": 0.04, "y": -0.08, "z": 0.03})
        boosted = policy.command(0.04, -0.08, 0.03, now_s=1.1,
                                 terminal_lateral_error_m=-0.35,
                                 terminal_distance_m=0.4)
        self.assertEqual(json.loads(boosted.parameter), {"x": 0.04, "y": -0.30, "z": 0.03})
        wrong_way = policy.command(0.04, 0.08, 0.03, now_s=1.2,
                                   terminal_lateral_error_m=-0.35,
                                   terminal_distance_m=0.4)
        self.assertEqual(json.loads(wrong_way.parameter)["y"], 0.08)
        near = policy.command(0.04, -0.08, 0.03, now_s=1.3,
                              terminal_lateral_error_m=-0.04,
                              terminal_distance_m=0.4)
        self.assertEqual(json.loads(near.parameter)["y"], 0.0)
    def test_logged_navigation_corrections_are_suppressed_not_amplified(self):
        policy = MotionPolicy(MotionPolicyConfig(start_enabled=True))
        result = policy.command(0.149, -0.003, -0.014, now_s=1.0)
        self.assertEqual(json.loads(result.parameter), {"x": 0.149, "y": 0.0, "z": 0.0})

    def test_all_deadband_command_stops_immediately(self):
        policy = MotionPolicy(MotionPolicyConfig(start_enabled=True))
        policy.command(0.1, 0.1, 0.2, now_s=1.0)
        self.assertEqual(policy.command(0.002, -0.003, 0.01, now_s=1.05).api_id, 1003)
        self.assertIsNone(policy.watchdog(now_s=3.0))

    def test_deadband_boundaries_keep_original_sign_and_magnitude(self):
        policy = MotionPolicy(MotionPolicyConfig(start_enabled=True))
        result = policy.command(-0.01, 0.01, -0.02, now_s=1.0)
        self.assertEqual(json.loads(result.parameter), {"x": -0.01, "y": 0.01, "z": -0.02})

    def test_deadbands_can_be_tuned_without_changing_velocity_limits(self):
        policy = MotionPolicy(MotionPolicyConfig(start_enabled=True,
            deadband_forward=0.005, deadband_lateral=0.02, deadband_yaw=0.04))
        result = policy.command(0.006, 0.015, -0.03, now_s=1.0)
        self.assertEqual(json.loads(result.parameter), {"x": 0.006, "y": 0.0, "z": 0.0})

    def test_invalid_deadband_is_rejected(self):
        for value in (-0.01, math.nan, math.inf, 0.9):
            with self.assertRaises(ValueError):
                MotionPolicy(MotionPolicyConfig(deadband_lateral=value))

    def test_effective_minimum_keeps_zero_zero_and_does_not_reduce_larger_commands(self):
        policy = MotionPolicy(MotionPolicyConfig(
            min_effective_forward=0.04,
            min_effective_lateral=0.05,
            min_effective_yaw=0.08,
        ))
        policy.set_enabled(True)
        self.assertEqual(policy.command(0.0, 0.0, 0.0, now_s=2.0).api_id, 1003)

        result = policy.command(0.10, -0.12, 0.20, now_s=2.1)

        self.assertEqual(
            json.loads(result.parameter),
            {"x": 0.10, "y": -0.12, "z": 0.20},
        )

    def test_effective_minimum_must_fit_both_velocity_directions(self):
        with self.assertRaisesRegex(ValueError, "effective minimum"):
            MotionPolicy(MotionPolicyConfig(
                max_forward=0.15,
                max_reverse=0.05,
                min_effective_forward=0.06,
            ))

    def test_motion_profile_uses_a2_switch_gait_and_speed_level_apis(self):
        requests = motion_profile_requests(gait_type=0, speed_level=0)

        self.assertEqual(
            [(request.api_id, json.loads(request.parameter)) for request in requests],
            [(1011, {"data": 0}), (1015, {"data": 0})],
        )


if __name__ == "__main__":
    unittest.main()
