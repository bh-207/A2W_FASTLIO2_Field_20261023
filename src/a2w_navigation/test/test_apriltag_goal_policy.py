#!/usr/bin/env python3

import math
import unittest

import numpy as np

from a2w_navigation.apriltag_goal_policy import (
    StableDetectionGate,
    bgr8_from_buffer,
    compute_nav_goal,
)


def planar_transform(x=0.0, y=0.0, yaw=0.0):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    transform = np.eye(4, dtype=np.float64)
    transform[:2, :2] = [[cosine, -sine], [sine, cosine]]
    transform[0, 3] = x
    transform[1, 3] = y
    return transform


class AprilTagGoalPolicyTests(unittest.TestCase):
    def test_detection_gate_requires_consecutive_good_frames_and_fresh_pose(self):
        gate = StableDetectionGate(required_frames=3, timeout_s=0.5)
        pose = planar_transform(0.5, 0.0, 0.0)
        gate.accept(pose, stamp_s=1.0)
        gate.accept(pose, stamp_s=1.1)
        self.assertEqual(gate.current(now_s=1.2), (None, "pose_not_stable"))

        gate.accept(pose, stamp_s=1.2)
        ready, reason = gate.current(now_s=1.3)
        np.testing.assert_array_equal(ready, pose)
        self.assertEqual(reason, "accepted")
        self.assertEqual(gate.current(now_s=1.71), (None, "pose_stale"))

    def test_detection_rejection_resets_consecutive_good_frame_count(self):
        gate = StableDetectionGate(required_frames=2, timeout_s=0.5)
        pose = planar_transform(0.5, 0.0, 0.0)
        gate.accept(pose, stamp_s=1.0)
        gate.reject()
        gate.accept(pose, stamp_s=1.1)

        self.assertEqual(gate.current(now_s=1.2), (None, "pose_not_stable"))

    def test_decodes_bgr8_image_while_discarding_row_padding(self):
        first_row = bytes([1, 2, 3, 4, 5, 6, 99, 99])
        second_row = bytes([7, 8, 9, 10, 11, 12, 88, 88])

        image = bgr8_from_buffer(
            first_row + second_row,
            height=2,
            width=2,
            step=8,
            encoding="bgr8",
        )

        np.testing.assert_array_equal(image, np.array([
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]],
        ], dtype=np.uint8))

    def test_rejects_image_encoding_that_does_not_match_calibrated_bgr_stream(self):
        with self.assertRaisesRegex(ValueError, "bgr8"):
            bgr8_from_buffer(
                bytes(12), height=2, width=2, step=6, encoding="rgb8")

    def test_composes_teach_relative_pose_with_current_map_body_pose(self):
        map_body = planar_transform(10.0, -2.0, math.pi / 2.0)
        tag_body_current = planar_transform()
        tag_body_target = planar_transform(0.5, 0.2, -math.pi / 6.0)

        result = compute_nav_goal(
            map_body,
            tag_body_current,
            tag_body_target,
            min_distance_m=0.05,
            max_distance_m=1.0,
            min_yaw_rad=math.radians(2.0),
            max_yaw_rad=math.radians(90.0),
        )

        self.assertTrue(result.accepted)
        self.assertEqual(result.reason, "accepted")
        self.assertAlmostEqual(result.relative_x, 0.5, places=6)
        self.assertAlmostEqual(result.relative_y, 0.2, places=6)
        self.assertAlmostEqual(result.relative_yaw, -math.pi / 6.0, places=6)
        self.assertAlmostEqual(result.x, 9.8, places=6)
        self.assertAlmostEqual(result.y, -1.5, places=6)
        self.assertAlmostEqual(result.yaw, math.pi / 3.0, places=6)

    def test_rejects_goal_beyond_short_range_limit(self):
        result = compute_nav_goal(
            planar_transform(),
            planar_transform(),
            planar_transform(1.01, 0.0, 0.0),
            min_distance_m=0.05,
            max_distance_m=1.0,
            min_yaw_rad=math.radians(2.0),
            max_yaw_rad=math.radians(90.0),
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "goal_too_far")

    def test_suppresses_goal_when_position_and_yaw_are_already_at_teach_target(self):
        result = compute_nav_goal(
            planar_transform(2.0, 3.0, 0.4),
            planar_transform(),
            planar_transform(0.01, -0.01, math.radians(1.0)),
            min_distance_m=0.05,
            max_distance_m=1.0,
            min_yaw_rad=math.radians(2.0),
            max_yaw_rad=math.radians(90.0),
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "already_at_target")

    def test_allows_rotation_only_goal_when_yaw_error_is_not_small(self):
        result = compute_nav_goal(
            planar_transform(),
            planar_transform(),
            planar_transform(0.01, 0.0, math.radians(20.0)),
            min_distance_m=0.05,
            max_distance_m=1.0,
            min_yaw_rad=math.radians(2.0),
            max_yaw_rad=math.radians(90.0),
        )

        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.yaw, math.radians(20.0), places=6)

    def test_rejects_non_finite_transform(self):
        invalid = planar_transform()
        invalid[0, 3] = math.nan

        with self.assertRaisesRegex(ValueError, "finite"):
            compute_nav_goal(
                invalid,
                planar_transform(),
                planar_transform(0.5, 0.0, 0.0),
                min_distance_m=0.05,
                max_distance_m=1.0,
                min_yaw_rad=math.radians(2.0),
                max_yaw_rad=math.radians(90.0),
            )


if __name__ == "__main__":
    unittest.main()
