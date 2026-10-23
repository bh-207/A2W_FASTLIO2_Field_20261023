#!/usr/bin/env python3

import pathlib
import math
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
MODEL_PATH = PACKAGE_ROOT / "urdf" / "a2w_navigation.urdf"


class A2WDescriptionTests(unittest.TestCase):
    def test_is_a_ros2_ament_description_package(self):
        manifest = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        self.assertEqual(manifest.findtext("name"), "a2w_description")
        self.assertEqual(manifest.findtext("export/build_type"), "ament_cmake")
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("project(a2w_description)", cmake)
        self.assertIn("ament_package()", cmake)
        self.assertIn("DIRECTORY meshes", cmake)
        self.assertIn("FILES urdf/a2w_navigation.urdf", cmake)
        self.assertNotIn("DIRECTORY launch", cmake)

    def test_navigation_model_has_body_as_its_single_root(self):
        root = ET.parse(MODEL_PATH).getroot()
        links = {link.attrib["name"] for link in root.findall("link")}
        children = {
            joint.find("child").attrib["link"] for joint in root.findall("joint")
        }
        roots = links - children
        self.assertEqual(roots, {"body"})
        self.assertNotIn("world", links)
        self.assertIn("base_link", links)
        bridge = next(
            joint for joint in root.findall("joint")
            if joint.attrib["name"] == "body_to_base_link")
        self.assertEqual(bridge.attrib["type"], "fixed")
        self.assertEqual(bridge.find("parent").attrib["link"], "body")
        self.assertEqual(bridge.find("child").attrib["link"], "base_link")
        origin = bridge.find("origin")
        xyz = [float(v) for v in origin.attrib["xyz"].split()]
        self.assertEqual(xyz, [0.33767, 0.0, -0.08134])

    def test_base_axes_match_measured_front_left_and_counterclockwise_motion(self):
        root = ET.parse(MODEL_PATH).getroot()
        origin = root.find("joint[@name='body_to_base_link']/origin")
        roll, pitch, yaw = map(float, origin.attrib["rpy"].split())
        self.assertAlmostEqual(roll, 0.)
        self.assertAlmostEqual(pitch, 0.)
        # URDF rotates child (base) vectors into the parent (body).
        # Observed physical forward is -body X; left is -body Y; yaw sign stays positive.
        c, s = math.cos(yaw), math.sin(yaw)
        forward, left = (c, s, 0.), (-s, c, 0.)
        for actual, expected in zip(forward, (-1., 0., 0.)):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(left, (0., -1., 0.)):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertAlmostEqual(forward[0]*left[1] - forward[1]*left[0], 1.)

    def test_model_keeps_real_a2w_wheel_leg_tree_without_fake_sensors(self):
        root = ET.parse(MODEL_PATH).getroot()
        links = {link.attrib["name"] for link in root.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        self.assertTrue({
            "body", "base_link", "front_bumper", "rear_bumper",
            "FL_hip", "FL_thigh", "FL_calf", "FL_wheel",
            "FR_hip", "FR_thigh", "FR_calf", "FR_wheel",
            "RL_hip", "RL_thigh", "RL_calf", "RL_wheel",
            "RR_hip", "RR_thigh", "RR_calf", "RR_wheel",
        } <= links)
        self.assertNotIn("jt128_link", links)
        self.assertNotIn("imu_link", links)
        for name in ("FL_wheel_joint", "FR_wheel_joint",
                     "RL_wheel_joint", "RR_wheel_joint"):
            self.assertEqual(joints[name].attrib["type"], "continuous")

    def test_meshes_are_portable_package_uris(self):
        root = ET.parse(MODEL_PATH).getroot()
        filenames = [mesh.attrib["filename"] for mesh in root.findall(".//mesh")]
        self.assertTrue(filenames)
        for filename in filenames:
            self.assertTrue(filename.startswith("package://a2w_description/meshes/"))
            self.assertNotIn("/home/", filename)


if __name__ == "__main__":
    unittest.main()
