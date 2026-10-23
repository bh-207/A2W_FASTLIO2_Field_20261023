#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src" / "a2w_navigation"


class NavigationRepositoryContractTests(unittest.TestCase):
    def test_navigation_has_only_msgs_as_an_internal_dependency(self):
        packages = {}
        for package_xml in (REPOSITORY_ROOT / "src").glob("*/package.xml"):
            root = ET.parse(package_xml).getroot()
            packages[root.findtext("name")] = {
                element.text
                for tag in ("depend", "build_depend", "exec_depend")
                for element in root.findall(tag)
                if element.text
            }
        internal = set(packages)
        self.assertEqual(packages["a2w_navigation"] & internal, {"a2w_fastlio_msgs"})

    def test_navigation_tree_contains_no_legacy_or_machine_paths(self):
        forbidden = (
            "/home/" "dndx", "Go2_Nav", "go2_driver", "go2_description",
            # base_link is provided by the current A2W model's body extrinsic.
            "livox_ros_driver2", "livox_frame", "base_footprint",
            "icp_registration", "nav2_amcl",
        )
        violations = []
        for path in PACKAGE_ROOT.rglob("*"):
            if (path.is_file() and "test" not in path.parts and
                    "__pycache__" not in path.parts):
                text = path.read_text(encoding="utf-8")
                for token in forbidden:
                    if token in text:
                        violations.append(f"{path.relative_to(REPOSITORY_ROOT)}:{token}")
        self.assertEqual(violations, [])

    def test_navigation_does_not_modify_existing_slam_packages(self):
        launch_source = (PACKAGE_ROOT / "launch" / "nav2_core.launch.py").read_text(
            encoding="utf-8")
        self.assertNotIn("a2w_fastlio_mapping", launch_source)
        self.assertNotIn("a2w_fastlio_localization", launch_source)
        self.assertNotIn("fast_lio", launch_source)


if __name__ == "__main__":
    unittest.main()
