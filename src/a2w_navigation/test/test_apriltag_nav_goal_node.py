#!/usr/bin/env python3

import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "a2w_navigation" / "apriltag_nav_goal.py"


class AprilTagNavGoalNodeTests(unittest.TestCase):
    def test_node_is_valid_python_and_does_not_embed_machine_specific_paths(self):
        source = NODE_PATH.read_text(encoding="utf-8")
        compile(source, str(NODE_PATH), "exec")
        self.assertNotIn("/home/" "dndx", source)


if __name__ == "__main__":
    unittest.main()
