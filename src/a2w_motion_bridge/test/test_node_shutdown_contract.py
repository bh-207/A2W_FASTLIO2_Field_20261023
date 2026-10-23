#!/usr/bin/env python3

import pathlib
import unittest


NODE_PATH = pathlib.Path(__file__).resolve().parents[1] / "a2w_motion_bridge" / "node.py"


class MotionBridgeShutdownContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NODE_PATH.read_text(encoding="utf-8")
        compile(cls.source, str(NODE_PATH), "exec")

    def test_sigint_is_owned_until_stopmove_has_been_published(self):
        self.assertIn("SignalHandlerOptions.NO", self.source)
        self.assertIn("except KeyboardInterrupt:", self.source)
        self.assertLess(
            self.source.index("node.stop_now()"),
            self.source.index("rclpy.shutdown()"),
        )


if __name__ == "__main__":
    unittest.main()
