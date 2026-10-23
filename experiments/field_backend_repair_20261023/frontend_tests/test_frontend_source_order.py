"""Narrow integration guards where the full ROS callback requires bag validation."""
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "source/frontend/src/laserMapping.cpp"

class FrontendSourceOrder(unittest.TestCase):
    def test_covariance_is_assigned_before_real_publish(self):
        source = SOURCE.read_text()
        body = source.split("void publish_odometry(", 1)[1].split("void publish_path(", 1)[0]
        self.assertLess(body.index("odomAftMapped.pose.covariance"), body.index("pubOdomAftMapped->publish"))

    def test_signal_handler_only_requests_stop(self):
        source = SOURCE.read_text()
        body = source.split("void SigHandle(", 1)[1].split("inline void dump_lio_state_to_log", 1)[0]
        self.assertNotIn("rclcpp::shutdown", body)
        self.assertNotIn("notify_all", body)
        self.assertNotIn("std::cout", body)

if __name__ == "__main__":
    unittest.main()
