"""Offline validator contract: never starts ROS; root runs the actual node."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

PATH=Path(__file__).with_name('frontend_init_v4_runtime_smoke.py')
spec=importlib.util.spec_from_file_location('runtime_smoke',PATH)
smoke=importlib.util.module_from_spec(spec);spec.loader.exec_module(smoke)

class RuntimeSmokeSafetyTest(unittest.TestCase):
    def test_missing_authorization_does_not_start_ros_or_create_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'never_created'
            with self.assertRaisesRegex(ValueError,'explicit --allow-localhost-ros'):
                smoke.run('/missing','/missing',output,170,False)
            self.assertFalse(output.exists())
    def test_requires_caller_localhost_and_matching_nonzero_domain(self):
        with patch.dict(os.environ,{},clear=True):
            with self.assertRaisesRegex(ValueError,'caller must explicitly set'):
                smoke.run('/missing','/missing','/missing',170,True)
        with self.assertRaisesRegex(ValueError,'nonzero unique domain'):
            smoke.run('/missing','/missing','/missing',0,True)
    def test_external_discovery_override_is_rejected_before_any_ros_import(self):
        with patch.dict(os.environ,{'ROS_LOCALHOST_ONLY':'1','ROS_DOMAIN_ID':'170','ROS_DISCOVERY_SERVER':'remote:11811'},clear=True):
            with self.assertRaisesRegex(ValueError,'remove externally configured discovery'):
                smoke.run('/missing','/missing','/missing',170,True)
    def test_missing_real_node_log_is_not_claimed_as_success(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError,'RUNTIME STATE LOG MISSING'):
                smoke.review_outputs(directory,[])

if __name__=='__main__':unittest.main()
