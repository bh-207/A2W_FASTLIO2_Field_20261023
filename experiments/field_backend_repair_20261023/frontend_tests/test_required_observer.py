"""Exact owner replacement regression; baseline uses its actual declared literal."""
from pathlib import Path
import importlib.util
import json
import os
import re
import subprocess
import tempfile
import sys
import unittest

HERE=Path(__file__).resolve().parent
SOURCE=Path(os.environ.get("A2W_OBSERVER_TEST_SOURCE",str(HERE.parent/"source")))
FRONTEND=SOURCE/"frontend"

class ExactRequiredObserver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary=tempfile.TemporaryDirectory(prefix="required-observer-")
        cls.binary=str(Path(cls.temporary.name)/"test_observer")
        command=["g++","-std=c++17","-Wall","-Wextra","-Werror","-Wno-misleading-indentation","-pthread",
            f"-I{FRONTEND/'include'}",f"-I{FRONTEND/'src'}",str(HERE/"test_required_observer.cpp"),"-o",cls.binary]
        if "required_observer_name(" not in (FRONTEND/"src/startup_trace.hpp").read_text():
            patterns=set(re.findall(r'"(/field_capture_observer_[^"\\]*)"',(FRONTEND/"src/laserMapping.cpp").read_text()))
            if patterns!={"/field_capture_observer_*"}:raise AssertionError("unrecognized baseline required observer policy")
            command.insert(1,'-DLEGACY_REQUIRED_OBSERVER_PATTERN="'+next(iter(patterns))+'"')
        subprocess.run(command,check=True)

    @classmethod
    def tearDownClass(cls):cls.temporary.cleanup()

    def execute(self,case):
        result=subprocess.run([self.binary,case],text=True,capture_output=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_missing_owner_cannot_be_replaced_by_prefix_debug_node(self):self.execute("replacement")
    def test_present_owner_allows_arbitrary_extra_debug_nodes(self):self.execute("extra_debug")
    def test_exact_canonical_observer_configuration_is_required(self):self.execute("configuration")

class RequiredObserverSessionEnvironment(unittest.TestCase):
    def api(self):
        sys.dont_write_bytecode=True
        sys.path.insert(0,str(HERE.parent/"source/scripts"))
        path=SOURCE/"experiments/field_fixed_20261023/field.py"
        spec=importlib.util.spec_from_file_location("required_observer_field_test",path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_session_uuid_overrides_any_inherited_observer_identity(self):
        module=self.api()
        previous=os.environ.get("A2W_REQUIRED_OBSERVER_NAME")
        self.addCleanup(lambda: os.environ.pop("A2W_REQUIRED_OBSERVER_NAME",None) if previous is None
            else os.environ.__setitem__("A2W_REQUIRED_OBSERVER_NAME",previous))
        os.environ["A2W_REQUIRED_OBSERVER_NAME"]="/field_capture_observer_ffeeddccbbaa99887766554433221100"
        with tempfile.TemporaryDirectory(prefix="observer-env-") as directory:
            session=Path(directory)
            (session/"session.json").write_text(json.dumps(dict(uuid="00112233445566778899aabbccddeeff",field_mode="offline-test")))
            env=module.session_env(session)
            self.assertEqual(env.get("A2W_REQUIRED_OBSERVER_NAME"),"/field_capture_observer_00112233445566778899aabbccddeeff")

    def test_malformed_session_uuid_cannot_launch_ambiguous_observer_identity(self):
        module=self.api()
        with tempfile.TemporaryDirectory(prefix="observer-bad-env-") as directory:
            session=Path(directory)
            (session/"session.json").write_text(json.dumps(dict(uuid="*",field_mode="offline-test")))
            with self.assertRaises(ValueError):module.session_env(session)

if __name__=="__main__":unittest.main()
