import importlib.util
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('repair_build',ROOT/'build.py');build=importlib.util.module_from_spec(spec);spec.loader.exec_module(build)

class BuildIdentityTest(unittest.TestCase):
    def test_actual_compiler_and_message_inputs_are_hashed(self):
        keys=build.source_hashes()
        for suffix in ['src/a2w_fastlio_common/third_party/Quatro/src/quatro_module.cc',
                       'src/a2w_fastlio_msgs/msg/RegistrationStatus.msg',
                       'src/a2w_fastlio_msgs/srv/SaveMapBundle.srv']:
            path=build.SOURCE/suffix
            if suffix.endswith('quatro_module.cc'):
                matches=list(build.SOURCE.rglob('quatro_module.cc'))
                self.assertTrue(matches);path=matches[0]
            self.assertIn(str(path),keys)
    def test_frontend_only_cannot_bless_changed_mapping(self):
        key=str(build.SOURCE/'src/a2w_fastlio_mapping/src/pose_graph_optimizer.cpp')
        with self.assertRaises(ValueError):build.check_build_coverage({'sources':{key:{'sha256':'old'}}},{key:{'sha256':'new'}},{'frontend'})
    def test_shared_frontend_header_requires_backend_rebuild(self):
        key=str(build.SOURCE/'frontend/src/startup_trace.hpp')
        old={'sources':{key:{'sha256':'old'}}};new={key:{'sha256':'new'}}
        with self.assertRaises(ValueError):build.check_build_coverage(old,new,{'frontend'})
        build.check_build_coverage(old,new,{'frontend','mapping'})
    def test_partial_first_build_has_no_provenance(self):
        with self.assertRaises(ValueError):build.check_build_coverage(None,{}, {'mapping'})
    def test_unchanged_unbuilt_sources_keep_old_binding(self):
        key=str(build.SOURCE/'src/a2w_fastlio_mapping/src/pose_graph_optimizer.cpp')
        current={key:{'sha256':'same'}}
        build.check_build_coverage({'sources':current},current,{'frontend'})

if __name__=='__main__':unittest.main()
