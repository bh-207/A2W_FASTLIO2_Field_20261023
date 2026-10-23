import importlib.util
from pathlib import Path
import unittest
import subprocess
import json
import tempfile

MODULE_PATH=Path(__file__).parents[1]/'scripts/controlled_mapping_build.py'

class ControlledBuildTests(unittest.TestCase):
    def module(self):
        self.assertTrue(MODULE_PATH.exists(),'controlled isolated build entry is missing')
        spec=importlib.util.spec_from_file_location('controlled_build',MODULE_PATH)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_plan_is_separate_from_production_and_compiles_current_patch(self):
        module=self.module();repo=Path('/tmp/repository')
        plan=module.build_plan(repo,repo/'.controlled_mapping')
        joined=str(plan)
        self.assertIn('frontend_init_v4_tools.py',joined)
        self.assertIn('backend_install',joined)
        self.assertIn('a2w_fastlio_mapping',joined)
        self.assertNotIn(str(repo/'install')+'/',joined)
        self.assertNotIn('install/setup.bash',joined)
        self.assertFalse(any('git'==part for cmd in plan for part in cmd))

    def test_production_and_bundle_roots_cannot_be_build_outputs(self):
        module=self.module();repo=Path('/tmp/repository')
        for output in [repo,repo/'install',repo/'build',repo/'maps'/'something']:
            with self.subTest(output=output),self.assertRaises(ValueError):module.build_plan(repo,output)

    def test_resume_cannot_follow_output_symlinks_into_production(self):
        module=self.module()
        for relative in ['backend_install','backend_build','frontend/source','frontend/build',
                         'frontend/install','colcon_logs',
                         'backend_install/a2w_fastlio_mapping/lib/a2w_fastlio_mapping',
                         'backend_build/a2w_fastlio_mapping']:
            with self.subTest(relative=relative), tempfile.TemporaryDirectory() as directory:
                repo=Path(directory); output=repo/'.controlled_mapping'
                target=repo/'install'; target.mkdir()
                link=output/relative; link.parent.mkdir(parents=True,exist_ok=True)
                link.symlink_to(target,target_is_directory=True)
                with self.assertRaises(ValueError):module.build_plan(repo,output)

    def test_capture_wrapper_template_never_guesses_sensor_units(self):
        script=MODULE_PATH.with_name('mapping_capture.sh')
        self.assertTrue(script.is_file(),'controlled capture wrapper is missing')
        result=subprocess.run(['bash',str(script),'interface-template'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        value=json.loads(result.stdout)
        self.assertEqual(value['gyro_unit'],'unknown')
        self.assertEqual(value['accel_unit'],'unknown')

if __name__=='__main__':unittest.main()
