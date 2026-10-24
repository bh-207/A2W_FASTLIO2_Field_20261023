import importlib.util
from pathlib import Path
import tempfile
import unittest

SPEC=importlib.util.spec_from_file_location('library_contract',Path(__file__).with_name('replay_library_contract.py'))
module=importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(module)

class ReplayLibraryContract(unittest.TestCase):
    def fixture(self,root):
        folder=root/'isolated prefix'/'lib'; folder.mkdir(parents=True)
        names=['liba2w_registration.so','liba2w_algorithm_suite.so']
        for name in names:(folder/name).write_bytes(name.encode())
        text='\n'.join(f'{name} => {folder/name} (0x1234)'for name in names)
        return folder,text
    def test_exact_isolated_libraries_have_content_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            folder,text=self.fixture(Path(directory)); result=module.verify(text,folder)
            self.assertEqual(set(result['required']),{'liba2w_registration.so','liba2w_algorithm_suite.so'})
            self.assertEqual(len(result['required']['liba2w_registration.so']['sha256']),64)
    def test_main_workspace_override_and_missing_dependency_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder,text=self.fixture(root)
            old=root/'old.so';old.write_text('old')
            with self.assertRaises(ValueError):module.verify(text.replace(str(folder/'liba2w_registration.so'),str(old)),folder)
            with self.assertRaises(ValueError):module.verify(text.replace('liba2w_registration.so =>','missing.so =>'),folder)
    def test_symlink_escape_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder,text=self.fixture(root)
            outside=root/'outside';outside.write_text('outside')
            (folder/'liba2w_registration.so').unlink();(folder/'liba2w_registration.so').symlink_to(outside)
            with self.assertRaises(ValueError):module.verify(text,folder)
    def test_preload_line_without_arrow_is_hashed(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder,text=self.fixture(root)
            shim=root/'shim.so';shim.write_text('frozen time shim')
            result=module.verify(str(shim)+' (0x4321)\n'+text,folder)
            self.assertIn(str(shim.resolve()),result['all_libraries'])
    def test_required_library_preloaded_without_arrow_is_verified(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);folder,text=self.fixture(root)
            text=text.replace('liba2w_registration.so => ','')
            result=module.verify(text,folder)
            self.assertIn('liba2w_registration.so',result['required'])
            outside=root/'wrong'/'liba2w_registration.so'
            outside.parent.mkdir();outside.write_text('wrong prefix')
            with self.assertRaises(ValueError):
                module.verify(text.replace(str(folder/'liba2w_registration.so'),str(outside)),folder)

if __name__=='__main__':unittest.main()
