from pathlib import Path
import subprocess
import tempfile
import unittest
import json

class CountsTest(unittest.TestCase):
    def test_new_bag_exact_counts_without_weakening_old_contract(self):
        code = '''#include "offline_consume_policy.hpp"
#include <cassert>
int main(){using offline_consume::full_counts;
assert(full_counts(166023,8188));
assert(!full_counts(355087,17778));
assert(full_counts(355087,17778,355087,17778));
assert(!full_counts(355086,17778,355087,17778));
assert(!full_counts(355088,17778,355087,17778));
assert(!full_counts(355087,17777,355087,17778));
assert(!full_counts(0,0,0,0));}'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'test.cpp').write_text(code)
            r=subprocess.run(['g++','-std=c++17','-Werror','-Wall','-I'+str(Path(__file__).parent/'source/frontend/include'),str(p/'test.cpp'),'-o',str(p/'test')],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
            self.assertEqual(subprocess.run([str(p/'test')]).returncode,0)

    def test_bounded_stage_ledger_fits_this_full_bag(self):
        code='''#include "frontend_input_integrity.hpp"
int main(int argc,char**argv){frontend_integrity::Trace trace(argv[1]);
trace.record(frontend_integrity::Stage::ImuRaw,123);
return trace.close({})?0:2;}'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'test.cpp').write_text(code)
            subprocess.run(['g++','-std=c++17','-pthread','-I'+str(Path(__file__).parent/'source/frontend/include'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
            subprocess.run([str(p/'test'),str(p/'output')],check=True)
            status=json.loads((p/'output/frontend_processing_status.json').read_text())
            self.assertEqual(status['trace_capacity'],2500000)
            self.assertTrue(status['closed'] and status['flush_ok'])
            self.assertEqual(status['trace_overflow'],0)

if __name__=='__main__':unittest.main()
