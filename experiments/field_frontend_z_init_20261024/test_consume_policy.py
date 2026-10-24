import pathlib
import subprocess
import tempfile
import unittest


class ConsumePolicyTest(unittest.TestCase):
    def test_explicit_isolation_and_exact_counts(self):
        header = pathlib.Path(__file__).parent / 'source/frontend/include/offline_consume_policy.hpp'
        code = '''#include "offline_consume_policy.hpp"
#include <cassert>
int main(){
  using namespace offline_consume;
  assert(isolated("offline-test","87","1"));
  assert(!isolated("live","0","0"));
  assert(!isolated("offline-test","87","0"));
  assert(full_counts(166023,8188));
  assert(!full_counts(166022,8188));
  assert(!full_counts(166023,8187));
  assert(!full_counts(166024,8188));
}'''
        with tempfile.TemporaryDirectory() as temp:
            source = pathlib.Path(temp)/'test.cpp';source.write_text(code)
            build = subprocess.run(['g++','-std=c++17','-Werror','-Wall','-I'+str(header.parent),str(source),'-o',temp+'/test'],capture_output=True,text=True)
            self.assertEqual(build.returncode,0,build.stderr)
            self.assertEqual(subprocess.run([temp+'/test']).returncode,0)
