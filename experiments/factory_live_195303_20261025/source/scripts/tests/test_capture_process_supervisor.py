"""Real short-lived children and startup evidence; no ROS, fabricated wait, or DDS."""
from pathlib import Path
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

NEAR_SCRIPTS=Path(__file__).resolve().parents[1]
CANONICAL=NEAR_SCRIPTS if (NEAR_SCRIPTS/'mapping_capture.py').is_file() else Path(__file__).resolve().parents[2]/'source/scripts'
SCRIPTS=Path(os.environ.get('A2W_SUPERVISOR_TEST_SCRIPTS',str(CANONICAL)))
sys.dont_write_bytecode=True
sys.path.insert(0,str(SCRIPTS))
spec=importlib.util.spec_from_file_location('fast_exit_capture',SCRIPTS/'mapping_capture.py')
capture=importlib.util.module_from_spec(spec);spec.loader.exec_module(capture)

class RealStartupEvidence(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='supervisor-fast-exit-')
        self.addCleanup(self.tmp.cleanup)
        self.session=Path(self.tmp.name)
        (self.session/'logs').mkdir();(self.session/'processes.json').write_text('{}')

    def invocation(self,command,unobserved=False,ignored_sigchld=False):
        script=SCRIPTS/'capture_process_supervisor.py'
        wrapper=self.session/'invoke_supervisor.py'
        wrapper.write_text('''import importlib.util,os,pathlib,signal,sys
sys.path.insert(0,str(pathlib.Path(sys.argv[1]).parent))
if IGNORED_CHLD:signal.signal(signal.SIGCHLD,signal.SIG_IGN)
s=importlib.util.spec_from_file_location("tested_waiter",sys.argv[1])
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
if UNOBSERVED:
 def unavailable(_pid):raise FileNotFoundError("injected proc visibility race")
 m.process_identity=unavailable
# Keep a real direct-child fd solely to clean up a broken baseline test. No
# wait/status result is replaced, and no PID lookup is used to signal cleanup.
real_popen=m.subprocess.Popen
owned={}
def tracked_popen(*args,**kwargs):
 child=real_popen(*args,**kwargs)
 owned.update(child=child,fd=os.pidfd_open(child.pid))
 return child
m.subprocess.Popen=tracked_popen
sys.argv=sys.argv[1:]
try:result=m.main()
finally:
 if owned:
  child=owned['child']
  if child.poll() is None:
   try:signal.pidfd_send_signal(owned['fd'],signal.SIGTERM)
   except ProcessLookupError:pass
   child.wait(timeout=5)
  os.close(owned['fd'])
sys.exit(result)
'''.replace('IGNORED_CHLD',repr(ignored_sigchld)).replace('UNOBSERVED',repr(unobserved)))
        return [sys.executable,str(wrapper),str(script),str(self.session),'probe',*command]

    def invoke(self,command,unobserved=False,ignored_sigchld=False):
        result=subprocess.run(self.invocation(command,unobserved,ignored_sigchld),text=True,capture_output=True,timeout=9)
        path=self.session/'process_exits/probe.json'
        self.assertTrue(path.is_file(),'actual child result missing: '+result.stderr)
        return result,json.loads(path.read_text())

    def child_status(self,status,code):
        self.assertEqual(status['schema'],'owned_process_exit_v1')
        self.assertEqual(status.get('outcome'),'child_exit')
        self.assertEqual(status['exit_code'],code)
        self.assertTrue(status['child_created'] and status['reaped'])
        self.assertIs(type(status['exec_identity_observed']),bool)
        self.assertTrue(status['log_flushed'] and status['log_closed'])
        self.assertIsNone(status['spawn_error'])

    def test_fast_true_retains_actual_zero_even_without_registration(self):
        _,status=self.invoke(['/bin/true']);self.child_status(status,0)

    def test_fast_false_retains_actual_one(self):
        _,status=self.invoke(['/bin/false']);self.child_status(status,1)

    def test_fast_seven_retains_actual_seven(self):
        _,status=self.invoke(['/bin/sh','-c','exit 7']);self.child_status(status,7)

    def test_observed_nonzero_child_keeps_verified_identity(self):
        _,status=self.invoke([sys.executable,'-c','import sys,time; time.sleep(.12); sys.exit(7)'])
        self.child_status(status,7);self.assertTrue(status['exec_identity_observed'])
        self.assertIsNone(status['startup_error'])

    def test_exec_missing_is_not_fabricated_child_exit(self):
        result,status=self.invoke([str(self.session/'missing-executable')])
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(status['schema'],'owned_process_spawn_error_v1')
        self.assertEqual(status['outcome'],'spawn_error')
        self.assertIsNone(status['exit_code']);self.assertIsNone(status['child_pid'])
        self.assertIsNone(status['identity']);self.assertFalse(status['child_created'])
        self.assertFalse(status['exec_identity_observed']);self.assertFalse(status['reaped'])
        self.assertEqual(status['spawn_error']['errno'],2)
        self.assertFalse((self.session/'process_exits/probe.started.json').exists())

    def test_unobserved_exec_is_reaped_without_invented_identity(self):
        _,status=self.invoke([sys.executable,'-c','import sys; sys.exit(7)'],unobserved=True)
        self.child_status(status,7)
        self.assertFalse(status['exec_identity_observed']);self.assertIsNone(status['identity'])
        self.assertTrue(status['startup_error'])
        self.assertFalse((self.session/'process_exits/probe.started.json').exists())

    def test_inherited_sigchld_ignore_cannot_fabricate_zero(self):
        _,status=self.invoke([sys.executable,'-c','import sys; sys.exit(7)'],ignored_sigchld=True)
        self.child_status(status,7);self.assertTrue(status['inherited_sigchld_ignored'])

    def test_unobserved_live_child_is_cleaned_only_via_owned_pidfd(self):
        _,status=self.invoke([sys.executable,'-c','import time; time.sleep(30)'],unobserved=True)
        self.child_status(status,-signal.SIGTERM)
        self.assertFalse(status['exec_identity_observed']);self.assertIsNone(status['identity'])
        self.assertEqual(status['cleanup_signal'],signal.SIGTERM)
        failure=json.loads((self.session/'process_exits/probe.startup_failed.json').read_text())
        self.assertTrue(failure['child_pidfd_captured'])
        self.assertFalse((self.session/'process_exits/probe.started.json').exists())

    def test_ignored_term_retains_pending_waiter_then_persists_real_exit(self):
        code='import signal,sys,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(4.2); sys.exit(7)'
        command=self.invocation([sys.executable,'-c',code],unobserved=True)
        process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            failed=self.session/'process_exits/probe.startup_failed.json'
            end=time.monotonic()+3.8
            while not failed.is_file() and time.monotonic()<end:time.sleep(.01)
            self.assertTrue(failed.is_file())
            failure=json.loads(failed.read_text())
            self.assertTrue(failure['child_reap_pending'])
            self.assertIsNone(process.poll())
            self.assertFalse((self.session/'process_exits/probe.json').exists())
        finally:
            out,err=process.communicate(timeout=7)
        self.assertEqual(process.returncode,7,out+err)
        self.child_status(json.loads((self.session/'process_exits/probe.json').read_text()),7)

    def test_spawn_owned_fast_exit_reports_status_path_and_does_not_register(self):
        with self.assertRaises(ValueError) as caught:
            capture.spawn_owned(self.session,'probe',['/bin/false',str(self.session)])
        path=self.session/'process_exits/probe.json'
        self.assertIn(str(path),str(caught.exception))
        self.assertTrue(path.is_file(),str(caught.exception))
        self.assertEqual(json.loads(path.read_text())['exit_code'],1)
        self.assertEqual(json.loads((self.session/'processes.json').read_text()),{})

    def test_spawn_owned_exec_error_reports_distinct_status_path(self):
        with self.assertRaises(ValueError) as caught:
            capture.spawn_owned(self.session,'probe',[str(self.session/'missing-executable'),str(self.session)])
        path=self.session/'process_exits/probe.json'
        self.assertIn(str(path),str(caught.exception));self.assertTrue(path.is_file())
        self.assertEqual(json.loads(path.read_text())['schema'],'owned_process_spawn_error_v1')

    def test_spawn_owned_pending_waiter_fails_quickly_without_registration(self):
        code='import signal,sys,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(4.2); sys.exit(7)'
        command=[sys.executable,'-c',code,str(self.session)]
        wrapper=Path(self.invocation(command,unobserved=True)[1])
        replacement=self.session/'capture_process_supervisor.py'
        replacement.write_text('import sys\nsys.argv.insert(1,'+repr(str(SCRIPTS/'capture_process_supervisor.py'))+')\n'+wrapper.read_text())
        previous=capture.__file__;capture.__file__=str(self.session/'mapping_capture.py')
        real_popen=subprocess.Popen;supervisors=[]
        def tracked_popen(*args,**kwargs):
            process=real_popen(*args,**kwargs);supervisors.append(process);return process
        begin=time.monotonic()
        try:
            with patch.object(capture.subprocess,'Popen',side_effect=tracked_popen):
                with self.assertRaises(ValueError) as caught:capture.spawn_owned(self.session,'probe',command)
            self.assertLess(time.monotonic()-begin,3.9)
            self.assertIn('pending/unknown',str(caught.exception))
            self.assertIn(str(self.session/'process_exits/probe.json'),str(caught.exception))
            self.assertEqual(json.loads((self.session/'processes.json').read_text()),{})
            self.assertIsNone(supervisors[0].poll())
        finally:
            capture.__file__=previous
            for process in supervisors:process.wait(timeout=7)
        self.child_status(json.loads((self.session/'process_exits/probe.json').read_text()),7)

    def test_normal_sigint_reaps_and_keeps_verified_identity(self):
        code='import signal,sys,time; signal.signal(signal.SIGINT,lambda *_:sys.exit(0)); print("READY",flush=True); time.sleep(30)'
        child,identity=capture.spawn_owned(self.session,'probe',[sys.executable,'-c',code,str(self.session)])
        self.addCleanup(lambda: child.poll() is None and child.send_signal(signal.SIGINT))
        end=time.monotonic()+3
        while 'READY' not in (self.session/'logs/probe.log').read_text() and time.monotonic()<end:time.sleep(.01)
        capture.signal_owned(identity,signal.SIGINT)
        self.assertEqual(child.wait(timeout=5),0)
        status=json.loads((self.session/'process_exits/probe.json').read_text())
        self.child_status(status,0);self.assertTrue(status['exec_identity_observed'])
        self.assertEqual(status['identity'],identity)

if __name__=='__main__':unittest.main()
