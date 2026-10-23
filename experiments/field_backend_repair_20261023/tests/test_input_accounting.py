import importlib.util
from pathlib import Path
import sqlite3
import struct
import tempfile
import unittest

PATH=Path(__file__).resolve().parents[1]/'audit_inputs.py'
spec=importlib.util.spec_from_file_location('account',PATH)
account=importlib.util.module_from_spec(spec);spec.loader.exec_module(account)

class InputAccountingTest(unittest.TestCase):
    def test_empty_comparison_cannot_pass(self):
        from collections import Counter
        self.assertFalse(account.compare(Counter(),Counter())['complete'])

    def test_published_prefix_and_tail_loss_are_not_clipped_away(self):
        from unittest.mock import patch
        from collections import Counter
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'input.csv').write_text('status,raw_stamp_ns\nadapted,2000000000\n')
            (root/'published_raw.csv').write_text('topic,header_ns\n/lidar_imu,1000000000\n/lidar_imu,2000000000\n/lidar_imu,3000000000\n')
            with patch.object(account,'headers',return_value=Counter({1000000000:1,2000000000:1,3000000000:1})):
                result=account.run(root)
            self.assertTrue(result['published_vs_callback']['complete'])  # descriptive overlap only
            self.assertFalse(result['published_vs_callback_full']['complete'])
            self.assertEqual(result['published_vs_callback_full']['a_only_count'],2)
            self.assertEqual(result['callback_vs_recorded_full']['b_only_count'],2)

    def test_cdr_integer_endian_and_duplicate_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'data.db3'
            with sqlite3.connect(path) as db:
                db.executescript('CREATE TABLE topics(id INTEGER,name TEXT); CREATE TABLE messages(topic_id INTEGER,data BLOB); INSERT INTO topics VALUES(1,"/lidar_imu");')
                for little,sec,ns in [(True,1792724419,896379948),(False,1792724419,896379948),(True,1792724419,896379949)]:
                    data=(b'\x00\x01\x00\x00' if little else b'\x00\x00\x00\x00')+struct.pack(('<' if little else '>')+'iI',sec,ns)
                    db.execute('INSERT INTO messages VALUES(1,?)',(data,))
            result=account.headers(tmp,'/lidar_imu')
            self.assertEqual(result[1792724419896379948],2)
            self.assertEqual(result[1792724419896379949],1)

    def test_differences_never_cancel_to_net_loss(self):
        from collections import Counter
        result=account.compare(Counter({100:3,101:1}),Counter({100:1,102:1}))
        self.assertEqual(result['a_only_count'],3)
        self.assertEqual(result['b_only_count'],1)
        self.assertFalse(result['complete'])

    def test_nonempty_wal_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'data.db3';path.touch();Path(str(path)+'-wal').write_bytes(b'uncommitted')
            with self.assertRaises(ValueError):account.headers(tmp,'/lidar_imu')

if __name__=='__main__':unittest.main()
