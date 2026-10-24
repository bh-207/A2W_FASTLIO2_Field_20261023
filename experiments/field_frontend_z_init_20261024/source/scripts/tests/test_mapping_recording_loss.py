"""A normal rosbag exit must not hide explicit cache-loss reports."""
import inspect
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mapping_capture as cli


class RecordingLossTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.session = Path(self.tmp.name)
        (self.session / 'logs').mkdir()
        self.log = self.session / 'logs/recorder.log'
        self.log.write_text('[INFO] Recording stopped\n')

    def quality(self):
        self.assertIn('recorder_log', inspect.signature(cli.recording_quality).parameters,
                      'Recording verification must inspect the actual recorder log')
        ranges = {t: dict(first_record_ns=100, last_record_ns=200, count=2,
                          max_record_gap_ns=100) for t in cli.RAW_TOPICS}
        live = dict(topics={t: dict(last_receipt_unix_ns=200, count=2)
                            for t in cli.RAW_TOPICS})
        return cli.recording_quality(dict(exit_code=0, metadata_present=True),
                                     ranges, live, 150, recorder_log=self.log)

    def test_normal_exit_and_good_ranges_cannot_override_explicit_cache_loss(self):
        self.log.write_text('[WARN] Cache buffers lost messages per topic:\n'
                            '\t/lidar_points: 29\nTotal lost: 789\n'
                            '[WARN] Cache buffers lost messages per topic:\n'
                            '\t/lidar_points: 29\nTotal lost: 805\n'
                            '[INFO] Recording stopped\n')
        result = self.quality()
        self.assertFalse(result['checks_passed'])
        self.assertTrue(any('cache loss' in e for e in result['errors']))
        self.assertEqual([789, 805], result['recorder_log']['reported_totals'])
        self.assertIs(False, result['lossless_proven'])

    def test_log_without_loss_report_is_checked_without_certifying_losslessness(self):
        result = self.quality()
        self.assertTrue(result['checks_passed'])
        self.assertTrue(result['recorder_log']['checked'])
        self.assertFalse(result['recorder_log']['loss_reported'])
        self.assertIs(False, result['lossless_proven'])

    def test_missing_log_and_incomplete_loss_block_fail_closed(self):
        self.log.unlink()
        result = self.quality()
        self.assertFalse(result['checks_passed'])
        self.log.write_text('[WARN] Cache buffers lost messages per topic:\n')
        self.assertFalse(self.quality()['checks_passed'])

    def test_live_save_degradation_also_reports_recorder_cache_loss(self):
        (self.session / 'session.json').write_text(json.dumps(dict(
            schema='mapping_capture_session_v1', session=str(self.session))))
        (self.session / 'processes.json').write_text('{}')
        self.log.write_text('[WARN] Cache buffers lost messages per topic:\nTotal lost: 1\n')
        self.assertTrue(any('cache loss' in e for e in cli.capture_degradation(self.session)))


if __name__ == '__main__':
    unittest.main()
