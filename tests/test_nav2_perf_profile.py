"""Hand-timed perf fixtures: no ROS process, root, or robot needed."""
import importlib.util
from pathlib import Path
import unittest


PATH = Path(__file__).resolve().parents[1] / "scripts/diagnostics/nav2_perf_profile.py"


class ProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not PATH.exists():
            cls.profile = None
            return
        spec = importlib.util.spec_from_file_location("nav2_perf_profile", PATH)
        cls.profile = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.profile)

    def analyze(self, text):
        self.assertIsNotNone(self.profile, "diagnostic analyzer not implemented")
        return self.profile.analyze(text)

    def test_separates_nested_stages_and_pre_eval_gap(self):
        # Wall-clock durations: MPPI 50ms, path 2ms, gap 5ms, eval 40ms, viz 3ms.
        text = "\n".join(f"  123 {stamp}: a2w_navprof_test:{event}:" for stamp, event in [
            ("1.000000", "mppi_in"), ("1.000000", "path_in"),
            ("1.002000", "path_out"), ("1.007000", "eval_in"),
            ("1.047000", "eval_out"), ("1.047000", "viz_in"),
            ("1.050000", "viz_out"), ("1.050000", "mppi_out")])
        result = self.analyze(text)
        for name, want in [("mppi", 50.), ("path", 2.), ("eval", 40.),
                           ("viz", 3.), ("pre_eval_gap", 5.)]:
            self.assertAlmostEqual(result["stages_ms"][name]["p50"], want)

    def test_does_not_mix_threads_or_incomplete_calls(self):
        result = self.analyze("""1 2.000000: a2w_navprof_t:eval_in:
2 2.010000: a2w_navprof_t:eval_in:
1 2.020000: a2w_navprof_t:eval_out:
2 2.050000: a2w_navprof_t:eval_out:
1 2.060000: a2w_navprof_t:eval_out:
1 2.070000: a2w_navprof_t:eval_in:
""")
        self.assertEqual(result["stages_ms"]["eval"]["count"], 2)
        self.assertAlmostEqual(result["stages_ms"]["eval"]["min"], 20.)
        self.assertAlmostEqual(result["stages_ms"]["eval"]["max"], 40.)
        self.assertEqual(result["unpaired_entries"], 1)
        self.assertEqual(result["unpaired_returns"], 1)

    def test_missing_samples_not_successful_profile(self):
        result = self.analyze("# no active goal\n")
        self.assertFalse(result["active_control_sampled"])
        self.assertEqual(result["stages_ms"], {})

    def test_missing_path_return_cannot_be_claimed_as_lock_wait(self):
        result = self.analyze("""1 1.000000: a2w_navprof_t:mppi_in:
1 1.007000: a2w_navprof_t:eval_in:
1 1.047000: a2w_navprof_t:eval_out:
1 1.050000: a2w_navprof_t:mppi_out:
""")
        self.assertNotIn("pre_eval_gap", result["stages_ms"])


if __name__ == "__main__":
    unittest.main()
