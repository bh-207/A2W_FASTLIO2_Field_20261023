"""Summary contracts: commits are not geometric truth or queued candidates."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

PATH = Path(__file__).with_name("loop_v3_summary.py")
summary = None
if PATH.exists():
    spec = importlib.util.spec_from_file_location("loop_v3_summary", PATH)
    summary = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(summary)


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def fixture(folder, cached=False):
    folder.mkdir()
    (folder / "run_metadata.txt").write_text("status=started\nstatus=complete\nfirst_match_id=0\nprocess_starts_at=0\n"
        "frame_process_wall_ms=process_only_excludes_csv_and_snapshot_io\n"
        "loop_pipeline.origin_keyframe_count=31\nloop_pipeline.queue_capacity=32\n"
        "loop_validation.maximum_fitness=0.25\nloop_validation.minimum_overlap=0.35\n"
        "loop_validation.minimum_correspondences=30\nsc_cached=" + str(int(cached)) + "\n")
    write_csv(folder / "frames.csv", [dict(current=i, stamp_ns=t, sensor_elapsed_s=elapsed,
        matching_suppressed=0,sc_cached=int(cached), events=n,committed_loops=c,
        process_wall_ms=p,pipeline_frame_total_ms=p-1,descriptor_ms=1,scan_context_ms=sc,gtsam_ms=g)
        for i,t,elapsed,n,c,p,sc,g in [(0,1000000000,0,1,0,100,10,1),(617,2000000000,1,2,0,200,20,2),(1193,3000000000,2,1,1,300,30,3)]])
    events=[]
    for current,candidate,accepted,committed,reason,fitness,q,f in [
        (0,0,0,0,"no_loop_candidates","inf",0,0),
        (617,3,1,0,"loop_confirmation_pending",.1,6,3),
        (617,4,0,0,"fitness_above_limit",.4,4,2),
        (1193,3,1,1,"loop_accepted",.1,8,4)]:
        events.append(dict(current=current,stamp_ns={0:1000000000,617:2000000000,1193:3000000000}[current],
            candidate=candidate,rank=0,candidates=0 if current==0 else 2,accepted=accepted,graph_committed=committed,
            reason=reason,rejection_stage="retrieval_or_policy" if current==0 else "confirmation",
            registration_success=int(q>0),registration_converged=int(q>0),fitness=fitness,overlap=.8,
            correspondences=100 if q else 0,descriptor_ms=1,scan_context_ms={0:10,617:20,1193:30}[current],
            source_local_map_ms=2 if q else 0,target_local_map_ms=1 if q else 0,quatro_ms=q,nano_gicp_ms=f,
            evidence_ms=1 if q else 0,validator_ms=.1 if q else 0,gtsam_ms=1,registration_total_ms=q+f+1 if q else 0,
            has_pose_check=int(q>0),position_pass=1,rotation_pass=1,origin_candidate=1,confirmation_count=2))
    write_csv(folder / "events.csv",events)
    write_csv(folder / "queries.csv",[dict(current=i,query_number=0,candidate=c,rank=r,raw_result_count=n,
        descriptor_distance=.1 if c!="" else "",matching_suppressed=0) for i,c,r,n in [(0,"","",0),(617,3,0,2),(617,4,1,2),(1193,3,0,1)]])
    write_csv(folder / "loop_constraints.csv",[dict(current_id=1193,candidate_id=3,relative_tx=0,relative_ty=0,relative_tz=0,relative_qx=0,relative_qy=0,relative_qz=0,relative_qw=1)])
    write_csv(folder / "summary.csv",[dict(metric=k,value=v) for k,v in [("frames",3),("committed_loops",1),("replay_wall_ms_including_csv_snapshot_io",650)]])


class LoopV3SummaryTest(unittest.TestCase):
    def api(self):
        self.assertIsNotNone(summary,"loop_v3_summary implementation absent")
        return summary

    def test_quantiles_use_linear_interpolation_and_empty_is_missing(self):
        a=self.api()
        stats=a.distribution([1,2,3,4])
        self.assertEqual(stats["p50"],2.5)
        self.assertAlmostEqual(stats["p95"],3.85)
        self.assertIsNone(a.distribution([])["p50"])

    def test_committed_is_not_temporary_accepted_and_false_labels_are_null(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            result=a.summarize_run(path)
            self.assertEqual(result["counts"]["committed_loops"],1)
            self.assertEqual(result["counts"]["accepted_event_rows"],2)
            self.assertIsNone(result["truth"]["false_count"])
            self.assertEqual(result["first_origin_commit"]["current"],1193)
            self.assertIsNone(result["first_start_window_commit"])

    def test_frame_stage_times_are_not_multiplied_by_candidate_count(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            result=a.summarize_run(path)
            self.assertEqual(result["frame_timings_ms"]["scan_context_ms"]["sum"],60)
            self.assertEqual(result["counts"]["geometry_attempts"],3)
            self.assertEqual(result["counts"]["query_calls"],3)
            self.assertEqual(result["counts"].get("joint_geometry_pose_pass_rows"),2)
            self.assertAlmostEqual(result["throughput"]["process_only_fps"],5)
            self.assertIsNone(result["actual_wall"]["end_latency_ms"]["p95"])
            self.assertFalse(result["throughput"]["cold_load_included"])

    def test_cached_sc_and_missing_scheduled_are_explicit_waiting(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);fixture(root/"v2",cached=True)
            result=a.build_report({"v2":root/"v2","scheduled":root/"scheduled"})
            self.assertEqual(result["status"],"waiting")
            self.assertEqual(result["runs"]["scheduled"]["status"],"waiting")
            self.assertFalse(result["runs"]["v2"]["sc_timing"]["online_comparable"])

    def test_constraint_mismatch_is_non_pass_not_false_loop(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            write_csv(path/"loop_constraints.csv",[dict(current_id=617,candidate_id=3)])
            result=a.summarize_run(path)
            self.assertIn("constraint_event_commit_mismatch",result["non_pass_markers"])
            self.assertIsNone(result["truth"]["false_count"])

    def test_actual_pace_cap32_overload_is_measured_not_simulated(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            write_csv(path/"pace_frames.csv",[dict(current=617,stamp_ns=2000000000,pace_enabled=1,
                scheduled_arrival_ms=1000,actual_start_wall_ms=1010,actual_graph_commit_wall_ms="",
                actual_end_after_io_wall_ms=1300,start_wall_latency_ms=10,end_wall_latency_ms=300,
                arrived_unprocessed_at_start=4,arrived_waiting_after_end=33,queue_capacity=32,capacity_exceeded=1)])
            write_csv(path/"commit_latency.csv",[dict(current=1193,candidate=3,stamp_ns=3000000000,pace_enabled=1,
                scheduled_arrival_ms=2000,actual_graph_commit_wall_ms=2300,commit_wall_latency_ms=300)])
            result=a.summarize_run(path)
            self.assertEqual(result["actual_wall"]["end_latency_ms"]["p50"],300)
            self.assertEqual(result["actual_wall"]["commit_latency_ms"]["p95"],300)
            self.assertEqual(result["actual_wall"]["maximum_pending"],33)
            self.assertEqual(result["actual_wall"]["capacity_acceptance"],"FAIL")
            self.assertFalse(result["actual_wall"]["real_transport_queue"])
            self.assertIn("pace_capacity_exceeded",result["non_pass_markers"])

    def test_pace_disabled_empty_fields_never_become_zero_latency(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            write_csv(path/"pace_frames.csv",[dict(current=0,pace_enabled=0,start_wall_latency_ms="",end_wall_latency_ms="")])
            result=a.summarize_run(path)
            self.assertEqual(result["actual_wall"]["status"],"not_paced")
            self.assertIsNone(result["actual_wall"]["end_latency_ms"]["p50"])

    def test_start_upper_bound_and_end_capacity_are_separate_not_bounded_queue_pass(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            write_csv(path/"pace_frames.csv",[dict(current=617,pace_enabled=1,start_wall_latency_ms=10,end_wall_latency_ms=20,
                arrived_unprocessed_at_start=33,arrived_waiting_after_end=32,queue_capacity=32,capacity_exceeded=0)])
            result=a.summarize_run(path)
            self.assertEqual(result["actual_wall"]["maximum_pending"],32)
            self.assertEqual(result["actual_wall"]["maximum_unprocessed_at_start_including_next_active"],33)
            self.assertEqual(result["actual_wall"].get("end_waiting_capacity_check"),"PASS")
            self.assertEqual(result["actual_wall"].get("start_including_active_capacity_upper_bound_check"),"EXCEEDS_UPPER_BOUND")
            self.assertEqual(result["actual_wall"]["capacity_acceptance"],"NOT_ESTABLISHED")
            self.assertFalse(result["actual_wall"].get("continuous_queue_bound_established",True))

    def test_both_sampled_queue_counts_below32_still_do_not_certify_continuous_queue(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            write_csv(path/"pace_frames.csv",[dict(current=617,pace_enabled=1,start_wall_latency_ms=10,end_wall_latency_ms=20,
                arrived_unprocessed_at_start=1,arrived_waiting_after_end=0,queue_capacity=32,capacity_exceeded=0)])
            result=a.summarize_run(path)
            self.assertEqual(result["actual_wall"].get("end_waiting_capacity_check"),"PASS")
            self.assertEqual(result["actual_wall"].get("start_including_active_capacity_upper_bound_check"),"PASS")
            self.assertEqual(result["actual_wall"]["capacity_acceptance"],"NOT_ESTABLISHED")

    def test_service_fps_includes_primary_io_but_excludes_pacing_sleep(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            write_csv(path/"pace_frames.csv",[dict(current=i,pace_enabled=1,start_wall_latency_ms=0,end_wall_latency_ms=duration,
                actual_start_wall_ms=start,actual_end_after_io_wall_ms=start+duration,
                arrived_unprocessed_at_start=1,arrived_waiting_after_end=0,queue_capacity=32,capacity_exceeded=0)
                for i,start,duration in [(0,0,120),(617,10000,240),(1193,20000,360)]])
            result=a.summarize_run(path)
            self.assertEqual(result["throughput"]["process_only_fps"],5)
            self.assertAlmostEqual(result["throughput"].get("service_with_primary_io_fps",0),25/6)
            self.assertEqual(result["actual_wall"].get("service_with_primary_io_ms",{}).get("sum"),720)
            self.assertTrue(result["throughput"].get("elapsed_rate_includes_pacing_sleep"))

    def test_unpaced_service_is_measured_even_when_latency_and_queue_are_empty(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            write_csv(path/"pace_frames.csv",[dict(current=i,pace_enabled=0,start_wall_latency_ms="",end_wall_latency_ms="",
                actual_start_wall_ms=start,actual_end_after_io_wall_ms=start+duration)
                for i,start,duration in [(0,0,120),(617,120,240),(1193,360,360)]])
            result=a.summarize_run(path)
            self.assertEqual(result["actual_wall"]["status"],"not_paced")
            self.assertIsNone(result["actual_wall"]["end_latency_ms"]["p50"])
            self.assertAlmostEqual(result["throughput"].get("service_with_primary_io_fps",0),25/6)

    def test_missing_service_time_does_not_turn_into_fake_service_fps(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            write_csv(path/"pace_frames.csv",[dict(current=0,pace_enabled=0,start_wall_latency_ms="",end_wall_latency_ms="",
                actual_start_wall_ms=0,actual_end_after_io_wall_ms="")])
            result=a.summarize_run(path)
            self.assertIsNone(result["throughput"].get("service_with_primary_io_fps"))

    def test_missing_process_cost_does_not_inflate_fps(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            with (path/"frames.csv").open() as stream: rows=list(csv.DictReader(stream))
            rows[1]["process_wall_ms"]="nan"
            write_csv(path/"frames.csv",rows)
            result=a.summarize_run(path)
            self.assertIsNone(result["throughput"]["process_only_fps"])
            self.assertIn("invalid_frame_process_times",result["non_pass_markers"])

    def test_missing_schema_keeps_negative_status(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/"run";fixture(path)
            (path/"events.csv").write_text("current,accepted\n0,1\n")
            result=a.summarize_run(path)
            self.assertEqual(result["status"],"invalid")
            self.assertIn("events", " ".join(result["errors"]))

    def test_output_cannot_overwrite_or_contain_inputs(self):
        a=self.api()
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);fixture(root/"run")
            runs={"baseline":root/"run"}
            a.write_report(runs,root/"report")
            data=json.loads((root/"report/summary.json").read_text())
            self.assertIn("baseline",data["runs"])
            self.assertIn("独立真值",(root/"report/summary_zh.txt").read_text())
            with self.assertRaises(ValueError):a.write_report(runs,root/"report")
            with self.assertRaises(ValueError):a.write_report(runs,root)


if __name__=="__main__":unittest.main()
