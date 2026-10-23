# Frontend initialization v4 implementation plan

> Execution: inline, explicitly authorized; TDD and verification before completion. Isolated source copy, never a production checkout edit.

Goal: default-off continuous IMU initialization quality gate and explicit one-pass unit/time interface integrated into FAST-LIO Process/callback, with actual state evidence and an independently reviewed external LI-Init export path.

Authority: parent v4 task, 2026-10-21. No robot/164 connection, no ROS publication, no production/default/main-install changes. Only new patches/fast_lio_v4, frontend_init_v4* scripts/tests/docs and artifacts/loop_v4_20261021/frontend.

Architecture: standard-library C++ core is copied into an isolated FAST_LIO_Hesai source; a tracked unified patch wires actual callback and ImuProcess::Process. Gate-off branch remains original. A ROS-free executable calls the same actual Process/ESKF with frozen synthetic raw IMU/point-time data, not saved1016 clouds. Build full ROS node but never launch it. Unit claims require explicit evidence and known frame; acceleration normalization inside FAST-LIO remains separately documented. LI-Init result review never directly changes a production config.

Base: src/FAST_LIO_Hesai HEAD16e97cdcc4260b9ba6241518a19ad8384224ba48, clean at task start. Store source/patch/build/input hashes. Parent specifically authorizes shared new-file paths; source isolation is an independent artifact copy, not a branch or submodule mutation.

## Tasks and completion evidence

- [x] Core TDD: static inclined window; duration AND count; gyro/acc variance, norm/direction; motion reset; duplicate/backward/gap reset; finite values/config; no partial initialization; one-pass unit/time (including positive/negative offset sign, double conversion and evidence mismatch failures).
- [x] Integration TDD: patch applies only expected base and refuses output overwrite; gate/config default off; enabled strict path reaches actual Process; fullgroup checked before commit so late bad sample cancels readiness; exact member stamps/stats and committed state trace.
- [x] Isolated build and actual Process mechanism: full frontend compile, no runtime ROS; actual ESKF/header harness on tilted synthetic static/moving/reset cases, gate-off reference and frozen terrain raw IMU/point-time prefix. Preserve negative cases and resource/build evidence.
- [x] Unit/source audit: given corrected164 premise is not independently re-measured; only local config/source evidence, else unknown. No numerical unit guessing or rescaling historical payloads. Explicit internal normalized-acc bias domain.
- [x] Primary-source LI-Init evaluation and separate result-review/config-export entry: frame/properSE3/time sign/provenance/excitation/stability requirements; refuse unverified production export, no upstream algorithm claims without execution.
- [x] Final review/verification: scoped tests green; actual integrated outputs inspected; original source tree/status/hash unchanged; limitations and all negative/missing evidence retained. No unrelated suite polling or commits.

## Rulings / progress ledger

- Ruling: no ROS publishers even in simulation — actual Process/ESKF harness is in-process/offline; reuse frozen terrain sensor values/time without invoking its ROS runner.
- Ruling: initial stationary gate is not a replacement for motion-excited extrinsic/time calibration; constant linear acceleration can evade an IMU-only stationarity test. Thresholds are experiment parameters, not hardware-calibrated production defaults.
- Ruling: initialize only after consuming an entire synchronized IMU group; do not latch readiness midway through a group and then ignore subsequent motion.
- Ruling: preserve gravity in camera_init rather than silently rotate world Z; stationary tilted starts must pass and remain tilted.
- Verification after independent review fixes: scoped25/25tests PASS; isolation_v8 complete node+harness build0; mechanism_v5 nine scenarios satisfy expectations; original parent/nested source clean. See artifacts/loop_v4_20261021/frontend/RESULT_v3_zh.txt. All earlier RESULT/evidence versions are superseded historical evidence.
- Review fixes: raw adapter rejection invalidates/skips an entire synchronized initialization group; v4-only offset validation stays inside enabled branch; actual effective input/gate/provenance/evidence config is written beside every enabled gate trace. Two actual REDs (219 samples wrongly initialized after rawNaN/negative timestamp drop) are preserved in review_red_v1.
- Final independent fresh review by loop_geometry_review_1020: original three Important items plus hidden positive raw backtrack notifier gap closed; v8 fix/evidence confirmed, no new Important found. No actual ROS callback/transport execution claim. Upstream LI-Init algorithm status remains NOT_RUN, not a missing implementation test disguised as algorithm success.
- v9 extension: root required hard state consistency before map release. Actual v8 RED18 invalid/inconsistent states +3stale-output cases retained, with4legacy/fresh controls. v9 prechecks raw state/extrinsic/P/noise and candidate residuals before actual KF/P mutation; enabled initialization clears output before empty-IMU return. Full build0, original25 tests/9mechanisms PASS, state_GREEN_v2 27/27 PASS including explicit threshold configurability; first26/27 fixture failure retained. Fresh successful state CSVs byte-identical to v8. Reviewer confirmed no new Critical/Important in final source; final evidence version is v4, not v3.
