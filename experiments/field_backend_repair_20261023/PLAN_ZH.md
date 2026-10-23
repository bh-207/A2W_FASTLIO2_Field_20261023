# 828KF backend preservation and engineering repair implementation plan

> Execute the user-approved full offline repair scope; no robot, motion, production install or original-data writes.

Goal: repair field engineering defects, attribute graph-induced seams, build a protected candidate and verify geometry and full replay.

Canonical source: source/frontend and source/src. Build from CMake/colcon in captures/field_backend_repair_20261023_001; no old object or link.txt reuse. Original generated sources remain frozen.

## Independent tasks
- [x] Frontend lifecycle/required endpoint identity and correctly timed ROS covariance, red/green tests.
- [x] Exact loop ledger/reconstruction audit, new-measurement provenance where missing, eight combinations and repeats on frozen raw828.
- [x] Freeze eight-ROI evaluation, common point/cell references, repeated-run margins, raw/old/new tables and plots.
- [x] Root: new build identity, real child exit supervisor and ordered capture stop, code integration.

## Dependent tasks
- [x] Require all-three comparison against saved optimized before historical attribution; preserve irrecoverable precision gaps.
- [x] Select one evidence-led, default-off causal backend trial/protection candidate, with a rollback-safe graph and no ID blacklist or global arbitrary downweight.
- [x] Repeat candidate, build complete Bundle/PCD from original body scans and actual new graph; no flattening.
- [x] Original-rate isolated localhost Bag playback, initialization, processing, save/drain/stop and integer source accounting.
- [x] Review all tests and frozen assets, deliver Chinese report, plots, runnable command and field runbook.

## Quality gates
No source POST/callback omissions hidden by summary counts; no closure from mere PID disappearance; required identities tolerate extra debug subscribers; covariance assembly is before publish.
Frozen graph factors must keep full pose/direction/noise/robust model/order/scan IDs and unwhitened+whitened residuals. Original missing factors are not inferred from final poses.
Geometry uses all8 original ROIs, same body point identities and a fixed global gauge, free slopes, no local ICP; uncertainty and unknown masks retained. Candidate repeat is mandatory.

The supplied detailed task is the execution specification and authorization. No additional design approval is requested.

## Final gate outcomes
Engineering fixes and final20s integration: PASS_SCOPED_OFFLINE. Backend actual fault exit0 ->2 and supervisor fast paths verified.
Historical exact factor reproduction: FAILED missing exact historical measurements; new three-factor sets clearly labelled.
New factor attribution: all8 masks x3 independent registrations, fixed-factor repeat bit-identical; candidate repeated3.
Candidate geometry: FAIL wall125 P95 (+14.65cm), despite four main low-floor improvements9.18–12.78cm.
Full original-rate runs010/013: completed saved/drained/real exits0, but input/recording completeness FAIL.013 had concurrent large-Bag I/O confound.
Final014 patch identity only20s:4000IMU and189clouds exact, all POST handoff including tail exact, save/exit pass.
Pure algorithm latency/continuous internal queue depth, live network/clock/record_manager, ARM64, physical accuracy and navigation remain unverified.
No recommended geometry release or production switch. Full task acceptance remains incomplete at geometry and whole-rate input gates.
