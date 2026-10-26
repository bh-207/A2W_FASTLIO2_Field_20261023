# Z motion-reference implementation plan

Goal: explain fixed-up ground shape and the rejected 150–215 s inertial residual using controlled independent LiDAR reference ablations.
Architecture: isolated Python registration/audit; reuses immutable raw preprocessing and original joint solver. No frontend/filter changes unless a real candidate passes all existing gates.
Spec: user message 2026-10-24; fixed R=I,T=[-0.00862,0.00914,0.03916],false,offset=0; no Q/units/epoch or acceptance changes.

1. Freeze local state and B009, retain old 16.3 s holdout failure; compare original stride3 with stride1 pair ICP.
2. Add default-off bounded scan-to-local-submap registration with causal scan provenance, same voxel/objective; test transform composition and finite map/invalid input first. Compare same prediction and per-pair geometry/IMU intervals; identify errors before changing compensation.
3. Decompose gyro vs independent rotation, force using LiDAR attitude, and finite-displacement integration residual at 0.5/1/2/5 s horizons. Preserve fixed g/ba/bg to isolate reference effects; then refit with unchanged solver/gates.
4. Only if supported, single-factor re-deskew variant, separate from scan frequency and submap choice. Choose training on motion/geometry alone; keep old holdout locked. Unknown/ambiguous solutions stay rejected.
5. Compute full-XYZ fixed-anchor chord deviations and segment angles, plus fixed-up heights/normals; no point modification or flattening.
6. Real accepted candidate => existing prepare/real commit path, full input build+map+repeat+frozen geometry. If none pass, no fallback map duplication; document exact first failing gate and concrete minimum observations.
7. Run registration/math regression, evidence/source-integrity checks; make Chinese HTML with visual causal comparisons, reproducible commands, and explicit completion boundaries.

Quality gates frozen before new results: original overlap .55, plane RMS .15m, existing init uncertainty<=1deg, holdout<=.5m/.05rad, training consistency must be reported and cannot be hidden by rank. Submap systematic correlations are not certified by a naive ICP Hessian.
Review: wrong map frame composition, future-scan leakage, changing compensation together with neighborhood, treating FAST-LIO comparison as truth, double-counted submap information, last scan boundary.
No automatic commit/upload, no production install or source overwrite.
