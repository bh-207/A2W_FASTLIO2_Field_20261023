# Portable workspace and 2D map recovery implementation plan

**Goal:** Publish the latest complete development branch with reproducible builds and a discoverable offline 2D-map entry point.

**Architecture:** Preserve the existing packages and frontend submodule. Restore the existing occupancy exporter, reuse the previously developed parallel PMF filters, and make projection reference explicit. Install dependencies and build from repository-relative entry points.

**Tech stack:** Ubuntu 22.04, ROS 2 Humble, C++17/GCC 11, Python 3.10, PCL, Eigen, GTSAM, OpenMP, Nav2.

**Spec:** User request in the current session: prepare current valid code, exclude generated/private data, restore 2D conversion, verify and push the existing development branch.

## Constraints and review focus

- Keep original maps, calibration values and frontend algorithm unchanged.
- Keep all currently developed runtime modules; do not stage backups or experimental outputs.
- Never infer gravity from an unverified map; record a separate projection frame when axes change.
- Verify dependency pins and the staged checkout, not only the existing install directory.
- Check paths with spaces, absent Unitree overlays, missing submodules, unknown projection reference, and empty obstacle clouds.
- Work in the current branch as explicitly requested. Commit and push are already authorized; never force-push.

## Tasks

1. [x] Inventory active source, dependencies and historical converters; record baseline checks. Configure ignore rules without deleting local files.
2. [x] Restore offline 2D conversion: explicit reference, configurable height/obstacle processing, portable wrapper and regression tests. Reuse the existing PMF implementation.
3. [x] Provide dependency initialization/check/build/environment entry points; preserve required interfaces and models. Verify a clean exported source tree with isolated build/install directories.
4. [x] Update documentation, inspect the exact upload for secrets/generated files and fixed paths, run relevant tests and review. Release is published to `feature/a2w-navigation` by normal commit/push; the delivery message records the verified remote commit identity.

## Verification

- Existing `tests/run_tests.sh` baseline and final result.
- Projection tests: coordinate identity, known tilted reference, rejection without reference, input immutability, standard PGM/YAML, non-finite input and no-obstacle case.
- PMF filter tests and offline CLI smoke test; no ROS graph or robot motion required.
- Build from staged-source export with a clean ROS underlay and bundled source dependencies; run CTest/colcon tests with loopback DDS.
- Compare the original map manifest before/after; do not publish generated test maps.
- Inspect staged files for paths, secrets, binaries, missing resources and dependency closure; compare local/remote commit after push.

## Progress

- Found existing `a2w_nav_map` in commit `357c99f` and a parallel PMF converter in the previous external converter workspace.
- Existing checkout contains substantial uncommitted runtime development; preserve it in place and select valid source explicitly.
- Verified a source-only Release build of all 15 packages, including bundled Unitree interfaces and required model resources.
- Built pinned GTSAM from source and verified Mapping against that installation. Registered the existing Python package tests with pytest. Final loopback-only colcon summary: 509 tests, zero errors/failures/skips.
- Restored PMF and projection regression tests pass, including explicit-reference checks and failed-publication rollback. Original map manifest remains valid.
- Reviewed and corrected quoted boolean handling, output publication rollback, input/output collision protection and colcon interpreter selection. Actual staged source audit passes.
