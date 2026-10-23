#!/usr/bin/env bash
set -euo pipefail
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd -- "$task_dir/../.." && pwd)
export A2W_REPAIR_OUTPUT="${A2W_REPAIR_OUTPUT:-$workspace/captures/field_backend_repair_20261023_001}"
build_dir="$A2W_REPAIR_OUTPUT/build"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
for package in a2w_fastlio_common a2w_fastlio_msgs a2w_fastlio_map a2w_fastlio_mapping; do
  source "$build_dir/backend_install/$package/share/$package/local_setup.bash"
done
source "$build_dir/frontend_install/fast_lio/share/fast_lio/local_setup.bash"
set -u
export LD_LIBRARY_PATH="$workspace/.deps/install/lib:${LD_LIBRARY_PATH:-}"
export A2W_CAPTURE_ROOT="$A2W_REPAIR_OUTPUT"
exec "$@"
