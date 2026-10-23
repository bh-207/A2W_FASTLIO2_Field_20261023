#!/usr/bin/env bash
# Environment of this child only; never launches/configures the JT128 driver.
set -euo pipefail
repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
controlled_root=${A2W_CONTROLLED_BUILD_ROOT:-"$repository_root/.controlled_mapping"}
[[ $controlled_root == /* ]] || controlled_root="$repository_root/$controlled_root"
if [[ -n ${LD_PRELOAD:-} || -n ${A2W_FROZEN_MATCHER_TIME:-} ]]; then
    printf 'Refusing inherited LD_PRELOAD/frozen matcher time in controlled capture; unset them explicitly.\n' >&2
    exit 2
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source "${A2W_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ -f $controlled_root/backend_install/setup.bash ]]; then
    source "$controlled_root/backend_install/setup.bash"
fi
set -u
deps_prefix=${A2W_DEPS_PREFIX:-"$repository_root/.deps/install"}
if [[ -d $deps_prefix/lib ]]; then
    export LD_LIBRARY_PATH="$deps_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export A2W_CONTROLLED_BUILD_ROOT="$controlled_root"
exec /usr/bin/python3 -s "$repository_root/scripts/mapping_capture.py" "$@"
