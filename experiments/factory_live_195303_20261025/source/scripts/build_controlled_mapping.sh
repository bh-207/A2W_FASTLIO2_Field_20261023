#!/usr/bin/env bash
set -euo pipefail
repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
ros_setup=${A2W_ROS_SETUP:-/opt/ros/humble/setup.bash}
# Child-process isolation only: leave the user's shell/environment untouched.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source "$ros_setup"
set -u
deps_prefix=${A2W_DEPS_PREFIX:-"$repository_root/.deps/install"}
export CMAKE_PREFIX_PATH="$deps_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$deps_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL=${A2W_BUILD_JOBS:-2}
export MAKEFLAGS="-j${A2W_BUILD_JOBS:-2}"
exec /usr/bin/python3 -s "$repository_root/scripts/controlled_mapping_build.py" "$@"
