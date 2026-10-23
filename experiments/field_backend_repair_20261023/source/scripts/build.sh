#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
# shellcheck source=scripts/lib/a2w_common.sh
source "$repository_root/scripts/lib/a2w_common.sh"

ros_distro=humble
ros_setup=${A2W_ROS_SETUP:-"/opt/ros/$ros_distro/setup.bash"}
git_bin=${GIT_BIN:-git}
colcon_bin=${COLCON_BIN:-/usr/bin/colcon}
python_executable=${A2W_PYTHON_EXECUTABLE:-/usr/bin/python3}

a2w_require_file "$ros_setup" "ROS setup"
a2w_require_file "$python_executable" "ROS-compatible Python interpreter"
command -v "$git_bin" >/dev/null 2>&1 || a2w_fail "git command not found: $git_bin"
command -v "$colcon_bin" >/dev/null 2>&1 || a2w_fail "colcon command not found: $colcon_bin"

set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u

if [[ -e $repository_root/.git ]]; then
    "$git_bin" -C "$repository_root" submodule update --init --recursive
fi
cd "$repository_root"
deps_prefix=${A2W_DEPS_PREFIX:-"$repository_root/.deps/install"}
if [[ -d $deps_prefix ]]; then
    export CMAKE_PREFIX_PATH="$deps_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$deps_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export CMAKE_BUILD_PARALLEL_LEVEL=${A2W_BUILD_JOBS:-2}
export MAKEFLAGS="-j${A2W_BUILD_JOBS:-2}"
if [[ ${A2W_SKIP_ENV_CHECK:-0} != 1 ]]; then
    "$repository_root/scripts/check_environment.sh"
fi
exec "$colcon_bin" --log-base "${A2W_LOG_BASE:-log}" build --symlink-install \
    --base-paths src --executor sequential \
    --build-base "${A2W_BUILD_BASE:-build}" --install-base "${A2W_INSTALL_BASE:-install}" \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    "-DPython3_EXECUTABLE=$python_executable" \
    "-DPYTHON_EXECUTABLE=$python_executable" \
    "$@"
