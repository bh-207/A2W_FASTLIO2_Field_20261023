#!/usr/bin/env bash
set -euo pipefail
repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
source /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 22.04 ]] || {
    printf 'Supported host: Ubuntu 22.04 (Humble), found %s %s\n' "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2; exit 1;
}
case $(uname -m) in x86_64|aarch64) ;; *) printf 'Unsupported CPU architecture\n' >&2; exit 1;; esac
for executable in git cmake g++ pkg-config /usr/bin/python3 colcon rosdep; do
    command -v "$executable" >/dev/null || { printf 'Missing dependency: %s\n' "$executable" >&2; exit 1; }
done
ros_setup=${A2W_ROS_SETUP:-/opt/ros/humble/setup.bash}
[[ -f $ros_setup ]] || { printf 'ROS Humble missing; run ./setup.sh --install-system\n' >&2; exit 1; }
set +u
source "$ros_setup"
set -u
export CMAKE_PREFIX_PATH="${A2W_DEPS_PREFIX:-$repository_root/.deps/install}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
/usr/bin/python3 -s -c 'import sys,numpy,yaml; assert sys.version_info[:2] == (3,10); print("Python",sys.version.split()[0],"NumPy",numpy.__version__)'
for package in rclcpp pcl_conversions rosidl_default_generators rosidl_generator_dds_idl nav2_mppi_controller nav2_smac_planner nav2_collision_monitor pointcloud_to_laserscan rviz2; do
    ros2 pkg prefix "$package" >/dev/null || { printf 'Missing ROS package: %s\n' "$package" >&2; exit 1; }
done
probe=$(mktemp -d /tmp/a2w-dependency-check.XXXXXX)
if ! cmake -S "$repository_root/cmake/dependency_probe" -B "$probe" -DA2W_CHECK_ALL=ON >"$probe/check.log" 2>&1; then
    tail -n 25 "$probe/check.log" >&2
    printf 'Dependency check failed; details: %s/check.log\n' "$probe" >&2
    exit 1
fi
if [[ -e $repository_root/.git ]]; then
    git -C "$repository_root" submodule status --recursive
    if git -C "$repository_root" submodule status --recursive | grep -qE '^[-+U]'; then
        printf 'Submodule missing or different from pin; run ./setup.sh\n' >&2; exit 1
    fi
fi
for path in src/FAST_LIO_Hesai/include/ikd-Tree/ikd_Tree.h third_party/TEASER-plusplus/teaser/src/registration.cc third_party/pmc/pmc_graph.cpp; do
    [[ -f $repository_root/$path ]] || { printf 'Missing pinned source: %s\n' "$path" >&2; exit 1; }
done
printf 'PASS: Ubuntu 22.04 / %s / Humble / C++17 dependencies / GTSAM 4.1.1\n' "$(uname -m)"
