#!/usr/bin/env bash
set -euo pipefail
repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
deps_root=${A2W_DEPS_ROOT:-"$repository_root/.deps"}
deps_prefix=${A2W_DEPS_PREFIX:-"$deps_root/install"}
export CMAKE_PREFIX_PATH="$deps_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
mkdir -p "$deps_root"
case ${1:-} in ''|--from-source) ;; *) printf 'Usage: %s [--from-source]\n' "$0" >&2; exit 2;; esac
if [[ ${1:-} != --from-source ]] && cmake -S "$repository_root/cmake/dependency_probe" -B "$deps_root/gtsam_probe" >"$deps_root/gtsam_probe.log" 2>&1; then
    printf 'GTSAM 4.1.1 already available.\n'; exit 0
fi
gtsam_commit=69a3a75195b65356d6e56669e5199d325c7962c9
gtsam_source="$deps_root/src/gtsam"
if [[ ! -d $gtsam_source/.git ]]; then
    mkdir -p "$gtsam_source"
    git -C "$gtsam_source" init
    git -C "$gtsam_source" remote add origin https://github.com/borglab/gtsam.git
fi
git -C "$gtsam_source" fetch --depth 1 origin "$gtsam_commit"
git -C "$gtsam_source" checkout --detach "$gtsam_commit"
[[ $(git -C "$gtsam_source" rev-parse HEAD) == "$gtsam_commit" ]]
cmake -S "$gtsam_source" -B "$deps_root/build/gtsam" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$deps_prefix" \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_WITH_TBB=OFF
cmake --build "$deps_root/build/gtsam" --parallel "${A2W_BUILD_JOBS:-2}"
cmake --install "$deps_root/build/gtsam"
