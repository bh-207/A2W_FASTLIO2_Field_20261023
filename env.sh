#!/usr/bin/env bash
# Usage: source ./env.sh (also works after cloning to a directory with spaces).
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    printf 'Use: source %q\n' "$0" >&2
    exit 2
fi
a2w_env_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
a2w_ros_setup=${A2W_ROS_SETUP:-/opt/ros/humble/setup.bash}
a2w_install_base=${A2W_INSTALL_BASE:-install}
[[ $a2w_install_base == /* ]] || a2w_install_base="$a2w_env_root/$a2w_install_base"
a2w_install_setup=${A2W_INSTALL_SETUP:-"$a2w_install_base/setup.bash"}
if [[ ! -f $a2w_ros_setup || ! -f $a2w_install_setup ]]; then
    printf 'ROS/workspace setup missing; run ./setup.sh and ./build.sh first.\n' >&2
    return 1
fi
a2w_env_had_nounset=false
[[ $- != *u* ]] || a2w_env_had_nounset=true
set +u
source "$a2w_ros_setup"
source "$a2w_install_setup"
export A2W_INSTALL_SETUP="$a2w_install_setup"
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
a2w_deps_prefix=${A2W_DEPS_PREFIX:-"$a2w_env_root/.deps/install"}
if [[ -d $a2w_deps_prefix ]]; then
    export CMAKE_PREFIX_PATH="$a2w_deps_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$a2w_deps_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
[[ $a2w_env_had_nounset != true ]] || set -u
unset a2w_env_root a2w_ros_setup a2w_install_setup a2w_env_had_nounset a2w_deps_prefix a2w_install_base
