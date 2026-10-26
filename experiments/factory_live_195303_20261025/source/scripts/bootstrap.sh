#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
# shellcheck source=scripts/lib/a2w_common.sh
source "$repository_root/scripts/lib/a2w_common.sh"

ros_distro=humble
ros_setup=${A2W_ROS_SETUP:-"/opt/ros/$ros_distro/setup.bash"}
git_bin=${GIT_BIN:-git}
rosdep_bin=${ROSDEP_BIN:-rosdep}

case ${1:-} in
    --check) exec "$repository_root/scripts/check_environment.sh";;
    --help|-h) printf 'Usage: ./setup.sh [--install-system|--check]\n'; exit 0;;
    --install-system)
        source /etc/os-release
        [[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 22.04 ]] || a2w_fail 'Requires Ubuntu 22.04'
        sudo apt-get update
        sudo apt-get install -y curl ca-certificates gnupg lsb-release software-properties-common
        sudo add-apt-repository -y universe
        if [[ ! -f $ros_setup ]]; then
            ros_key_temp=$(mktemp /tmp/a2w-ros-key.XXXXXX)
            curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$ros_key_temp"
            sudo install -m 644 "$ros_key_temp" /usr/share/keyrings/a2w-ros-archive-keyring.gpg
            printf 'deb [arch=%s signed-by=/usr/share/keyrings/a2w-ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\n' "$(dpkg --print-architecture)" | sudo tee /etc/apt/sources.list.d/a2w-ros2.list >/dev/null
            sudo apt-get update
        fi
        sudo apt-get install -y build-essential cmake git pkg-config libeigen3-dev libpcl-dev libtbb-dev \
            libyaml-cpp-dev libssl-dev libboost-all-dev python3-colcon-common-extensions python3-rosdep \
            python3-numpy python3-yaml python3-pytest ros-humble-desktop ros-humble-rmw-cyclonedds-cpp
        ;;
    '') ;;
    *) a2w_fail "Unknown option: $1"; exit 2;;
esac

a2w_require_file "$ros_setup" "ROS setup"
command -v "$git_bin" >/dev/null 2>&1 || a2w_fail "git command not found: $git_bin"
command -v "$rosdep_bin" >/dev/null 2>&1 || a2w_fail "rosdep command not found: $rosdep_bin"

set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u

"$git_bin" -C "$repository_root" submodule update --init --recursive
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo "$rosdep_bin" init
fi
if [[ $rosdep_bin == rosdep ]]; then
    "$rosdep_bin" update --rosdistro humble
fi
"$rosdep_bin" install --from-paths "$repository_root/src" --ignore-src -r -y --rosdistro humble --skip-keys gtsam
"$repository_root/scripts/install_gtsam.sh"

printf 'A2W Fast-LIO2 dependencies are ready.\n'
