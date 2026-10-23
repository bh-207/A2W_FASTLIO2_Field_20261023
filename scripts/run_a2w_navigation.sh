#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
pc_launcher=${A2W_PC_LAUNCHER:-"$repository_root/scripts/run_fastlio_jt128_pc.sh"}
unitree_ros_setup=${A2W_UNITREE_ROS_SETUP:-}

if [[ -n $unitree_ros_setup && ! -f $unitree_ros_setup ]]; then
    printf '%s\n' \
        'A2W Navigation: Unitree ROS setup not found; set A2W_UNITREE_ROS_SETUP.' >&2
    exit 1
fi
if [[ -n $unitree_ros_setup ]]; then
    set +u
    source "$unitree_ros_setup"
    set -u
fi

if [[ $# -lt 1 || $1 == *:=* ]]; then
    printf 'Usage: %s MAP_BUNDLE_PATH [NAVIGATION_MAP_YAML] [launch_argument:=value ...]\n' "$0" >&2
    exit 2
fi

map_bundle_path=$(realpath -e -- "$1")
shift
if [[ ! -d $map_bundle_path || ! -f $map_bundle_path/manifest.sha256 ||
      ! -f $map_bundle_path/metadata.yaml ]]; then
    printf 'A2W Navigation: invalid Map Bundle directory: %s\n' "$map_bundle_path" >&2
    exit 1
fi
if ! grep -Eq '^map_frame:[[:space:]]*map([[:space:]]*(#.*)?)?$' \
    "$map_bundle_path/metadata.yaml"; then
    printf 'A2W Navigation: Map Bundle frame is not map: %s\n' "$map_bundle_path" >&2
    exit 1
fi

if [[ $# -gt 0 && $1 != *:=* ]]; then
    navigation_map_yaml=$(realpath -e -- "$1")
    shift
else
    if [[ -f $map_bundle_path/navigation/map.yaml ]]; then
        navigation_map_yaml=$(realpath -e -- "$map_bundle_path/navigation/map.yaml")
    else
        navigation_map_yaml=$(realpath -e -- "$map_bundle_path/navigation/nav_map.yaml")
    fi
fi

"${A2W_PYTHON_EXECUTABLE:-/usr/bin/python3}" "$repository_root/scripts/validate_navigation_map.py" \
    "$map_bundle_path" "$navigation_map_yaml" >/dev/null

map_image=$(awk '$1 == "image:" {$1=""; sub(/^[[:space:]]+/, ""); print; exit}' \
    "$navigation_map_yaml")
map_image=${map_image#\"}
map_image=${map_image%\"}
if [[ -z $map_image ]]; then
    printf 'A2W Navigation: map YAML has no image entry: %s\n' "$navigation_map_yaml" >&2
    exit 1
fi
if [[ $map_image == /* ]]; then
    resolved_map_image=$map_image
else
    resolved_map_image=$(dirname "$navigation_map_yaml")/$map_image
fi
if [[ ! -f $resolved_map_image ]]; then
    printf 'A2W Navigation: map image not found: %s\n' "$resolved_map_image" >&2
    exit 1
fi

export A2W_BRINGUP_LAUNCH=navigation.launch.py
export A2W_SAVE_ARGUMENT_NAME=

# The one-command live-navigation profile. Explicit launch arguments supplied
# by the operator always win. Hardware output is prepared at launch, while the
# LocalizationGuard remains fail-closed until LOCALIZED pose data is fresh.
launch_arguments=("$@")
for default_argument in \
    "rviz:=true" \
    "autostart:=true" \
    "auto_enable_motion:=true" \
    "use_collision_monitor:=false" \
    "runtime_diagnostics:=true"
do
    argument_name=${default_argument%%:=*}
    argument_present=false
    for supplied_argument in "${launch_arguments[@]}"; do
        if [[ $supplied_argument == "$argument_name:="* ]]; then
            argument_present=true
            break
        fi
    done
    if [[ $argument_present == false ]]; then
        launch_arguments+=("$default_argument")
    fi
done

exec "$pc_launcher" \
    "map_bundle_path:=$map_bundle_path" \
    "navigation_map_yaml:=$navigation_map_yaml" \
    "${launch_arguments[@]}"
