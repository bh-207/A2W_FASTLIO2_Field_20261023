#!/usr/bin/env bash
set -uo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
# shellcheck source=scripts/lib/a2w_common.sh
source "$repository_root/scripts/lib/a2w_common.sh"

ros2_bin=${ROS2_BIN:-ros2}
service_timeout_s=${A2W_STOP_TIMEOUT_S:-3}
failures=0

if ! a2w_source_ros_environment "$repository_root"; then
    exit 1
fi

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
runtime_cyclonedds="$repository_root/log/fast_lio_runtime/cyclonedds.xml"
if [[ -z ${CYCLONEDDS_URI:-} && -f $runtime_cyclonedds ]]; then
    export CYCLONEDDS_URI="file://$runtime_cyclonedds"
fi

call_stop_service() {
    local description=$1
    shift
    if timeout "$service_timeout_s" "$ros2_bin" "$@"; then
        printf 'A2W STOP: %s\n' "$description"
    else
        printf 'A2W STOP WARNING: %s failed or timed out\n' "$description" >&2
        failures=$((failures + 1))
    fi
}

printf '%s\n' 'A2W STOP: issuing immediate software stop...'

# Disable the final hardware boundary first. Its SetBool(false) callback sends
# the verified Unitree StopMove API request (api_id=1003) before responding.
call_stop_service 'StopMove sent; hardware output disabled' \
    service call /a2w_motion_bridge/enable std_srvs/srv/SetBool '{data: false}'

# Prevent any later Nav2 command from reaching the bridge.
call_stop_service 'navigation velocity gate disabled' \
    service call /navigation/enable_motion std_srvs/srv/SetBool '{data: false}'

# A zero UUID and zero timestamp cancel every active NavigateToPose goal.
call_stop_service 'all NavigateToPose goals canceled' \
    service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'

# Continuous sampled routes use FollowPath directly, not NavigateToPose.
call_stop_service 'all FollowPath goals canceled' \
    service call /follow_path/_action/cancel_goal action_msgs/srv/CancelGoal '{}'

if ((failures > 0)); then
    printf '%s\n' \
        'A2W STOP INCOMPLETE: use the physical/remote emergency stop immediately.' >&2
    exit 1
fi

printf '%s\n' 'A2W STOP COMPLETE: robot motion output is disabled.'
