#!/usr/bin/env bash
# Source in each terminal. Reads local addresses only; never changes routes/time/driver.
field_ws=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
field_original_ws=$(cd -- "$field_ws/../../.." && pwd)
export A2W_REPAIR_OUTPUT="${A2W_REPAIR_OUTPUT:-$field_original_ws/captures/field_backend_repair_20261023_001}"
export A2W_CAPTURE_ROOT="$A2W_REPAIR_OUTPUT"
if [[ -n ${LD_PRELOAD:-} || -n ${A2W_FROZEN_MATCHER_TIME:-} ]]; then
    printf 'Unset LD_PRELOAD/A2W_FROZEN_MATCHER_TIME before field capture.\n' >&2
    return 2
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash || return
for field_package in a2w_fastlio_common a2w_fastlio_msgs a2w_fastlio_map a2w_fastlio_mapping; do
    source "$A2W_REPAIR_OUTPUT/build/backend_install/$field_package/share/$field_package/local_setup.bash" || return
done
source "$A2W_REPAIR_OUTPUT/build/frontend_install/fast_lio/share/fast_lio/local_setup.bash" || return
export LD_LIBRARY_PATH="$field_original_ws/.deps/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS2CLI_NO_DAEMON=1
if [[ -n ${1:-} ]]; then
    export SESSION=$1
    export CYCLONEDDS_URI="file://$SESSION/field_environment/cyclonedds.xml"
    export ROS_LOG_DIR="$SESSION/ros_logs" ROS_HOME="$SESSION/ros_home"
else
    source "$field_ws/scripts/lib/a2w_common.sh"
    export A2W_DDS_PEER=192.168.123.164
    export A2W_RUNTIME_DIR="$A2W_REPAIR_OUTPUT/network"
    field_selection=$(ip -o -4 addr show scope global | python3 "$field_ws/scripts/select_network_interface.py" --peer "$A2W_DDS_PEER") || {
        printf 'No direct robot-LAN IPv4. Connect the robot Ethernet first; refusing VPN/default-route fallback.\n' >&2
        return 2
    }
    read -r A2W_NETWORK_INTERFACE A2W_PC_IP <<<"$field_selection"
    export A2W_NETWORK_INTERFACE A2W_PC_IP
    a2w_prepare_cyclonedds "$field_ws" || return
    export ROS_LOG_DIR="$A2W_RUNTIME_DIR/ros_logs" ROS_HOME="$A2W_RUNTIME_DIR/ros_home"
    mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
fi
printf 'FIELD env: domain=%s localhost=%s DDS=%s\n' "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$CYCLONEDDS_URI"
