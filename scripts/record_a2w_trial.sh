#!/usr/bin/env bash
# Records an existing ROS graph only; no driver, motion, launch or goal action.
set -euo pipefail
trial_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
set +u
source /opt/ros/humble/setup.bash
source "$trial_root/install/setup.bash"
set -u
export ROS_LOG_DIR="${ROS_LOG_DIR:-$trial_root/log/trial_recorder}"
exec /usr/bin/python3 "$trial_root/scripts/diagnostics/localization_trial.py" "$@"
