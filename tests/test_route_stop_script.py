"""Stop ordering is observable without any ROS graph or hardware."""
import os
from pathlib import Path
import subprocess


def test_stop_disables_hardware_then_guard_then_both_action_types(tmp_path):
    root = Path(__file__).parents[1]
    setup = tmp_path / 'setup.bash'
    setup.write_text(':\n')
    record = tmp_path / 'calls.txt'
    fake = tmp_path / 'ros2'
    fake.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$CALL_RECORD"\n')
    fake.chmod(0o755)
    env = dict(os.environ, ROS2_BIN=str(fake), CALL_RECORD=str(record),
               A2W_ROS_SETUP=str(setup), A2W_INSTALL_SETUP=str(setup))
    result = subprocess.run(['bash', str(root/'scripts/a2w_stop_now.sh')],
                            env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    calls = record.read_text().splitlines()
    assert [line.split()[2] for line in calls] == [
        '/a2w_motion_bridge/enable', '/navigation/enable_motion',
        '/navigate_to_pose/_action/cancel_goal', '/follow_path/_action/cancel_goal']
