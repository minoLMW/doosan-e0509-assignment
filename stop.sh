#!/usr/bin/env bash
set -eo pipefail

SESSION="doosan_assign"

export ROS_DOMAIN_ID=0
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 1) stop 서비스 시도
ros2 service call /assignment/stop std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true

# 2) tmux 세션 kill
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
  echo "[stop] killed tmux session '$SESSION'"
else
  echo "[stop] tmux session '$SESSION' not found (already stopped)"
fi

# 3) 남은 프로세스 정리
pkill -f "ros2 launch dsr_moveit_config_e0509 demo.launch.py" >/dev/null 2>&1 || true
pkill -f "assignment_status_node" >/dev/null 2>&1 || true
pkill -f "assignment_sequence_move" >/dev/null 2>&1 || true
pkill -f "assignment_gui.py" >/dev/null 2>&1 || true
pkill -f "rviz2" >/dev/null 2>&1 || true

echo "[stop] done"
