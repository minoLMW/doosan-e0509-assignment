#!/usr/bin/env bash
set -euo pipefail

SESSION="doosan_assign"

# setup.bash가 unset 변수에 민감할 수 있어 안전장치
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

# 이미 떠있으면 안내만 하고 종료
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[start] tmux session '$SESSION' already exists."
  echo "Attach: tmux attach -t $SESSION"
  exit 0
fi

# 혹시 남아있는 프로세스가 있으면 정리(안전)
pkill -f "assignment_status_node" >/dev/null 2>&1 || true
pkill -f "assignment_sequence_move" >/dev/null 2>&1 || true

# 공통 환경
ROS_SETUP_CMD='export ROS_DOMAIN_ID=0; source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash'

tmux new-session -d -s "$SESSION" -n LAUNCH

# 1) LAUNCH: MoveIt2 demo (RViz 포함)
tmux send-keys -t "$SESSION:LAUNCH" "$ROS_SETUP_CMD; ros2 launch dsr_moveit_config_e0509 demo.launch.py" C-m

# 2) CONTROL: controller spawner (이미 active면 무시)
tmux new-window -t "$SESSION" -n CONTROL
tmux send-keys -t "$SESSION:CONTROL" \
"$ROS_SETUP_CMD; ros2 run controller_manager spawner dsr_moveit_controller --controller-manager /controller_manager || true; \
echo '[CONTROL] done (ignore if already active)'; bash" C-m

# 3) STATUS: assignment_status_node
tmux new-window -t "$SESSION" -n STATUS
tmux send-keys -t "$SESSION:STATUS" "$ROS_SETUP_CMD; ros2 run my_ros2_assignment assignment_status_node" C-m

# 4) GUI: PyQt5 GUI 실행
tmux new-window -t "$SESSION" -n GUI
tmux send-keys -t "$SESSION:GUI" "$ROS_SETUP_CMD; python3 ~/ros2_ws/src/my_ros2_assignment/my_ros2_assignment/assignment_gui.py" C-m

# 5) MONITOR: 상태/이벤트 모니터
tmux new-window -t "$SESSION" -n MONITOR
tmux send-keys -t "$SESSION:MONITOR" \
"$ROS_SETUP_CMD; echo '--- /assignment/status (once) ---'; ros2 topic echo /assignment/status --once; echo; \
echo '--- /assignment/state_event (live) ---'; ros2 topic echo /assignment/state_event" C-m

tmux select-window -t "$SESSION:LAUNCH"

echo "[start] started tmux session '$SESSION'"
echo "Attach: tmux attach -t $SESSION"
echo "Windows: LAUNCH / CONTROL / STATUS / GUI / MONITOR"
