#!/usr/bin/env python3
import os
import sys
import signal
import subprocess
from typing import Optional, Dict, Any, List

from PyQt5 import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from my_ros2_assignment_msgs.msg import AssignmentStatus


# =========================
# Helpers
# =========================
def ros_bash_cmd(cmd: str) -> List[str]:
    return [
        "bash",
        "-lc",
        "export ROS_DOMAIN_ID=0; "
        "source /opt/ros/humble/setup.bash; "
        "source ~/ros2_ws/install/setup.bash; "
        + cmd,
    ]


def f3(x: float) -> str:
    return f"{x:.3f}"


# =========================
# Status subscriber thread
# =========================
class StatusSubNode(Node):
    def __init__(self, on_status_cb):
        super().__init__("assignment_gui_status_sub")
        self._on_status_cb = on_status_cb
        self.create_subscription(AssignmentStatus, "/assignment/status", self._cb, 10)

    def _cb(self, msg: AssignmentStatus):
        data: Dict[str, Any] = {
            "move_group_available": bool(msg.move_group_available),
            "trajectory_controller_ready": bool(msg.trajectory_controller_ready),
            "controller_name": msg.controller_name,
            "motion_state_text": msg.motion_state_text,
            "is_moving": bool(msg.is_moving),
            "active_goal_id": msg.active_goal_id,
            "last_moveit_error_code": int(msg.last_moveit_error_code),
            "last_error_msg": msg.last_error_msg,
            "target_total": int(msg.target_total),
            "target_index": int(msg.target_index),
            "mode": msg.mode,
            "vel": float(msg.vel),
            "acc": float(msg.acc),
            "keep_orientation": bool(msg.keep_orientation),
            "joint_names": list(msg.joint_state.name),
            "joint_pos": list(msg.joint_state.position),
            "ee_frame": msg.ee_pose_base.header.frame_id,
            "ee_pos": (
                float(msg.ee_pose_base.pose.position.x),
                float(msg.ee_pose_base.pose.position.y),
                float(msg.ee_pose_base.pose.position.z),
            ),
            "ee_ori": (
                float(msg.ee_pose_base.pose.orientation.x),
                float(msg.ee_pose_base.pose.orientation.y),
                float(msg.ee_pose_base.pose.orientation.z),
                float(msg.ee_pose_base.pose.orientation.w),
            ),
            "recent_logs": list(msg.recent_logs),
        }
        self._on_status_cb(data)


class StatusThread(QtCore.QThread):
    status_signal = QtCore.pyqtSignal(dict)
    error_signal = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = True
        self._executor: Optional[SingleThreadedExecutor] = None
        self._node: Optional[StatusSubNode] = None

    def run(self):
        try:
            rclpy.init(args=None)
            self._executor = SingleThreadedExecutor()

            def on_status(data: dict):
                self.status_signal.emit(data)

            self._node = StatusSubNode(on_status)
            self._executor.add_node(self._node)

            while self._running and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)

        except Exception as e:
            self.error_signal.emit(f"[StatusThread] {e}")
        finally:
            try:
                if self._executor and self._node:
                    self._executor.remove_node(self._node)
                    self._node.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    def stop(self):
        self._running = False


# =========================
# Worker thread (run sequence)
# =========================
class RunWorker(QtCore.QThread):
    log_signal = QtCore.pyqtSignal(str)
    finished_signal = QtCore.pyqtSignal(int)

    def __init__(self, cmd_str: str, parent=None):
        super().__init__(parent)
        self.cmd_str = cmd_str
        self._proc: Optional[subprocess.Popen] = None

    def run(self):
        try:
            self.log_signal.emit(f"[RUN] {self.cmd_str}")
            self._proc = subprocess.Popen(
                ros_bash_cmd(self.cmd_str),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                preexec_fn=os.setsid,
            )

            assert self._proc.stdout is not None
            for line in self._proc.stdout:
                self.log_signal.emit(line.rstrip())

            rc = self._proc.wait()
            self.finished_signal.emit(rc)

        except Exception as e:
            self.log_signal.emit(f"[RUN][ERROR] {e}")
            self.finished_signal.emit(255)

    def request_stop(self):
        # 1) stop service
        try:
            subprocess.run(
                ros_bash_cmd('ros2 service call /assignment/stop std_srvs/srv/Trigger "{}"'),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.0,
            )
            self.log_signal.emit("[STOP] /assignment/stop called")
        except Exception:
            self.log_signal.emit("[STOP] /assignment/stop call failed/timeout (ignored)")

        # 2) SIGINT to process group
        if self._proc and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self.log_signal.emit("[STOP] Sent SIGINT to movegroup_sequence")
            except Exception:
                pass


# =========================
# Main GUI
# =========================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Doosan E0509 Assignment GUI (ROS2 Humble + MoveIt2)")
        self.resize(1200, 720)

        self.worker: Optional[RunWorker] = None
        self._prev_recent_logs: List[str] = []

        self._build_ui()

        self.status_thread = StatusThread()
        self.status_thread.status_signal.connect(self.on_status)
        self.status_thread.error_signal.connect(self.append_log)
        self.status_thread.start()

    def closeEvent(self, event):
        try:
            if self.worker and self.worker.isRunning():
                self.worker.request_stop()
                self.worker.wait(1000)
        except Exception:
            pass
        try:
            self.status_thread.stop()
            self.status_thread.wait(1500)
        except Exception:
            pass
        super().closeEvent(event)

    # ---- UI ----
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # Left panel
        left = QtWidgets.QGroupBox("Control")
        left.setMinimumWidth(420)
        left_l = QtWidgets.QVBoxLayout(left)

        # Mode
        mode_box = QtWidgets.QGroupBox("Mode")
        mode_l = QtWidgets.QHBoxLayout(mode_box)
        self.rb_abs = QtWidgets.QRadioButton("ABS")
        self.rb_rel = QtWidgets.QRadioButton("REL")
        self.rb_abs.setChecked(True)
        mode_l.addWidget(self.rb_abs)
        mode_l.addWidget(self.rb_rel)
        mode_l.addStretch(1)

        # Targets
        targets_box = QtWidgets.QGroupBox("Targets (x,y,z; x,y,z; ...)")
        targets_l = QtWidgets.QVBoxLayout(targets_box)
        self.ed_targets = QtWidgets.QPlainTextEdit()
        self.ed_targets.setPlaceholderText('Example: 0.35,0.00,0.45; 0.30,0.10,0.40')
        self.ed_targets.setFixedHeight(80)
        targets_l.addWidget(self.ed_targets)

        targets_btn_row = QtWidgets.QHBoxLayout()
        self.btn_fill_sample = QtWidgets.QPushButton("Fill sample")
        self.btn_clear_targets = QtWidgets.QPushButton("Clear")
        targets_btn_row.addWidget(self.btn_fill_sample)
        targets_btn_row.addWidget(self.btn_clear_targets)
        targets_btn_row.addStretch(1)
        targets_l.addLayout(targets_btn_row)

        self.btn_fill_sample.clicked.connect(self.fill_sample_targets)
        self.btn_clear_targets.clicked.connect(lambda: self.ed_targets.setPlainText(""))

        # Vel/Acc
        va_box = QtWidgets.QGroupBox("Velocity/Acceleration scaling (0.0 ~ 1.0)")
        va_l = QtWidgets.QFormLayout(va_box)
        self.ed_vel = QtWidgets.QLineEdit("0.03")
        self.ed_acc = QtWidgets.QLineEdit("0.0")
        va_l.addRow("vel", self.ed_vel)
        va_l.addRow("acc", self.ed_acc)

        # Options
        opt_box = QtWidgets.QGroupBox("Options")
        opt_l = QtWidgets.QVBoxLayout(opt_box)
        self.cb_keep_ori = QtWidgets.QCheckBox("keep_orientation (try pos+ori then fallback to pos-only)")
        self.cb_plan_only = QtWidgets.QCheckBox("plan_only (no execution)")
        opt_l.addWidget(self.cb_keep_ori)
        opt_l.addWidget(self.cb_plan_only)

        # Buttons
        btn_row = QtWidgets.QHBoxLayout()
        self.btn_run = QtWidgets.QPushButton("RUN")
        self.btn_stop = QtWidgets.QPushButton("STOP")
        self.btn_stop.setEnabled(False)
        btn_row.addWidget(self.btn_run)
        btn_row.addWidget(self.btn_stop)

        self.btn_run.clicked.connect(self.on_run)
        self.btn_stop.clicked.connect(self.on_stop)

        left_l.addWidget(mode_box)
        left_l.addWidget(targets_box)
        left_l.addWidget(va_box)
        left_l.addWidget(opt_box)
        left_l.addLayout(btn_row)
        left_l.addStretch(1)

        # Right panel
        right = QtWidgets.QGroupBox("Status / Monitor")
        right_l = QtWidgets.QVBoxLayout(right)

        top_grid = QtWidgets.QGridLayout()
        self.lb_conn = QtWidgets.QLabel("move_group: - , controller: -")
        self.lb_motion = QtWidgets.QLabel("motion: -")
        self.lb_goal = QtWidgets.QLabel("goal_id: -")
        self.lb_target = QtWidgets.QLabel("target: -")
        top_grid.addWidget(QtWidgets.QLabel("Connection:"), 0, 0)
        top_grid.addWidget(self.lb_conn, 0, 1)
        top_grid.addWidget(QtWidgets.QLabel("Motion:"), 1, 0)
        top_grid.addWidget(self.lb_motion, 1, 1)
        top_grid.addWidget(QtWidgets.QLabel("Goal:"), 2, 0)
        top_grid.addWidget(self.lb_goal, 2, 1)
        top_grid.addWidget(QtWidgets.QLabel("Target:"), 3, 0)
        top_grid.addWidget(self.lb_target, 3, 1)

        joint_box = QtWidgets.QGroupBox("Joint (rad)")
        joint_l = QtWidgets.QVBoxLayout(joint_box)
        self.joint_table = QtWidgets.QTableWidget(6, 2)
        self.joint_table.setHorizontalHeaderLabels(["Joint", "Position"])
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.joint_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        for i, jn in enumerate(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]):
            self.joint_table.setItem(i, 0, QtWidgets.QTableWidgetItem(jn))
            self.joint_table.setItem(i, 1, QtWidgets.QTableWidgetItem("-"))
        self.joint_table.resizeColumnsToContents()
        joint_l.addWidget(self.joint_table)

        pose_box = QtWidgets.QGroupBox("EE Pose (base_link)")
        pose_l = QtWidgets.QGridLayout(pose_box)
        self.lb_pos = QtWidgets.QLabel("pos: -")
        self.lb_ori = QtWidgets.QLabel("ori: -")
        pose_l.addWidget(self.lb_pos, 0, 0)
        pose_l.addWidget(self.lb_ori, 1, 0)

        log_box = QtWidgets.QGroupBox("Logs")
        log_l = QtWidgets.QVBoxLayout(log_box)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(800)
        log_l.addWidget(self.log_view)

        right_l.addLayout(top_grid)
        right_l.addWidget(joint_box)
        right_l.addWidget(pose_box)
        right_l.addWidget(log_box)

        root.addWidget(left)
        root.addWidget(right, stretch=1)

        # store for lock/unlock
        self._left_controls = [
            self.rb_abs,
            self.rb_rel,
            self.ed_targets,
            self.btn_fill_sample,
            self.btn_clear_targets,
            self.ed_vel,
            self.ed_acc,
            self.cb_keep_ori,
            self.cb_plan_only,
        ]

    # ---- UX helpers ----
    def set_controls_enabled(self, enabled: bool):
        for w in self._left_controls:
            w.setEnabled(enabled)

    def fill_sample_targets(self):
        self.ed_targets.setPlainText("0.35,0.00,0.45;0.30,0.10,0.40")

    # ---- Actions ----
    def append_log(self, line: str):
        self.log_view.appendPlainText(line)

    def on_run(self):
        if self.worker and self.worker.isRunning():
            return

        targets = self.ed_targets.toPlainText().strip().replace("\n", " ")
        if not targets:
            QtWidgets.QMessageBox.warning(self, "Input required", "Targets is empty.\nPlease enter x,y,z; x,y,z; ...")
            return

        mode = "abs" if self.rb_abs.isChecked() else "rel"

        try:
            vel = float(self.ed_vel.text().strip())
            acc = float(self.ed_acc.text().strip())
        except Exception:
            QtWidgets.QMessageBox.warning(self, "Invalid input", "vel/acc must be numbers.")
            return

        keep_ori = self.cb_keep_ori.isChecked()
        plan_only = self.cb_plan_only.isChecked()

        cmd = (
            'ros2 run my_ros2_assignment movegroup_sequence -- '
            f'--mode {mode} '
            f'--targets "{targets}" '
            f'--vel {vel} --acc {acc} '
        )
        if keep_ori:
            cmd += "--keep-orientation "
        if plan_only:
            cmd += "--plan-only "

        self.worker = RunWorker(cmd)
        self.worker.log_signal.connect(self.append_log)
        self.worker.finished_signal.connect(self.on_worker_done)

        self.append_log("[GUI] RUN pressed.")
        self.set_controls_enabled(False)
        self.btn_run.setEnabled(False)
        self.btn_stop.setEnabled(True)

        self.worker.start()

    def on_stop(self):
        if self.worker and self.worker.isRunning():
            self.append_log("[GUI] STOP pressed.")
            self.btn_stop.setEnabled(False)  # prevent double-click spam
            self.worker.request_stop()
        else:
            self.append_log("[GUI] STOP ignored (no active worker)")

    def on_worker_done(self, rc: int):
        self.append_log(f"[GUI] Worker finished rc={rc}")
        self.btn_run.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.set_controls_enabled(True)

    # ---- Status update ----
    def on_status(self, st: dict):
        mg = "OK" if st["move_group_available"] else "DOWN"
        ct = "OK" if st["trajectory_controller_ready"] else "DOWN"
        self.lb_conn.setText(f"move_group={mg}, controller={ct} ({st['controller_name']})")

        moving = "MOVING" if st["is_moving"] else "STOPPED/IDLE"
        self.lb_motion.setText(
            f"{st['motion_state_text']} | {moving} | err={st['last_moveit_error_code']} msg={st['last_error_msg']}"
        )

        self.lb_goal.setText(st["active_goal_id"] if st["active_goal_id"] else "-")
        self.lb_target.setText(
            f"{st['target_index']}/{st['target_total']} | mode={st['mode']} vel={st['vel']:.3f} acc={st['acc']:.3f} keep_ori={st['keep_orientation']}"
        )

        name_to_pos = {n: p for n, p in zip(st["joint_names"], st["joint_pos"])}
        for i, jn in enumerate(["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]):
            val = name_to_pos.get(jn, None)
            self.joint_table.item(i, 1).setText(f"{val:.6f}" if val is not None else "-")

        x, y, z = st["ee_pos"]
        ox, oy, oz, ow = st["ee_ori"]
        self.lb_pos.setText(f"pos: x={f3(x)} y={f3(y)} z={f3(z)}")
        self.lb_ori.setText(f"ori: x={f3(ox)} y={f3(oy)} z={f3(oz)} w={f3(ow)}")

        # Logs: append only new lines
        logs = st.get("recent_logs", []) or []
        if logs:
            prev = self._prev_recent_logs
            if len(logs) >= len(prev) and logs[: len(prev)] == prev:
                new_lines = logs[len(prev) :]
            else:
                self.log_view.clear()
                new_lines = logs

            for line in new_lines:
                self.append_log(f"[STATUS] {line}")

            self._prev_recent_logs = logs


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
