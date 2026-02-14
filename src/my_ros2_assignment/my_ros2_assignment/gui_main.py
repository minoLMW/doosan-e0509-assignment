from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QLineEdit, QPushButton, QTextEdit, QCheckBox, QListWidget
)
from PyQt5.QtCore import QTimer

from .workers import DummyMoveWorker

class MainWindow(QWidget):
    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.setWindowTitle("my_ros2_assignment")
        self.resize(1000, 600)

        self.targets = []
        self.stop_flag = False
        self.worker = None

        root = QHBoxLayout(self)

        # ===== Left: Control =====
        left = QVBoxLayout()
        root.addLayout(left, 1)

        gb_ctrl = QGroupBox("Control")
        left.addWidget(gb_ctrl)
        ctrl = QVBoxLayout(gb_ctrl)

        self.chk_abs = QCheckBox("Absolute")
        self.chk_rel = QCheckBox("Relative")
        self.chk_abs.setChecked(True)

        # 둘 중 하나만 선택되게
        self.chk_abs.toggled.connect(self._on_abs_toggle)
        self.chk_rel.toggled.connect(self._on_rel_toggle)

        ctrl.addWidget(self.chk_abs)
        ctrl.addWidget(self.chk_rel)

        self.edit_x = QLineEdit(); self.edit_x.setPlaceholderText("X")
        self.edit_y = QLineEdit(); self.edit_y.setPlaceholderText("Y")
        self.edit_z = QLineEdit(); self.edit_z.setPlaceholderText("Z")
        ctrl.addWidget(self.edit_x)
        ctrl.addWidget(self.edit_y)
        ctrl.addWidget(self.edit_z)

        self.edit_vel = QLineEdit(); self.edit_vel.setPlaceholderText("Speed (0~1)")
        self.edit_acc = QLineEdit(); self.edit_acc.setPlaceholderText("Accel (0~1)")
        ctrl.addWidget(self.edit_vel)
        ctrl.addWidget(self.edit_acc)

        self.btn_add = QPushButton("Add Target")
        self.btn_exec = QPushButton("Execute")
        ctrl.addWidget(self.btn_add)
        ctrl.addWidget(self.btn_exec)

        self.target_list = QListWidget()
        left.addWidget(QLabel("Targets (queue)"))
        left.addWidget(self.target_list, 1)

        self.btn_add.clicked.connect(self.on_add_target)
        self.btn_exec.clicked.connect(self.on_execute)

        # ===== Right: Status =====
        right = QVBoxLayout()
        root.addLayout(right, 1)

        gb_stat = QGroupBox("Status")
        right.addWidget(gb_stat)
        stat = QVBoxLayout(gb_stat)

        self.lbl_conn = QLabel("Robot: -")
        self.lbl_state = QLabel("Motion: -")
        stat.addWidget(self.lbl_conn)
        stat.addWidget(self.lbl_state)

        right.addWidget(QLabel("Log"))
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        right.addWidget(self.txt_log, 1)

        # UI update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start(200)

    def _on_abs_toggle(self, checked):
        if checked:
            self.chk_rel.setChecked(False)

    def _on_rel_toggle(self, checked):
        if checked:
            self.chk_abs.setChecked(False)

    def on_add_target(self):
        try:
            x = float(self.edit_x.text())
            y = float(self.edit_y.text())
            z = float(self.edit_z.text())
        except ValueError:
            self.txt_log.append("Invalid XYZ")
            return

        self.targets.append((x, y, z))
        self.target_list.addItem(f"({x:.3f}, {y:.3f}, {z:.3f})")

    def on_execute(self):
        if not self.targets:
            self.txt_log.append("No targets to execute")
            return

        # 버튼 동작은 별도 스레드 요구 충족
        self.btn_exec.setEnabled(False)
        self.worker = DummyMoveWorker(self.backend)
        self.worker.finished_signal.connect(self._on_worker_done)
        self.worker.start()

        self.txt_log.append("Execute started (dummy worker)")

    def _on_worker_done(self, msg):
        self.txt_log.append(f"Execute finished: {msg}")
        self.btn_exec.setEnabled(True)

    def refresh_status(self):
        snap = self.backend.get_snapshot()
        self.lbl_conn.setText(f"Robot: {'Connected' if snap['connected'] else 'Disconnected'}")
        self.lbl_state.setText(f"Motion: {snap['motion_state']}")
        self.txt_log.setPlainText(snap["log"])
