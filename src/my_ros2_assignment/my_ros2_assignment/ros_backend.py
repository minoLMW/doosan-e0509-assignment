import time
import threading
import rclpy
from rclpy.node import Node

class RosBackend(Node):
    def __init__(self):
        super().__init__("my_ros2_assignment_backend")
        self._lock = threading.Lock()

        self.connected = True
        self.motion_state = "IDLE"
        self.log_lines = ["Backend started"]

        # 상태 업데이트 타이머(테스트용)
        self.create_timer(1.0, self._tick)

    def _tick(self):
        with self._lock:
            self.log_lines.append(f"[{time.strftime('%H:%M:%S')}] heartbeat")
            self.log_lines = self.log_lines[-200:]  # 최근 200줄 유지

    def spin(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin()

    def shutdown(self):
        self.destroy_node()

    def get_snapshot(self):
        with self._lock:
            return {
                "connected": self.connected,
                "motion_state": self.motion_state,
                "log": "\n".join(self.log_lines[-50:]),  # 최근 50줄만 표시
            }
