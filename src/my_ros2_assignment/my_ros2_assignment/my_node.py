import sys
import threading
import rclpy
from PyQt5.QtWidgets import QApplication

from .ros_backend import RosBackend
from .gui_main import MainWindow

def main():
    rclpy.init()

    backend = RosBackend()

    # ROS spin thread
    spin_thread = threading.Thread(target=backend.spin, daemon=True)
    spin_thread.start()

    # Qt GUI
    app = QApplication(sys.argv)
    win = MainWindow(backend)
    win.show()

    rc = app.exec_()

    backend.shutdown()
    rclpy.shutdown()
    sys.exit(rc)

if __name__ == "__main__":
    main()
