from PyQt5.QtCore import QThread, pyqtSignal
import time

class DummyMoveWorker(QThread):
    finished_signal = pyqtSignal(str)

    def __init__(self, backend):
        super().__init__()
        self.backend = backend

    def run(self):
        # 버튼 동작이 별도 스레드에서 돈다는 걸 증명하기 위한 더미 동작
        for i in range(5):
            time.sleep(0.5)
        self.finished_signal.emit("done")
