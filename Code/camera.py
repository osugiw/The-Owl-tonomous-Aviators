import cv2
import threading

class ThreadedCamera:
    def __init__(self, src=0):
        self.cam = cv2.VideoCapture(src)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 120)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 80)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.frame = None
        self.lock = threading.Lock()
        self.running = True

        # Start background capture thread
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            ret, frame = self.cam.read()
            if ret:
                with self.lock:
                    self.frame = frame   # always overwrite with latest

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def release(self):
        self.running = False
        self.thread.join()
        self.cam.release()