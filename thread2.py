import cv2
import threading


class Camera:
    def __init__(self, camera_id=0):
        self.latest_frame = None
        self.lock = threading.Lock()
        self.stop_capture = False  # 用于停止捕获的标志
        self.capture_thread = threading.Thread(target=self.capture_video)
        self.capture_thread.start()

    def capture_video(self):
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)

        if not cap.isOpened():
            print("Error: Unable to open camera")
            return
    
        try:
            while True:
                ret, frame = cap.read()
                if ret:
                    with self.lock:
                        # 更新latest_frame为最新捕获的帧
                        self.latest_frame = frame
                        cv2.imshow("thread",frame)
                        cv2.waitKey(1)
                else:
                    print("Error: Unable to fetch frame from camera")
                    break
        finally:
            cap.release()

    def get_latest_frame(self):
        with self.lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        self.stop_capture = True  # 设置停止捕获的标志
        self.capture_thread.join()  # 等待捕获线程完成

# 注意：在实际使用中，你可能需要将类定义放在单独的文件中，例如 camera.py，并在需要时导入它。