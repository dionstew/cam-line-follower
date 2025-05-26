import cv2
import numpy as np
from collections import deque
from modules.pid import PIDController  # opsional
from modules.kalman import KalmanCentroidPredictor  # pastikan Anda pisahkan kelas Kalman juga

# --- Fungsi bantu ---
def filter_contours(contours, area_min, area_max):
    return [cnt for cnt in contours if area_min < cv2.contourArea(cnt) < area_max]

def calculate_centroid(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)

# --- Kelas utama pelacak garis ---
class LineTracker:
    def __init__(self, 
                 video_path, 
                 roi_bounds, 
                 hsv_bounds, 
                 area_bounds, 
                 history_len=5,
                 show_output=True):
        self.cap = cv2.VideoCapture(video_path)
        self.roi_y1, self.roi_y2, self.roi_x1, self.roi_x2 = roi_bounds
        self.h_min, self.s_min, self.v_min, self.h_max, self.s_max, self.v_max = hsv_bounds
        self.area_min, self.area_max = area_bounds
        self.kalman = KalmanCentroidPredictor()
        self.centroid_history = deque(maxlen=history_len)
        self.kernel = np.ones((3, 3), np.uint8)
        self.show_output = show_output

    def process_frame(self, frame, dt=0.03):
        roi = frame[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([self.h_min, self.s_min, self.v_min])
        upper_hsv = np.array([self.h_max, self.s_max, self.v_max])
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        result = cv2.bitwise_and(roi, roi, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = filter_contours(contours, self.area_min, self.area_max)
        cv2.drawContours(result, filtered, -1, (0, 0, 255), 2)

        centroid = None
        if filtered:
            largest = max(filtered, key=cv2.contourArea)
            centroid = calculate_centroid(largest)
            if centroid:
                self.centroid_history.append(centroid)
                cv2.circle(roi, centroid, 5, (0, 255, 0), -1)  # Hijau

        if len(self.centroid_history) >= 3:
            avg_cx = int(np.mean([pt[0] for pt in self.centroid_history]))
            avg_cy = int(np.mean([pt[1] for pt in self.centroid_history]))
            self.kalman.correct((avg_cx, avg_cy))

        predicted = self.kalman.predict()
        pred_pt = (int(predicted[0]), int(predicted[1]))
        cv2.circle(roi, pred_pt, 5, (255, 0, 0), -1)  # Biru

        return frame, mask, result, roi, pred_pt

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            frame = cv2.resize(frame, (640, 480))
            frame, mask, result, roi, _ = self.process_frame(frame)
            frame[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2] = roi

            if self.show_output:
                cv2.imshow("ROI", roi)
                cv2.imshow("Mask", mask)
                cv2.imshow("Filtered", result)
                cv2.imshow("Input Camera", frame)

            if cv2.waitKey(10) == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
