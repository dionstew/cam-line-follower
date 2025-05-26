import cv2
import numpy as np
from pid import PIDController
from collections import deque

class KalmanCentroidPredictor:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                   [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                 [0, 1, 0, 1],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

    def predict(self):
        return self.kalman.predict()

    def correct(self, centroid):
        measured = np.array([[np.float32(centroid[0])],
                             [np.float32(centroid[1])]])
        return self.kalman.correct(measured)

kalman_filter = KalmanCentroidPredictor()
centroid_history = deque(maxlen=5)  # simpan 5 posisi terakhir

kernel = np.ones((3, 3), np.uint8)

h_min, s_min, v_min = 21, 7, 10
h_max, s_max, v_max = 38, 200, 255

# Open the default camera
cam = cv2.VideoCapture(r"WhatsApp Video 2025-05-07 at 09.51.21_74c3f56a.mp4")

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

area_min = 50  # bisa disesuaikan
area_max = 10000  # batas maksimum kontur (misalnya noise besar seperti cahaya)

while True:
    ret, frame = cam.read()

    # Write the frame to the output file
    # out.write(frame)
    frame = cv2.resize(frame, (640, 480))
    
    roi = frame[300:480, 160:480]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Buat mask berdasarkan nilai HSV
    lower_hsv = np.array([h_min, s_min, v_min])
    upper_hsv = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # hilangkan noise kecil
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # isi lubang kecil

    result = cv2.bitwise_and(roi, roi, mask=mask)

    filtered_contours = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroid = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area_min < area < area_max:
            filtered_contours.append(cnt)

    cv2.drawContours(result, filtered_contours, -1, (0, 0, 255), 2)

    if filtered_contours:
        # cv2.drawContours(result, filtered_contours, -1, (0, 0, 255), 2)  # merah, tebal 2 piksel
        largest = max(filtered_contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroid = (cx, cy)

            # Simpan ke buffer history
            centroid_history.append(centroid)

            # Gambar titik hijau: aktual
            cv2.circle(roi, centroid, 5, (0, 255, 0), -1)

    # Jika ada cukup data di buffer (minimal 3–5), gunakan rata-rata untuk koreksi
    if len(centroid_history) >= 3:
        avg_cx = int(np.mean([pt[0] for pt in centroid_history]))
        avg_cy = int(np.mean([pt[1] for pt in centroid_history]))
        kalman_filter.correct((avg_cx, avg_cy))

    predicted = kalman_filter.predict()
    pred_pt = (int(predicted[0]), int(predicted[1]))

    cv2.circle(roi, pred_pt, 5, (255, 0, 0), -1)  # Biru = prediksi

    cv2.imshow("ROI", roi)
    cv2.imshow("Mask", mask)
    cv2.imshow("Filtered", result)

    frame[300:480, 160:480] = roi
    cv2.imshow("Input Camera", frame)
        
    # Press 'q' to exit the loop
    if cv2.waitKey(10) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
# out.release()
cv2.destroyAllWindows()