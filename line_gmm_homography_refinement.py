# import cv2
# import numpy as np
# from sklearn.mixture import GaussianMixture

# from collections import deque

# class KalmanCentroidPredictor:
#     def __init__(self):
#         self.kalman = cv2.KalmanFilter(4, 2)
#         self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
#                                                    [0, 1, 0, 0]], np.float32)
#         self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
#                                                  [0, 1, 0, 1],
#                                                  [0, 0, 1, 0],
#                                                  [0, 0, 0, 1]], np.float32)
#         self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

#     def predict(self):
#         return self.kalman.predict()

#     def correct(self, centroid):
#         measured = np.array([[np.float32(centroid[0])],
#                              [np.float32(centroid[1])]])
#         return self.kalman.correct(measured)

# kalman_filter = KalmanCentroidPredictor()
# centroid_history = deque(maxlen=5)  # simpan 5 posisi terakhir


# # === Ambil Sampel Piksel dari Frame Pertama ===
# def sample_line_pixels(frame, roi_box):
#     x, y, w, h = roi_box
#     roi = frame[y:y+h, x:x+w]
#     samples = roi.reshape(-1, 3)  # Ambil semua piksel RGB
#     return samples

# # === Segmentasi Frame Berdasarkan GMM ===
# def segment_using_gmm(frame, gmm, threshold=0.9):
#     h, w, _ = frame.shape
#     reshaped = frame.reshape(-1, 3)
#     probs = np.exp(gmm.score_samples(reshaped))  # Probabilitas tiap piksel
#     probs = probs.reshape(h, w)
    
#     # Buat mask: 1 jika probabilitas cukup tinggi
#     mask = (probs > np.percentile(probs, threshold * 100)).astype(np.uint8) * 255
#     return mask

# # === Video Capture ===
# cap = cv2.VideoCapture("WhatsApp Video 2025-05-07 at 09.51.21_74c3f56a.mp4")
# ret, first_frame = cap.read()
# first_frame = cv2.resize(first_frame, (640, 480))

# # === Ambil Sampel Warna dari ROI ===
# roi_box = (320, 240, 30, 30)  # Ganti sesuai posisi garis
# samples = sample_line_pixels(first_frame, roi_box)

# # === Latih GMM ===
# gmm = GaussianMixture(n_components=2, covariance_type='full')
# gmm.fit(samples)


# area_min = 50  # bisa disesuaikan
# area_max = 10000  # batas maksimum kontur (misalnya noise besar seperti cahaya)

# # === Loop Video untuk Pelacakan ===
# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     frame = cv2.resize(frame, (640, 480))
#     roi = frame[240:480, 160:480]

#     # === Segmentasi GMM ===
#     mask = segment_using_gmm(roi, gmm, threshold=0.9)
    
#     # === Morphological filter (opsional) ===
#     kernel = np.ones((5,5), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

#     # === Tampilkan hasil ===
#     result = cv2.bitwise_and(roi, roi, mask=mask)

#     filtered_contours = []
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     centroid = None

#     for cnt in contours:
#         area = cv2.contourArea(cnt)
#         if area_min < area < area_max:
#             filtered_contours.append(cnt)

#     cv2.drawContours(result, filtered_contours, -1, (0, 0, 255), 2)

#     if filtered_contours:
#         # cv2.drawContours(result, filtered_contours, -1, (0, 0, 255), 2)  # merah, tebal 2 piksel
#         largest = max(filtered_contours, key=cv2.contourArea)
#         M = cv2.moments(largest)
#         if M["m00"] != 0:
#             cx = int(M["m10"] / M["m00"])
#             cy = int(M["m01"] / M["m00"])
#             centroid = (cx, cy)

#             # Simpan ke buffer history
#             centroid_history.append(centroid)

#             # Gambar titik hijau: aktual
#             cv2.circle(roi, centroid, 5, (0, 255, 0), -1)

#     # Jika ada cukup data di buffer (minimal 3–5), gunakan rata-rata untuk koreksi
#     if len(centroid_history) >= 3:
#         avg_cx = int(np.mean([pt[0] for pt in centroid_history]))
#         avg_cy = int(np.mean([pt[1] for pt in centroid_history]))
#         kalman_filter.correct((avg_cx, avg_cy))

#     predicted = kalman_filter.predict()
#     pred_pt = (int(predicted[0]), int(predicted[1]))

#     cv2.circle(roi, pred_pt, 5, (255, 0, 0), -1)  # Biru = prediksi

#     cv2.imshow("GMM Result", result)
#     cv2.imshow("GMM Mask", mask)

#     cv2.circle(roi, pred_pt, 5, (255, 0, 0), -1)  # Biru = prediksi

#     cv2.imshow("ROI", roi)
#     cv2.imshow("Mask", mask)
#     cv2.imshow("Filtered", result)

#     frame[240:480, 160:480] = roi

#     cv2.imshow("Input Camera", frame)

#     if cv2.waitKey(10) == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()

import cv2
import numpy as np
from sklearn.mixture import GaussianMixture

def sample_line_pixels(frame, roi_box):
    x, y, w, h = roi_box
    roi = frame[y:y+h, x:x+w]
    samples = roi.reshape(-1, 3)
    return samples

def segment_using_gmm(frame, gmm, threshold=0.9):
    h, w, _ = frame.shape
    reshaped = frame.reshape(-1, 3)
    probs = np.exp(gmm.score_samples(reshaped))
    probs = probs.reshape(h, w)
    mask = (probs > np.percentile(probs, threshold * 100)).astype(np.uint8) * 255
    return mask

def estimate_homography(prev_gray, curr_gray):
    # Gunakan feature matching + optical flow
    orb = cv2.ORB_create(500)
    kp1, des1 = orb.detectAndCompute(prev_gray, None)
    kp2, des2 = orb.detectAndCompute(curr_gray, None)
    if des1 is None or des2 is None:
        return None

    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(des1, des2, k=2)
    good = [m[0] for m in matches if len(m) == 2 and m[0].distance < 0.75 * m[1].distance]

    if len(good) > 10:
        pts1 = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, _ = cv2.findHomography(pts1, pts2, cv2.RANSAC)
        return H
    return None

cap = cv2.VideoCapture("WhatsApp Video 2025-05-07 at 09.51.21_74c3f56a.mp4")

ret, first_frame = cap.read()
if not ret:
    raise Exception("Video tidak terbaca.")
first_frame = cv2.resize(first_frame, (640, 480))

# ==== Inisialisasi: Pelatihan GMM ====
roi_box = (320, 300, 30, 30)  # Ubah untuk ambil warna garis
samples = sample_line_pixels(first_frame, roi_box)
gmm = GaussianMixture(n_components=2, covariance_type='full')
gmm.fit(samples)

# ==== Segmentasi Awal ====
prev_frame = first_frame.copy()
prev_mask = segment_using_gmm(prev_frame, gmm)
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # === Estimasi Homography dari frame sebelumnya ===
    H = estimate_homography(prev_gray, gray)

    if H is not None:
        # Warp mask sebelumnya ke frame sekarang
        warped_mask = cv2.warpPerspective(prev_mask, H, (frame.shape[1], frame.shape[0]))
        mask = warped_mask

        # Jika terlalu berbeda dari hasil segmentasi ulang → pakai segmentasi baru
        area = cv2.countNonZero(mask)
        if area < 1000 or area > 50000:
            mask = segment_using_gmm(frame, gmm)
    else:
        # Jika homography gagal → lakukan segmentasi penuh
        mask = segment_using_gmm(frame, gmm)

    # Filter noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Visualisasi
    result = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    # Update frame sebelumnya
    prev_frame = frame.copy()
    prev_gray = gray.copy()
    prev_mask = mask.copy()

    if cv2.waitKey(10) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
