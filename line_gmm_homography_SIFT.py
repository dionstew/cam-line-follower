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

# === Inisialisasi Video ===
cap = cv2.VideoCapture("WhatsApp Video 2025-05-07 at 09.51.21_74c3f56a.mp4")

ret, first_frame = cap.read()
if not ret:
    raise Exception("Video tidak terbaca.")
first_frame = cv2.resize(first_frame, (640, 480))

# === Tentukan ROI (bagian bawah gambar) ===
roi_y1, roi_y2 = 240, 480
roi_x1, roi_x2 = 160, 480

first_roi = first_frame[roi_y1:roi_y2, roi_x1:roi_x2]
roi_gray_prev = cv2.cvtColor(first_roi, cv2.COLOR_BGR2GRAY)

# === Ambil Sampel Warna dari ROI untuk Latih GMM ===
roi_box = (30, 180, 20, 20)  # pada ROI, ganti sesuai posisi garis
samples = sample_line_pixels(first_roi, roi_box)
gmm = GaussianMixture(n_components=2, covariance_type='full')
gmm.fit(samples)

# === Segmentasi awal ===
prev_mask = segment_using_gmm(first_roi, gmm)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # === Estimasi Homography dari ROI ===
    H = estimate_homography(roi_gray_prev, roi_gray)

    if H is not None:
        # Warp hasil segmentasi sebelumnya ke ROI sekarang
        warped_mask = cv2.warpPerspective(prev_mask, H, (roi.shape[1], roi.shape[0]))
        mask = warped_mask

        # Jika area hasil terlalu kecil/besar → lakukan segmentasi ulang
        area = cv2.countNonZero(mask)
        if area < 1000 or area > 40000:
            mask = segment_using_gmm(roi, gmm)
    else:
        # Jika gagal estimasi homography → segmentasi ulang
        mask = segment_using_gmm(roi, gmm)

    # === Bersihkan noise kecil ===
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # === Visualisasi ===
    result = cv2.bitwise_and(roi, roi, mask=mask)
    cv2.imshow("ROI Mask", mask)
    cv2.imshow("ROI Result", result)

    # === Update data frame sebelumnya ===
    roi_gray_prev = roi_gray.copy()
    prev_mask = mask.copy()

    if cv2.waitKey(10) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
