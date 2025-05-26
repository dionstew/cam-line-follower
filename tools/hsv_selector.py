import cv2
import numpy as np

# Fungsi callback kosong untuk trackbar
def nothing(x):
    pass

# Baca gambar
image = cv2.imread('WhatsApp Image 2025-05-07 at 09.51.21_25d343e4.jpg')  # Ganti dengan nama file gambar Anda
image = cv2.resize(image, (640, 480))  # Ukuran opsional

# Konversi ke HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Buat window dan trackbar
cv2.namedWindow("HSV Selector")

# Trackbar untuk rentang H, S, dan V
cv2.createTrackbar("H Min", "HSV Selector", 0, 179, nothing)
cv2.createTrackbar("H Max", "HSV Selector", 179, 179, nothing)
cv2.createTrackbar("S Min", "HSV Selector", 0, 255, nothing)
cv2.createTrackbar("S Max", "HSV Selector", 255, 255, nothing)
cv2.createTrackbar("V Min", "HSV Selector", 0, 255, nothing)
cv2.createTrackbar("V Max", "HSV Selector", 255, 255, nothing)

while True:
    # Ambil nilai dari trackbar
    h_min = cv2.getTrackbarPos("H Min", "HSV Selector")
    h_max = cv2.getTrackbarPos("H Max", "HSV Selector")
    s_min = cv2.getTrackbarPos("S Min", "HSV Selector")
    s_max = cv2.getTrackbarPos("S Max", "HSV Selector")
    v_min = cv2.getTrackbarPos("V Min", "HSV Selector")
    v_max = cv2.getTrackbarPos("V Max", "HSV Selector")

    # Buat mask berdasarkan nilai HSV
    lower_hsv = np.array([h_min, s_min, v_min])
    upper_hsv = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Tampilkan hasil masking
    result = cv2.bitwise_and(image, image, mask=mask)

    # Tampilkan gambar asli, mask, dan hasil
    cv2.imshow("Original", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Filtered", result)

    # Tekan 'q' untuk keluar
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
