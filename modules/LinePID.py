import cv2
from modules.line_tracker import LineTracker
from modules.pid import PIDController
import time

roi_bounds = (300, 480, 160, 480)  # y1, y2, x1, x2
hsv_bounds = (21, 7, 10, 38, 200, 255)  # h_min, s_min, v_min, h_max, s_max, v_max
area_bounds = (50, 10000)
setpoint = int(((roi_bounds[2]-roi_bounds[2]) + (roi_bounds[3]-roi_bounds[2]))/2)

show_output = True

tracker = LineTracker(
    video_path=r"WhatsApp Video 2025-05-07 at 09.51.21_74c3f56a.mp4",
    roi_bounds=roi_bounds,
    hsv_bounds=hsv_bounds,
    area_bounds=area_bounds,
    history_len=70
)

pid = PIDController(Kp = 0.5, Ki = 0.1, Kd = 0.05, setpoint=setpoint)

while True:
    ret, frame = tracker.cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    dt = 0.03
    frame, mask, result, roi, pred_pt = tracker.process_frame(frame, dt)

    # Hitung PID
    prediction_x = pred_pt[0]
    error = setpoint - prediction_x
    pid_output = pid.compute(pred_pt[0], dt)

    # Di sinilah Anda bisa publish ke ROS 2
    debug_text = f"Pred (px): {pred_pt[0]} | Set(px): {setpoint} | Err(px): {error} | PID(px): {pid_output:.2f}"
    print(debug_text)
    cv2.putText(frame, debug_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Tampilkan hasil
    frame[300:480, 160:480] = roi
    roi_s = frame[315:325, 160:480]
    
    if show_output:
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("ROI", roi)
        cv2.imshow("Filtered", result)

    if cv2.waitKey(10) == ord('q'):
        break

# Bersihkan resource
tracker.cap.release()
cv2.destroyAllWindows()