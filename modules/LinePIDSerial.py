import cv2
from modules.line_tracker import LineTracker
from modules.pid import PIDController
import time
import serial
import threading

def read_from_arduino(ser):
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(f"Arduino says: {line}")

def main():
    # Serial setup — sesuaikan port Anda
    ser = serial.Serial('COM11', 115200, timeout=1)
    time.sleep(2)  # tunggu serial siap

    # Thread baca data dari Arduino
    threading.Thread(target=read_from_arduino, args=(ser,), daemon=True).start()

    # Setup LineTracker dan PID
    roi_bounds = (300, 480, 160, 480)
    hsv_bounds = (21, 7, 10, 38, 200, 255)
    area_bounds = (50, 10000)
    setpoint = int((roi_bounds[3] - roi_bounds[2]) // 2)

    tracker = LineTracker(
        video_path=r'WhatsApp Video 2025-05-07 at 09.51.21_74c3f56a.mp4',
        roi_bounds=roi_bounds,
        hsv_bounds=hsv_bounds,
        area_bounds=area_bounds,
        history_len=60
    )
    pid = PIDController(Kp=0.5, Ki=0.1, Kd=0.05, setpoint=setpoint)

    prev_time = time.time()

    while True:
        ret, frame = tracker.cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (640, 480))
        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time

        frame, mask, result, roi, pred_pt = tracker.process_frame(frame, dt)

        # Hitung PID
        prediction_x = pred_pt[0]
        error = setpoint - prediction_x
        pid_output = pid.compute(prediction_x, dt)

        # Kirim ke Arduino format: PID:<nilai>\n
        serial_data = f"PID:{pid_output:.2f}\n"
        ser.write(serial_data.encode())

        # Debug teks di frame
        debug_text = f"Pred: {prediction_x} | Set: {setpoint} | Err: {error} | PID: {pid_output:.2f}"
        print(debug_text)
        cv2.putText(frame, debug_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        frame[roi_bounds[0]:roi_bounds[1], roi_bounds[2]:roi_bounds[3]] = roi

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("ROI", roi)
        cv2.imshow("Filtered", result)

        if cv2.waitKey(10) == ord('q'):
            break

    tracker.cap.release()
    cv2.destroyAllWindows()
    stop = 0
    serial_data = f"CMD:{stop: .2f}\n"
    ser.write("CMD:{0.2f}")
    ser.close()

if __name__ == "__main__":
    main()
