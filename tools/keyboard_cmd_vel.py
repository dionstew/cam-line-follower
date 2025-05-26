import serial
import threading
import sys

try:
    import msvcrt  # Windows
except ImportError:
    import tty
    import termios

def getch():
    try:
        return msvcrt.getch().decode('utf-8')
    except ImportError:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def read_from_serial(ser):
    while True:
        data = ser.readline().decode('utf-8').strip()
        if data:
            print(f"Arduino: {data}")

def main():
    port = 'COM11'  # Sesuaikan port
    baudrate = 115200
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except serial.SerialException:
        print(f"Gagal membuka port {port}")
        return

    threading.Thread(target=read_from_serial, args=(ser,), daemon=True).start()

    print("Ketik karakter, 'q' untuk keluar")
    while True:
        ch = getch()
        if ch == 'q':
            break
        ser.write(ch.encode())
        print(f"Anda: {ch}")

    ser.close()

if __name__ == "__main__":
    main()
