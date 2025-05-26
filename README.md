# Cam-Line-Follower 🚗📷

Robot line follower ini menggunakan **kamera** sebagai sensor utama untuk mendeteksi dan mengikuti garis jalur (line) secara real-time. Sistem ini memanfaatkan **pengolahan citra (Computer Vision)** untuk mengidentifikasi posisi garis, kemudian mengontrol motor agar robot tetap berada di jalurnya.

---

## 🔍 Fitur Utama
- Deteksi garis berbasis kamera (grayscale / HSV thresholding)
- Pengambilan keputusan berdasarkan posisi garis di frame
- Kontrol kecepatan motor untuk menjaga arah laju robot
- Dapat dijalankan pada sistem embedded seperti Raspberry Pi atau Jetson

---

## 📷 Tampilan Sistem
![Line Follower Demo](images/demo-line-follower.jpg)

> Gambar di atas menunjukkan tampilan kamera saat robot mengikuti jalur hitam di lantai putih.

---

## 🚀 Cara Menjalankan
1. Pastikan semua dependensi sudah diinstal (misalnya `OpenCV`, `PySerial`, dll.)
2. Jalankan skrip utama:
   ```bash
   python main.py
