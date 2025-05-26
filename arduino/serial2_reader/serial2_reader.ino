// =========================================================================================
// GUNAKAN LOGIKA TERBALIK UNTUK PENGATURAN PIN ARAH DAN REM (LOW (ARDUINO) == HIGH (RELAY))
// =========================================================================================

#include <Wire.h>

// Contoh pin untuk PWM/motor
short PWML = 6;         // pinout PWML
short PWMR = 7;         // pinout PWMR
short arahR = 37;       // pinout atur arah Kiri 
short arahL = 39;       // pinout atur arah Kanan
short brakeKanan = 30;   // rem Kiri
short brakeKiri = 28;  // rem Kanan

String receivedData = ""; // Variabel global untuk mengumpulkan data yang masuk
bool newDataAvailable = false; // Flag untuk menandakan jika ada data baru yang siap diproses

String inputData = "";
bool dataComplete = false;

int arahMotorKiri = 0;
int pwmKiri = 0;
int arahMotorKanan = 0;
int pwmKanan = 0;
int scalePWMKanan = 0;
int scalePWMKiri = 0;

void setup() {
  int baudrate0 = 9600;
  Serial.begin(baudrate0);
  Serial.println("Format diharapkan: arahMotorKiri, PWMKiri; arahMotorKanan, PWMKanan");
  delay(2000);
  Serial2.begin(115200);  // Inisialisasi Serial1 dengan baudrate 115200

  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(brakeKanan, OUTPUT);
  pinMode(brakeKiri, OUTPUT);
  pinMode(arahL, OUTPUT);
  pinMode(arahR, OUTPUT);

  digitalWrite(brakeKanan, LOW);
  digitalWrite(brakeKiri, LOW);

  digitalWrite(arahL, LOW);
  digitalWrite(arahR, LOW);

  // Inisialisasi I2C sebagai Master
  Wire.begin();
  // (Opsional) mengatur kecepatan I2C, mis. 400 kHz
  // Wire.setClock(400000L);

  Serial.println("Master I2C Siap");
  delay(1000);

  // Menjalankan motor (contoh)
  // analogWrite(PWML, pwmLdata);
  // analogWrite(PWMR, pwmRdata);
  Serial.println("Motor is Running...");
  delay(5000);
}

void loop() {
  
  float receivedRpm1 = 0.0;
  float receivedRpm2 = 0.0;

  // ------------------------------------------------------
  // 1. Minta data dari Slave 1 (alamat 0x10) atau Motor Kiri
  // ------------------------------------------------------
  Wire.requestFrom((uint8_t)0x10, (uint8_t)4);
  if (Wire.available() >= 4) {
    byte dataArr2[4];
    for (int i = 0; i < 4; i++) {
      dataArr2[i] = Wire.read();
    }
    memcpy(&receivedRpm2, dataArr2, sizeof(receivedRpm2));
  }

  // ------------------------------------------------------
  // 2. Minta data dari Slave 2 (alamat 0x11) atau Motor Kanan
  // ------------------------------------------------------
  Wire.requestFrom((uint8_t)0x11, (uint8_t)4);
  if (Wire.available() >= 4) {
    byte dataArr1[4];
    for (int i = 0; i < 4; i++) {
      dataArr1[i] = Wire.read();
    }
    memcpy(&receivedRpm1, dataArr1, sizeof(receivedRpm1));
  }

  // ------------------------------------------------------
  // 3. Baca perintah dan parse data dari Serial2 (data dikirim dari joystick)
  // ------------------------------------------------------
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (c == '\n') {      // Data paket selesai
      dataComplete = true;
      break;
    } else {
      inputData += c;
    }
  }

  if (dataComplete) {
    Serial.print("Kecepatan Motor Kiri: ");
    Serial.print(receivedRpm1);
    Serial.print(", Kecepatan Motor Kanan: ");
    Serial.print(receivedRpm2);
    
    parseData(inputData);
    inputData = "";
    dataComplete = false;
  }
  //-------------------------------------------
  // 4. Control Motor Rotation and PWMs
  //-------------------------------------------

  
  scalePWMKanan = map(pwmKanan, 0, 255, 30, 123);
  scalePWMKiri = map(pwmKiri, 0, 255, 30, 115);

  // Kontrol Motor Kiri
  if (arahMotorKiri == 1){
    digitalWrite(arahL, HIGH);  // arah Putar Maju  
    digitalWrite(brakeKiri, HIGH);  // matikan Rem
    analogWrite(PWML, scalePWMKiri); // atur PWM
  }
  else if (arahMotorKiri == -1){
    digitalWrite(arahL, LOW);   // arah Putar Mundur
    digitalWrite(brakeKiri, HIGH);  // matikan Rem
    analogWrite(PWML, scalePWMKiri); // atur PWM
  }
  else {
    // arah putar Diam
    digitalWrite(arahL, HIGH);      // arah default maju
    digitalWrite(brakeKiri, LOW);  // aktifkan Rem
  };

  // Kontrol Motor Kanan
  if (arahMotorKanan >= 1){
    digitalWrite(arahR, HIGH);  // arah Putar Maju  
    digitalWrite(brakeKanan, HIGH);  // aktifkan Rem
    analogWrite(PWMR, scalePWMKanan); // atur PWM
  }
  else if (arahMotorKanan == -1){
    digitalWrite(arahR, LOW);   // arah Putar Mundur
    digitalWrite(brakeKanan, HIGH);  // aktifkan Rem
    analogWrite(PWMR, scalePWMKanan); // atur PWM
  }
  else {
    // arah putar Diam
    digitalWrite(arahR, HIGH);      // arah default maju
    digitalWrite(brakeKanan, LOW);  // aktifkan Rem
  };

  
  // Tampilkan hasil parsing ke Serial Monitor
  Serial.print(", Arah Motor Kiri: ");
  Serial.print(arahMotorKiri);
  Serial.print(", PWM Kiri: ");
  Serial.print(scalePWMKiri);
  Serial.print(", Arah Motor Kanan: ");
  Serial.print(arahMotorKanan);
  Serial.print(", PWM Kanan: ");
  Serial.println(scalePWMKanan);
}

void parseData(String data) {
  int parts[4];   // Menyimpan indeks posisi ';'
  int lastIndex = -1;
  int count = 0;

  // Cari posisi delimiter ';' sebanyak 4 kali
  for (int i = 0; i < data.length() && count < 4; i++) {
    if (data.charAt(i) == ';') {
      parts[count] = i;
      count++;
    }
  }

  if (count < 4) {
    Serial.println("Error: Data kurang lengkap");
    return;
  }

  // Ekstrak substring untuk tiap nilai dan konversi ke integer
  arahMotorKiri = data.substring(lastIndex + 1, parts[0]).toInt();
  lastIndex = parts[0];

  pwmKiri = data.substring(lastIndex + 1, parts[1]).toInt();
  lastIndex = parts[1];

  arahMotorKanan = data.substring(lastIndex + 1, parts[2]).toInt();
  lastIndex = parts[2];

  pwmKanan = data.substring(lastIndex + 1, parts[3]).toInt();
}
