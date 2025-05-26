#include <Wire.h>

// Pin PWM dan brake
const short pwmL = 6;
const short pwmR = 7;
const short brakeKanan = 28;
const short brakeKiri = 30;

// Nilai PWM awal dan arah
int pwmLdata = 0;
int pwmRdata = 0;
int arahL = 1; // 1 = maju, 0 = mundur
int arahR = 1; // 1 = maju, 0 = mundur

const int step = 5;  // Besar kenaikan/penurunan PWM

String inputString = "";    // buffer untuk data serial masuk
bool stringComplete = false;

void setup() {
  pinMode(pwmL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(brakeKanan, OUTPUT);
  pinMode(brakeKiri, OUTPUT);

  digitalWrite(brakeKanan, LOW);
  digitalWrite(brakeKiri, LOW);

  Serial.begin(115200);
  Wire.begin();  // Inisialisasi I2C Master

  Serial.println("Master I2C Siap");
  delay(1000);

  analogWrite(pwmL, pwmLdata);
  analogWrite(pwmR, pwmRdata);
  Serial.println("Motor is Running...");
  delay(2000);
}

void loop() {
  // Baca serial input
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == ';') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    parseCommand(inputString);
    kendalikanMotor();
    inputString = "";
    stringComplete = false;
  }

  // ===============================
  // 2. Baca Data RPM via I2C
  // ===============================
  float rpmKanan = 0.0;
  float rpmKiri = 0.0;

  // Slave 0x10 = Kanan
  Wire.requestFrom((uint8_t)0x10, (uint8_t)4);
  if (Wire.available() >= 4) {
    byte buff[4];
    for (int i = 0; i < 4; i++) buff[i] = Wire.read();
    memcpy(&rpmKanan, buff, sizeof(rpmKanan));
  }

  // Slave 0x11 = Kiri
  Wire.requestFrom((uint8_t)0x11, (uint8_t)4);
  if (Wire.available() >= 4) {
    byte buff[4];
    for (int i = 0; i < 4; i++) buff[i] = Wire.read();
    memcpy(&rpmKiri, buff, sizeof(rpmKiri));
  }

  // ===============================
  // 3. Tampilkan Semua Data
  // ===============================
  Serial.print("PWM L: ");
  Serial.print(pwmLdata);
  Serial.print(" (");
  Serial.print(arahL == 1 ? "maju" : "mundur");
  Serial.print(") | PWM R: ");
  Serial.print(pwmRdata);
  Serial.print(" (");
  Serial.print(arahR == 1 ? "maju" : "mundur");
  Serial.print(") || RPM L: ");
  Serial.print(rpmKiri, 2);
  Serial.print(" | RPM R: ");
  Serial.println(rpmKanan, 2);

  delay(500);  // Refresh rate tampilan
}

void parseCommand(String cmd) {
  // Format: pwm_kiri,arah_kiri;pwm_kanan,arah_kanan
  int sepIndex = cmd.indexOf(';');
  if (sepIndex == -1) {
    // Format tidak lengkap, abaikan
    return;
  }

  String kiriStr = cmd.substring(0, sepIndex);
  String kananStr = cmd.substring(sepIndex + 1);

  int commaKiri = kiriStr.indexOf(',');
  if (commaKiri > 0) {
    pwmLdata = kiriStr.substring(0, commaKiri).toInt();
    arahL = kiriStr.substring(commaKiri + 1).toInt();
  }

  int commaKanan = kananStr.indexOf(',');
  if (commaKanan > 0) {
    pwmRdata = kananStr.substring(0, commaKanan).toInt();
    arahR = kananStr.substring(commaKanan + 1).toInt();
  }
}

void kendalikanMotor() {
  // Motor Kiri
  if (arahL == 1) {
    digitalWrite(brakeKiri, LOW);
    analogWrite(pwmL, pwmLdata);
  } else {
    digitalWrite(brakeKiri, HIGH);
    analogWrite(pwmL, pwmLdata);
  }

  // Motor Kanan
  if (arahR == 1) {
    digitalWrite(brakeKanan, LOW);
    analogWrite(pwmR, pwmRdata);
  } else {
    digitalWrite(brakeKanan, HIGH);
    analogWrite(pwmR, pwmRdata);
  }
}
