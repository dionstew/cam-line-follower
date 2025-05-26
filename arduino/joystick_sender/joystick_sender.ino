// Pin motor kanan
const int motorKananIN1 = 2;
const int motorKananIN2 = 3;
const int motorKananPWM = 6;

// Pin motor kiri
const int motorKiriIN1 = 4;
const int motorKiriIN2 = 5;
const int motorKiriPWM = 7;

int motorSpeedKanan = 0;
int motorSpeedKiri = 0;

String arahKanan = "Stop";
String arahKiri = "Stop";

// Batas dead zone joystick
const int deadzoneMin = 470;
const int deadzoneMax = 550;

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);  // Inisialisasi Serial1 dengan baudrate 115200

  pinMode(motorKananIN1, OUTPUT);
  pinMode(motorKananIN2, OUTPUT);
  pinMode(motorKananPWM, OUTPUT);

  pinMode(motorKiriIN1, OUTPUT);
  pinMode(motorKiriIN2, OUTPUT);
  pinMode(motorKiriPWM, OUTPUT);

  stopMotor(motorKananIN1, motorKananIN2, motorKananPWM);
  stopMotor(motorKiriIN1, motorKiriIN2, motorKiriPWM);
}

void loop() {
  int yAxisR = analogRead(A1);  // Membaca sumbu Y untuk motor kanan (A1)
  int yAxisL = analogRead(A2);  // Membaca sumbu Y untuk motor kiri (A3)

  arahKanan = "Stop";
  arahKiri = "Stop";

  // Cek apakah joystick di posisi center (dead zone) untuk kanan dan kiri
  bool yCenterR = (yAxisR >= deadzoneMin && yAxisR <= deadzoneMax);
  bool yCenterL = (yAxisL >= deadzoneMin && yAxisL <= deadzoneMax);

  if (yCenterR && yCenterL) {
    stopMotor(motorKananIN1, motorKananIN2, motorKananPWM);
    stopMotor(motorKiriIN1, motorKiriIN2, motorKiriPWM);
    motorSpeedKanan = 0;
    motorSpeedKiri = 0;
    arahKanan = "Stop";
    arahKiri = "Stop";
  } else {
    // Kontrol motor kanan
    if (yAxisR < deadzoneMin) {
      digitalWrite(motorKananIN1, HIGH);
      digitalWrite(motorKananIN2, LOW);
      arahKanan = "Mundur";

      motorSpeedKanan = map(yAxisR, deadzoneMin, 0, 0, 255);
    } else if (yAxisR > deadzoneMax) {
      digitalWrite(motorKananIN1, LOW);
      digitalWrite(motorKananIN2, HIGH);
      arahKanan = "Maju";

      motorSpeedKanan = map(yAxisR, deadzoneMax, 1023, 0, 255);
    } else {
      motorSpeedKanan = 0;
    }

    // Kontrol motor kiri
    if (yAxisL < deadzoneMin) {
      digitalWrite(motorKiriIN1, HIGH);
      digitalWrite(motorKiriIN2, LOW);
      arahKiri = "Mundur";

      motorSpeedKiri = map(yAxisL, deadzoneMin, 0, 0, 255);
    } else if (yAxisL > deadzoneMax) {
      digitalWrite(motorKiriIN1, LOW);
      digitalWrite(motorKiriIN2, HIGH);
      arahKiri = "Maju";

      motorSpeedKiri = map(yAxisL, deadzoneMax, 1023, 0, 255);
    } else {
      motorSpeedKiri = 0;
    }
  }

  // Hindari buzzing pada kecepatan rendah
  if (motorSpeedKanan < 70) motorSpeedKanan = 0;
  if (motorSpeedKiri < 70) motorSpeedKiri = 0;

  analogWrite(motorKananPWM, motorSpeedKanan);
  analogWrite(motorKiriPWM, motorSpeedKiri);

  // Konversi arah ke kode -1, 0, 1
  int kodeArahKiri = 0;
  if (arahKiri == "Mundur") kodeArahKiri = -1;
  else if (arahKiri == "Maju") kodeArahKiri = 1;

  int kodeArahKanan = 0;
  if (arahKanan == "Mundur") kodeArahKanan = -1;
  else if (arahKanan == "Maju") kodeArahKanan = 1;

  // Debugging: Tampilkan status motor
  Serial.print(kodeArahKanan);
  Serial.print(";");
  Serial.print(motorSpeedKanan);
  Serial.print(";");
  Serial.print(kodeArahKiri);
  Serial.print(";");
  Serial.print(motorSpeedKiri);
  Serial.println(";");

  // Kirim data ke Serial1
  Serial1.print(kodeArahKiri);
  Serial1.print(";");
  Serial1.print(motorSpeedKiri);
  Serial1.print(";");
  Serial1.print(kodeArahKanan);
  Serial1.print(";");
  Serial1.print(motorSpeedKanan);
  Serial1.println(";");

  delay(100);
}

void stopMotor(int pinIN1, int pinIN2, int pinPWM) {
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  analogWrite(pinPWM, 0);
}
