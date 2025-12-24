#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

// -------- Pins --------
const int calibButton = 2;   // زر المعايرة
const int vibPin      = 3;   // موتور الاهتزاز (PWM)
const int ledPin      = 8;   // LED

// -------- Parameters --------
float neutralPitch = 0.0;
const float POSTURE_THRESHOLD = 13.0; // درجة الميل المسموح بيها

// Time settings
unsigned long reminderInterval = 5UL * 60UL * 1000UL; // تنبيه الحركة (جرب 1 دقيقة للاختبار)
const unsigned long BAD_DELAY  = 3000; // 3 ثواني تأخير قبل إنذار القعدة الغلط

// Vibration strength
const int gentleDuty = 90;
const int strongDuty = 255;
const unsigned long gentleDuration = 300;
const unsigned long strongDuration = 1000;

// -------- State variables --------
unsigned long lastGoodStart = 0;
unsigned long badStartTime  = 0;
bool inBad = false;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  pinMode(calibButton, INPUT_PULLUP);
  pinMode(vibPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  analogWrite(vibPin, 0);
  digitalWrite(ledPin, LOW);

  delay(500);
  calibrateNeutral();
  lastGoodStart = millis();

  Serial.println("System Ready");
}

// ===================== LOOP =====================
void loop() {

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  float ax_f = ax / 16384.0;
  float az_f = az / 16384.0;

  float pitch = atan2(ax_f, az_f) * 57.295779513;
  float rel   = pitch - neutralPitch;

  unsigned long now = millis();

  // -------- Calibration Button --------
  if (digitalRead(calibButton) == LOW) {
    calibrateNeutral();
    lastGoodStart = now;
    delay(300);
  }

  // -------- Bad Posture Detection (with delay) --------
  if (abs(rel) > POSTURE_THRESHOLD) {

    if (badStartTime == 0) {
      badStartTime = now;   // نبدأ العد
    }

    // لو الميل مستمر أكتر من وقت التأخير
    if (now - badStartTime >= BAD_DELAY) {

      Serial.println("Bad posture confirmed!");

      // تشغيل LED والموتور معًا
      digitalWrite(ledPin, HIGH);
      analogWrite(vibPin, strongDuty);
      delay(strongDuration);
      analogWrite(vibPin, 0);
      digitalWrite(ledPin, LOW);

      lastGoodStart = now;
      inBad = true;
      badStartTime = 0;
    }

  } else {
    // رجع قعدته مظبوطة
    badStartTime = 0;

    if (inBad) {
      inBad = false;
      lastGoodStart = now;
      Serial.println("Recovered");
    }
  }

  // -------- Sitting Too Long Reminder --------
  if (now - lastGoodStart >= reminderInterval) {

    Serial.println("Gentle reminder: move your body");

    // تشغيل LED والموتور معًا
    digitalWrite(ledPin, HIGH);
    analogWrite(vibPin, gentleDuty);
    delay(gentleDuration);
    analogWrite(vibPin, 0);
    digitalWrite(ledPin, LOW);

    lastGoodStart = now;
  }

  // -------- Debug Print --------
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Relative: ");
  Serial.println(rel);

  delay(200);
}

// ===================== CALIBRATION FUNCTION =====================
void calibrateNeutral() {

  Serial.println("Calibrating... Hold straight");

  long sum = 0;
  const int samples = 30;

  for (int i = 0; i < samples; i++) {

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    float ax_f = ax / 16384.0;
    float az_f = az / 16384.0;

    float p = atan2(ax_f, az_f) * 57.295779513;
    sum += p;

    delay(60);
  }

  neutralPitch = sum / (float)samples;

  // LED يومض عند انتهاء المعايرة
  digitalWrite(ledPin, HIGH);
  delay(300);
  digitalWrite(ledPin, LOW);

  Serial.print("Neutral Pitch set to: ");
  Serial.println(neutralPitch);
}