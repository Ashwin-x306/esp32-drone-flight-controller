#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

/* ================= CONFIG ================= */
#define MPU_ADDR 0x68

// RX Pins
#define CH1 34   // Roll
#define CH2 35   // Pitch
#define CH3 32   // Throttle
#define CH4 33   // Yaw

// Motor Pins
#define M13_PIN 13
#define M12_PIN 12
#define M14_PIN 14
#define M27_PIN 27

/* ================= GLOBALS ================= */

// ---------- Gyro ----------
int16_t GyroXraw, GyroYraw, GyroZraw;
float gyroRoll, gyroPitch, gyroYaw;
float gyroRollOffset = 0, gyroPitchOffset = 0, gyroYawOffset = 0;

// ---------- Accel ----------
int16_t AccXraw, AccYraw, AccZraw;
float accRoll, accPitch;
float accXOffset = 0, accYOffset = 0, accZOffset = 0;

// ---------- Angles ----------
float rollAngle = 0;
float pitchAngle = 0;
const float alpha = 0.98;

// ---------- PID ----------
float PRoll = 4.0, IRoll = 0.02, DRoll = 0.08;
float PPitch = 4.0, IPitch = 0.02, DPitch = 0.08;
float PYaw = 2.0, IYaw = 0.0, DYaw = 0.0;

float iRoll = 0, iPitch = 0, iYaw = 0;
float prevRollErr = 0, prevPitchErr = 0, prevYawErr = 0;

// Timing
unsigned long lastTime = 0;

// ---------- RX ----------
volatile int ch1_value = 1500;
volatile int ch2_value = 1500;
volatile int ch3_value = 1000;
volatile int ch4_value = 1500;

unsigned long ch1_start, ch2_start, ch3_start, ch4_start;

// ---------- Motors ----------
Servo mot13, mot12, mot14, mot27;

/* ================= INTERRUPTS ================= */
void IRAM_ATTR ch1_ISR() { if (digitalRead(CH1)) ch1_start = micros(); else ch1_value = micros() - ch1_start; }
void IRAM_ATTR ch2_ISR() { if (digitalRead(CH2)) ch2_start = micros(); else ch2_value = micros() - ch2_start; }
void IRAM_ATTR ch3_ISR() { if (digitalRead(CH3)) ch3_start = micros(); else ch3_value = micros() - ch3_start; }
void IRAM_ATTR ch4_ISR() { if (digitalRead(CH4)) ch4_start = micros(); else ch4_value = micros() - ch4_start; }

/* ================= SENSOR ================= */
void readGyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  GyroXraw = Wire.read() << 8 | Wire.read();
  GyroYraw = Wire.read() << 8 | Wire.read();
  GyroZraw = Wire.read() << 8 | Wire.read();

  gyroRoll  = GyroXraw / 65.5 - gyroRollOffset;
  gyroPitch = GyroYraw / 65.5 - gyroPitchOffset;
  gyroYaw   = GyroZraw / 65.5 - gyroYawOffset;
}

void readAccel() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  AccXraw = (Wire.read() << 8 | Wire.read()) - accXOffset;
  AccYraw = (Wire.read() << 8 | Wire.read()) - accYOffset;
  AccZraw = (Wire.read() << 8 | Wire.read()) - accZOffset;

  accRoll  = atan2(AccYraw, AccZraw) * 57.2958;
  accPitch = atan2(-AccXraw,
                  sqrt((float)AccYraw * AccYraw +
                       (float)AccZraw * AccZraw)) * 57.2958;
}

/* ================= FILTER ================= */
void updateAngles(float dt) {
  float gyroRollAngle  = rollAngle  + gyroRoll  * dt;
  float gyroPitchAngle = pitchAngle + gyroPitch * dt;

  rollAngle  = alpha * gyroRollAngle  + (1.0 - alpha) * accRoll;
  pitchAngle = alpha * gyroPitchAngle + (1.0 - alpha) * accPitch;
}

/* ================= PID ================= */
float pidUpdate(float error, float &prevError, float &iTerm,
                float kp, float ki, float kd) {

  const float dt = 0.004;

  float p = kp * error;
  iTerm += ki * (error + prevError) * dt * 0.5;
  iTerm = constrain(iTerm, -400, 400);

  float d = kd * (error - prevError) / dt;

  prevError = error;
  return constrain(p + iTerm + d, -400, 400);
}

/* ================= CALIBRATION ================= */
void calibrateSensors() {

  Serial.println("CALIBRATING GYRO...");
  delay(2000);

  for (int i = 0; i < 4000; i++) {
    readGyro();
    gyroRollOffset  += gyroRoll;
    gyroPitchOffset += gyroPitch;
    gyroYawOffset   += gyroYaw;
    delay(1);
  }

  gyroRollOffset  /= 4000;
  gyroPitchOffset /= 4000;
  gyroYawOffset   /= 4000;

  Serial.println("CALIBRATING ACCEL...");
  delay(2000);

  long ax = 0, ay = 0, az = 0;

  for (int i = 0; i < 4000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    ax += Wire.read() << 8 | Wire.read();
    ay += Wire.read() << 8 | Wire.read();
    az += Wire.read() << 8 | Wire.read();
    delay(1);
  }

  accXOffset = ax / 4000.0;
  accYOffset = ay / 4000.0;
  accZOffset = (az / 4000.0) - 16384.0;

  readAccel();
  rollAngle = accRoll;
  pitchAngle = accPitch;

  Serial.println("CALIBRATION DONE");
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);

  attachInterrupt(CH1, ch1_ISR, CHANGE);
  attachInterrupt(CH2, ch2_ISR, CHANGE);
  attachInterrupt(CH3, ch3_ISR, CHANGE);
  attachInterrupt(CH4, ch4_ISR, CHANGE);

  mot13.attach(M13_PIN, 1000, 2000);
  mot12.attach(M12_PIN, 1000, 2000);
  mot14.attach(M14_PIN, 1000, 2000);
  mot27.attach(M27_PIN, 1000, 2000);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  calibrateSensors();

  lastTime = micros();
  Serial.println("READY");
}

/* ================= LOOP ================= */
void loop() {

  // Safety: motors off
  if (ch3_value < 900) {
    mot13.writeMicroseconds(1000);
    mot12.writeMicroseconds(1000);
    mot14.writeMicroseconds(1000);
    mot27.writeMicroseconds(1000);
    iRoll = iPitch = iYaw = 0;
    return;
  }

  // Timing
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6;
  lastTime = now;
  if (dt <= 0 || dt > 0.02) return;

  // Sensor update
  readGyro();
  readAccel();
  updateAngles(dt);

  // RC input
  int thr = constrain(ch3_value, 1060, 2000);

  float desRoll  = (ch1_value - 1494) * 0.06;
  float desPitch = (ch2_value - 1462) * 0.06;
  float desYaw   = (ch4_value - 1470) * 0.15;

  // PID
  float rollPID  = pidUpdate(desRoll - rollAngle,  prevRollErr,  iRoll,  PRoll,  IRoll,  DRoll);
  float pitchPID = pidUpdate(desPitch - pitchAngle, prevPitchErr, iPitch, PPitch, IPitch, DPitch);
  float yawPID   = pidUpdate(desYaw - gyroYaw, prevYawErr, iYaw, PYaw, IYaw, DYaw);

  // Motor mixing
  int m13 = thr - rollPID - pitchPID - yawPID;
  int m12 = thr - rollPID + pitchPID + yawPID;
  int m14 = thr + rollPID + pitchPID - yawPID;
  int m27 = thr + rollPID - pitchPID + yawPID;

  // Output
  mot13.writeMicroseconds(constrain(m13, 1060, 2000));
  mot12.writeMicroseconds(constrain(m12, 1060, 2000));
  mot14.writeMicroseconds(constrain(m14, 1060, 2000));
  mot27.writeMicroseconds(constrain(m27, 1060, 2000));

  // Debug
  Serial.print("Roll: "); Serial.print(rollAngle);
  Serial.print(" Pitch: "); Serial.print(pitchAngle);
  Serial.print(" | M: ");
  Serial.print(m13); Serial.print(" ");
  Serial.print(m12); Serial.print(" ");
  Serial.print(m14); Serial.print(" ");
  Serial.println(m27);
}
