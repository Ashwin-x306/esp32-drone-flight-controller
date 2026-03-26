#include <Wire.h>
#include <Arduino.h>

// ================== LOOP TIMING ==================
#define LOOP_DT 0.004  // 4ms

uint32_t looptimer;

// ================== MPU6050 ==================
#define MPU_ADDR 0x68

float acc_x, acc_y, acc_z;
float rateroll, ratepitch, rateyaw;
float rateroll_offset = 0, ratepitch_offset = 0, rateyaw_offset = 0;
float ax_off = 0, ay_off = 0, az_off = 0;

// ================== ANGLES ==================
float angle_roll, angle_pitch;

// ================== KALMAN ==================
float kalman_roll = 0, kalman_pitch = 0;
float kalman_roll_unc = 2.2, kalman_pitch_unc = 2.2;

// ================== PID ==================
float DesiredAngleRoll = 0, DesiredAnglePitch = 0;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;

float ErrorAngleRoll, ErrorAnglePitch;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;

float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0;
float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;

float ItermAngleRoll = 0, ItermAnglePitch = 0;
float ItermRateRoll = 0, ItermRatePitch = 0, ItermRateYaw = 0;

// PID Gains
float PAngle = 2, IAngle = 0, DAngle = 0;
float PRate = 0.6, IRate = 3.5, DRate = 0.03;
float PRateYaw = 2, IRateYaw = 12, DRateYaw = 0;

// ================== MOTOR ==================
float InputThrottle = 1500;
float InputRoll, InputPitch, InputYaw;

float Motor1, Motor2, Motor3, Motor4;

// ================== FUNCTIONS ==================

void mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Low pass filter
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Accel ±8g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Gyro ±500 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void read_mpu() {
  // ACCEL
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);

  int16_t AccX = Wire.read() << 8 | Wire.read();
  int16_t AccY = Wire.read() << 8 | Wire.read();
  int16_t AccZ = Wire.read() << 8 | Wire.read();

  acc_x = (AccX - ax_off) / 4096.0;
  acc_y = (AccY - ay_off) / 4096.0;
  acc_z = (AccZ - az_off) / 4096.0;

  // GYRO
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 6);

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  rateroll = (GyroX / 65.5) - rateroll_offset;
  ratepitch = (GyroY / 65.5) - ratepitch_offset;
  rateyaw = (GyroZ / 65.5) - rateyaw_offset;

  // ANGLES
  angle_roll = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)) * (180.0 / 3.141592);
  angle_pitch = -atan(acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * (180.0 / 3.141592);
}

void kalman_1d(float &state, float &uncertainty, float rate, float measurement) {
  // Prediction
  state += LOOP_DT * rate;
  uncertainty += LOOP_DT * LOOP_DT * 16;

  // Update
  float gain = uncertainty / (uncertainty + 9);
  state += gain * (measurement - state);
  uncertainty *= (1 - gain);
}

float pid(float error, float &prev_error, float &iterm, float kp, float ki, float kd) {
  float p = kp * error;

  iterm += ki * (error + prev_error) * LOOP_DT / 2;
  iterm = constrain(iterm, -400, 400);

  float d = kd * (error - prev_error) / LOOP_DT;

  float output = p + iterm + d;
  output = constrain(output, -400, 400);

  prev_error = error;
  return output;
}

// ================== SETUP ==================

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  mpu_init();

  delay(2000);

  // CALIBRATION
  for (int i = 0; i < 2000; i++) {
    read_mpu();

    rateroll_offset += rateroll;
    ratepitch_offset += ratepitch;
    rateyaw_offset += rateyaw;

    ax_off += acc_x * 4096;
    ay_off += acc_y * 4096;
    az_off += acc_z * 4096;

    delay(2);
  }

  rateroll_offset /= 2000;
  ratepitch_offset /= 2000;
  rateyaw_offset /= 2000;

  ax_off /= 2000;
  ay_off /= 2000;
  az_off = az_off / 2000 - 4096;

  looptimer = micros();
}

// ================== LOOP ==================

void loop() {

  read_mpu();

  // KALMAN
  kalman_1d(kalman_roll, kalman_roll_unc, rateroll, angle_roll);
  kalman_1d(kalman_pitch, kalman_pitch_unc, ratepitch, angle_pitch);

  // OUTER LOOP (ANGLE)
  ErrorAngleRoll = DesiredAngleRoll - kalman_roll;
  ErrorAnglePitch = DesiredAnglePitch - kalman_pitch;

  DesiredRateRoll = pid(ErrorAngleRoll, PrevErrorAngleRoll, ItermAngleRoll, PAngle, IAngle, DAngle);
  DesiredRatePitch = pid(ErrorAnglePitch, PrevErrorAnglePitch, ItermAnglePitch, PAngle, IAngle, DAngle);

  // INNER LOOP (RATE)
  ErrorRateRoll = DesiredRateRoll - rateroll;
  ErrorRatePitch = DesiredRatePitch - ratepitch;
  ErrorRateYaw = DesiredRateYaw - rateyaw;

  InputRoll = pid(ErrorRateRoll, PrevErrorRateRoll, ItermRateRoll, PRate, IRate, DRate);
  InputPitch = pid(ErrorRatePitch, PrevErrorRatePitch, ItermRatePitch, PRate, IRate, DRate);
  InputYaw = pid(ErrorRateYaw, PrevErrorRateYaw, ItermRateYaw, PRateYaw, IRateYaw, DRateYaw);

  // MOTOR MIXING
  Motor1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
  Motor2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  Motor3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  Motor4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

  // LIMITS
  Motor1 = constrain(Motor1, 1180, 2000);
  Motor2 = constrain(Motor2, 1180, 2000);
  Motor3 = constrain(Motor3, 1180, 2000);
  Motor4 = constrain(Motor4, 1180, 2000);

  // DEBUG
  Serial.print("Roll: "); Serial.print(kalman_roll);
  Serial.print(" Pitch: "); Serial.print(kalman_pitch);
  Serial.print(" | M1: "); Serial.print(Motor1);
  Serial.print(" M2: "); Serial.print(Motor2);
  Serial.print(" M3: "); Serial.print(Motor3);
  Serial.print(" M4: "); Serial.println(Motor4);

  // LOOP CONTROL
  while (micros() - looptimer < 4000);
  looptimer = micros();
}