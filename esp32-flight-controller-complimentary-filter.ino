// ============================================================
//   ESP32 FLIGHT CONTROLLER
//   Updated for drone configuration
//   M1=GPIO27 M2=GPIO14 M3=GPIO12 M4=GPIO13
//   RC Calibrated to your transmitter values
//   + Emergency Stop CH5 (VrA knob GPIO25)
//   + Fixed loop timer bug
//   + Separate ISR per channel
// ============================================================

#include <Wire.h>
#include <ESP32Servo.h>

// ================== YOUR MOTOR PINS ==================
//        FRONT
//  M4(CW)13 ── M1(CCW)27
//       \  X  /
//  M3(CCW)12 ── M2(CW)14
//        REAR

const int mot1_pin = 27;   // Front-Right CCW
const int mot2_pin = 14;   // Rear-Right  CW
const int mot3_pin = 12;   // Rear-Left   CCW
const int mot4_pin = 13;   // Front-Left  CW

// ================== RC PINS ==================
const int channel_1_pin = 34;   // Roll
const int channel_2_pin = 35;   // Pitch
const int channel_3_pin = 32;   // Throttle
const int channel_4_pin = 33;   // Yaw
const int channel_5_pin = 25;   // Emergency Stop (VrA knob)

// ================== LED ==================
#define LED_PIN   15

// ================== ESC ==================
int ESCfreq     = 500;
int ThrottleIdle   = 1180;   // your ESC starts at 1180
int ThrottleCutOff = 1000;
int ThrottleMax    = 1800;

// ================== LOOP TIMING ==================
uint32_t LoopTimer;
float t = 0.004f;   // 4ms = 250Hz

// ================== MPU6050 ==================
#define MPU_ADDR  0x68

volatile float RateRoll, RatePitch, RateYaw;
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;

// ================== CALIBRATION ==================
// Auto-computed on boot — no hardcoded values!
float RateCalibrationRoll  = 0;
float RateCalibrationPitch = 0;
float RateCalibrationYaw   = 0;
float AccXCalibration      = 0;
float AccYCalibration      = 0;
float AccZCalibration      = 0;

// ================== COMPLEMENTARY FILTER ==================
float complementaryAngleRoll  = 0.0f;
float complementaryAnglePitch = 0.0f;

// ================== PID GAINS ==================
// Outer — Angle loop
float PAngleRoll  = 2.0f;  float PAnglePitch  = 2.0f;
float IAngleRoll  = 0.5f;  float IAnglePitch  = 0.5f;
float DAngleRoll  = 0.007f; float DAnglePitch = 0.007f;

// Inner — Rate loop Roll/Pitch
float PRateRoll  = 0.625f; float PRatePitch  = 0.625f;
float IRateRoll  = 2.1f;   float IRatePitch  = 2.1f;
float DRateRoll  = 0.0088f; float DRatePitch = 0.0088f;

// Inner — Rate loop Yaw
float PRateYaw = 4.0f;
float IRateYaw = 3.0f;
float DRateYaw = 0.0f;

// ================== PID STATE ==================
float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;

float DesiredAngleRoll=0,  DesiredAnglePitch=0;
float ErrorAngleRoll=0,    ErrorAnglePitch=0;
float PrevErrorAngleRoll=0, PrevErrorAnglePitch=0;
float PrevItermAngleRoll=0, PrevItermAnglePitch=0;

float DesiredRateRoll=0,   DesiredRatePitch=0,  DesiredRateYaw=0;
float ErrorRateRoll=0,     ErrorRatePitch=0,    ErrorRateYaw=0;
float PrevErrorRateRoll=0, PrevErrorRatePitch=0, PrevErrorRateYaw=0;
float PrevItermRateRoll=0, PrevItermRatePitch=0, PrevItermRateYaw=0;

float InputRoll=0, InputPitch=0, InputYaw=0, InputThrottle=0;

// ================== MOTOR OUTPUTS ==================
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// ================== MOTORS ==================
Servo mot1, mot2, mot3, mot4;

// ============================================================
//   INTERRUPT RECEIVER — Separate ISR per channel
// ============================================================
volatile uint32_t ch_start[5]  = {0, 0, 0, 0, 0};
volatile uint16_t ch_value[5]  = {1500, 1500, 1000, 1500, 2000};

// CH1 Roll
void IRAM_ATTR isr_ch1() {
  if (digitalRead(channel_1_pin)) ch_start[0] = micros();
  else ch_value[0] = (uint16_t)(micros() - ch_start[0]);
}
// CH2 Pitch
void IRAM_ATTR isr_ch2() {
  if (digitalRead(channel_2_pin)) ch_start[1] = micros();
  else ch_value[1] = (uint16_t)(micros() - ch_start[1]);
}
// CH3 Throttle
void IRAM_ATTR isr_ch3() {
  if (digitalRead(channel_3_pin)) ch_start[2] = micros();
  else ch_value[2] = (uint16_t)(micros() - ch_start[2]);
}
// CH4 Yaw
void IRAM_ATTR isr_ch4() {
  if (digitalRead(channel_4_pin)) ch_start[3] = micros();
  else ch_value[3] = (uint16_t)(micros() - ch_start[3]);
}
// CH5 Emergency Stop (VrA knob)
void IRAM_ATTR isr_ch5() {
  if (digitalRead(channel_5_pin)) ch_start[4] = micros();
  else ch_value[4] = (uint16_t)(micros() - ch_start[4]);
}

// Safe atomic read
void read_receiver(uint16_t* rc) {
  noInterrupts();
  for (int i = 0; i < 5; i++) rc[i] = ch_value[i];
  interrupts();
}

// ============================================================
//   MY RC CALIBRATION — actual transmitter values
//   Roll  : min=1110  mid=1509  max=1874
//   Pitch : min=1175  mid=1462  max=1821
//   Thr   : min=1099          max=1779
//   Yaw   : min=1074  mid=1470  max=1826
// ============================================================
float normalize_stick(float input, float mn, float mid, float mx) {
  float norm;
  if (input <= mid)
    norm = 1500.0f + (input - mid) * (500.0f / (mid - mn));
  else
    norm = 1500.0f + (input - mid) * (500.0f / (mx - mid));
  return constrain(norm, 1000.0f, 2000.0f);
}

float normalize_throttle(float input, float mn, float mx) {
  return constrain(
    (input - mn) / (mx - mn) * 1000.0f + 1000.0f,
    1000.0f, 2000.0f
  );
}

float apply_deadband(float val, float db) {
  if (abs(val) < db) return 0.0f;
  return val;
}

// Roll → ±50° (0.1 * normalized offset from 1500)
float map_roll(float rc) {
  float norm = normalize_stick(rc, 1110, 1509, 1874);
  return apply_deadband(norm - 1500.0f, 10) * 0.1f;
}

// Pitch → ±50°
float map_pitch(float rc) {
  float norm = normalize_stick(rc, 1175, 1462, 1821);
  return apply_deadband(norm - 1500.0f, 10) * 0.1f;
}

// Yaw → ±75°/s
float map_yaw(float rc) {
  float norm = normalize_stick(rc, 1074, 1470, 1826);
  return apply_deadband(norm - 1500.0f, 10) * 0.15f;
}

// Throttle → 1000~1800
float map_throttle(float rc) {
  float norm = normalize_throttle(rc, 1099, 1779);
  return constrain(
    (norm - 1000.0f) / 1000.0f * (ThrottleMax - ThrottleCutOff) + ThrottleCutOff,
    ThrottleCutOff, ThrottleMax
  );
}

// ============================================================
//   LED HELPER
// ============================================================
void blink(int times, int ms = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(ms);
    digitalWrite(LED_PIN, LOW);  delay(ms);
  }
}

// ============================================================
//   MPU6050 INIT
// ============================================================
void mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);   // Wake up
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x05);   // DLPF
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x10);   // Accel ±8g
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08);   // Gyro ±500°/s
  Wire.endTransmission();
}

// ============================================================
//   MPU6050 READ
// ============================================================
void read_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t AccXLSB = Wire.read()<<8 | Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | Wire.read();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();

  RateRoll  = (float)GyroX / 65.5f;
  RatePitch = (float)GyroY / 65.5f;
  RateYaw   = (float)GyroZ / 65.5f;

  AccX = (float)AccXLSB / 4096.0f;
  AccY = (float)AccYLSB / 4096.0f;
  AccZ = (float)AccZLSB / 4096.0f;

  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;
  AccX      -= AccXCalibration;
  AccY      -= AccYCalibration;
  AccZ      -= AccZCalibration;

  AngleRoll  =  atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 57.29f;
  AnglePitch = -atan(AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 57.29f;
}

// ============================================================
//   AUTO CALIBRATION — runs on boot
// ============================================================
void calibrate_mpu() {
  Serial.println("============================================");
  Serial.println("        MPU6050 AUTO CALIBRATION");
  Serial.println("============================================");
  Serial.println(">> Place drone FLAT and STILL!");

  for (int i = 3; i > 0; i--) {
    Serial.print(">> Starting in "); Serial.print(i); Serial.println("...");
    blink(1, 400);
  }

  const int N = 2000;
  long gx_sum=0, gy_sum=0, gz_sum=0;
  long ax_sum=0, ay_sum=0, az_sum=0;

  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    ax_sum += (int16_t)(Wire.read()<<8 | Wire.read());
    ay_sum += (int16_t)(Wire.read()<<8 | Wire.read());
    az_sum += (int16_t)(Wire.read()<<8 | Wire.read());

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    gx_sum += (int16_t)(Wire.read()<<8 | Wire.read());
    gy_sum += (int16_t)(Wire.read()<<8 | Wire.read());
    gz_sum += (int16_t)(Wire.read()<<8 | Wire.read());

    if (i % 500 == 0) {
      Serial.print(">> Progress: ");
      Serial.print((i * 100) / N);
      Serial.println("%");
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    delay(1);
  }

  RateCalibrationRoll  = (gx_sum / (float)N) / 65.5f;
  RateCalibrationPitch = (gy_sum / (float)N) / 65.5f;
  RateCalibrationYaw   = (gz_sum / (float)N) / 65.5f;

  AccXCalibration =  ax_sum / (float)N / 4096.0f;
  AccYCalibration =  ay_sum / (float)N / 4096.0f;
  AccZCalibration = (az_sum / (float)N / 4096.0f) - 1.0f;

  Serial.println(">> Progress: 100%");
  Serial.println("============================================");
  Serial.println("         CALIBRATION COMPLETE!");
  Serial.println("============================================");
  Serial.print("Gyro  → Roll:");  Serial.print(RateCalibrationRoll,  4);
  Serial.print("  Pitch:");        Serial.print(RateCalibrationPitch, 4);
  Serial.print("  Yaw:");          Serial.println(RateCalibrationYaw, 4);
  Serial.print("Accel → X:");      Serial.print(AccXCalibration, 4);
  Serial.print("  Y:");            Serial.print(AccYCalibration, 4);
  Serial.print("  Z:");            Serial.println(AccZCalibration, 4);
  Serial.println("============================================\n");

  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

// ============================================================
//   RESET PID
// ============================================================
void reset_pid() {
  PrevErrorRateRoll=0;   PrevErrorRatePitch=0;   PrevErrorRateYaw=0;
  PrevItermRateRoll=0;   PrevItermRatePitch=0;   PrevItermRateYaw=0;
  PrevErrorAngleRoll=0;  PrevErrorAnglePitch=0;
  PrevItermAngleRoll=0;  PrevItermAnglePitch=0;
  complementaryAngleRoll=0; complementaryAnglePitch=0;
}

// ============================================================
//   SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  blink(5, 100);

  Serial.println("============================================");
  Serial.println("   ESP32 FLIGHT CONTROLLER BOOTING");
  Serial.println("   M1=GPIO27 M2=GPIO14 M3=GPIO12 M4=GPIO13");
  Serial.println("============================================");

  // I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(250);

  // MPU check
  mpu_init();
  delay(250);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  byte who = Wire.read();

  // if (who == 0x68) {
  //   Serial.println(">> MPU6050 ✅ Found!");
  // } else {
  //   Serial.print(">> MPU6050 ❌ Not found! WHO_AM_I=0x");
  //   Serial.println(who, HEX);
  //   blink(10, 100);
  // }

  // RC Receiver pins
  // GPIO 34,35 = input only (no pullup)
  // GPIO 32,33,25 = normal with pullup
  pinMode(channel_1_pin, INPUT);
  pinMode(channel_2_pin, INPUT);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);
  pinMode(channel_5_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), isr_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), isr_ch2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), isr_ch3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), isr_ch4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_5_pin), isr_ch5, CHANGE);

  Serial.println(">> RC Receiver ✅ Ready (5 channels)");
  Serial.println(">> Waiting for RC signal (2s)...");
  delay(2000);

  // Show RC values
  uint16_t rc[5];
  read_receiver(rc);
  Serial.println(">> Raw RC Values:");
  Serial.print("   CH1(Roll)=");    Serial.print(rc[0]);
  Serial.print("  CH2(Pitch)=");    Serial.print(rc[1]);
  Serial.print("  CH3(Thr)=");      Serial.print(rc[2]);
  Serial.print("  CH4(Yaw)=");      Serial.print(rc[3]);
  Serial.print("  CH5(Estop)=");    Serial.println(rc[4]);

  // ESC setup — setPeriodHertz BEFORE attach
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  mot1.setPeriodHertz(ESCfreq);
  mot2.setPeriodHertz(ESCfreq);
  mot3.setPeriodHertz(ESCfreq);
  mot4.setPeriodHertz(ESCfreq);

  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);

  // Min signal for ESC init
  mot1.writeMicroseconds(ThrottleCutOff);
  mot2.writeMicroseconds(ThrottleCutOff);
  mot3.writeMicroseconds(ThrottleCutOff);
  mot4.writeMicroseconds(ThrottleCutOff);

  Serial.println(">> ESCs ✅ Initialized");
  Serial.println(">> Waiting for ESC beeps (3s)...");
  delay(3000);

  blink(2, 300);

  // Auto Calibrate MPU
  calibrate_mpu();

  Serial.println("============================================");
  Serial.println("            ✅ SYSTEM READY!");
  Serial.println("   Throttle < 1030  → Motors OFF");
  Serial.println("   CH5 VrA LEFT     → Emergency STOP");
  Serial.println("============================================\n");

  blink(3, 200);
  LoopTimer = micros();
}

// ============================================================
//   MAIN LOOP — Strict 250Hz
// ============================================================
void loop() {

  // ── 1. READ MPU ──────────────────────────────────────────
  read_mpu();

  // ── 2. EMERGENCY STOP — CH5 VrA knob left ────────────────
  // Check FIRST before anything else!
  if (ch_value[4] < 1500) {
    mot1.writeMicroseconds(ThrottleCutOff);
    mot2.writeMicroseconds(ThrottleCutOff);
    mot3.writeMicroseconds(ThrottleCutOff);
    mot4.writeMicroseconds(ThrottleCutOff);
    reset_pid();
    Serial.println("🚨 EMERGENCY STOP — VrA knob!");
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
    return;
  }

  // ── 3. COMPLEMENTARY FILTER ──────────────────────────────
  complementaryAngleRoll  = 0.991f * (complementaryAngleRoll  + RateRoll  * t)
                          + 0.009f * AngleRoll;
  complementaryAnglePitch = 0.991f * (complementaryAnglePitch + RatePitch * t)
                          + 0.009f * AnglePitch;

  // Clamp ±20°
  complementaryAngleRoll  = constrain(complementaryAngleRoll,  -20.0f, 20.0f);
  complementaryAnglePitch = constrain(complementaryAnglePitch, -20.0f, 20.0f);

  // ── 4. READ RECEIVER ─────────────────────────────────────
  uint16_t rc[5];
  read_receiver(rc);

  float rc_roll     = rc[0];
  float rc_pitch    = rc[1];
  float rc_throttle = rc[2];
  float rc_yaw      = rc[3];

  // ── 5. THROTTLE CUTOFF ───────────────────────────────────
  if (rc_throttle < 1030) {
    mot1.writeMicroseconds(ThrottleCutOff);
    mot2.writeMicroseconds(ThrottleCutOff);
    mot3.writeMicroseconds(ThrottleCutOff);
    mot4.writeMicroseconds(ThrottleCutOff);
    reset_pid();
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
    return;
  }

  // ── 6. MAP RC → DESIRED VALUES ───────────────────────────
  DesiredAngleRoll  = map_roll(rc_roll);
  DesiredAnglePitch = map_pitch(rc_pitch);
  DesiredRateYaw    = map_yaw(rc_yaw);
  InputThrottle     = map_throttle(rc_throttle);

  if (InputThrottle > ThrottleMax) InputThrottle = ThrottleMax;

  // ── 7. OUTER PID — ANGLE LOOP ────────────────────────────
  // Roll
  ErrorAngleRoll  = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll       = PAngleRoll * ErrorAngleRoll;
  ItermRoll       = PrevItermAngleRoll
                  + IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t/2.0f);
  ItermRoll       = constrain(ItermRoll, -400.0f, 400.0f);
  DtermRoll       = DAngleRoll * (ErrorAngleRoll - PrevErrorAngleRoll) / t;
  PIDOutputRoll   = constrain(PtermRoll + ItermRoll + DtermRoll, -400.0f, 400.0f);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll  = ErrorAngleRoll;
  PrevItermAngleRoll  = ItermRoll;

  // Pitch
  ErrorAnglePitch  = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch       = PAnglePitch * ErrorAnglePitch;
  ItermPitch       = PrevItermAnglePitch
                   + IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t/2.0f);
  ItermPitch       = constrain(ItermPitch, -400.0f, 400.0f);
  DtermPitch       = DAnglePitch * (ErrorAnglePitch - PrevErrorAnglePitch) / t;
  PIDOutputPitch   = constrain(PtermPitch + ItermPitch + DtermPitch, -400.0f, 400.0f);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // ── 8. INNER PID — RATE LOOP ─────────────────────────────
  // Roll rate
  ErrorRateRoll   = DesiredRateRoll - RateRoll;
  PtermRoll       = PRateRoll * ErrorRateRoll;
  ItermRoll       = PrevItermRateRoll
                  + IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t/2.0f);
  ItermRoll       = constrain(ItermRoll, -400.0f, 400.0f);
  DtermRoll       = DRateRoll * (ErrorRateRoll - PrevErrorRateRoll) / t;
  InputRoll       = constrain(PtermRoll + ItermRoll + DtermRoll, -400.0f, 400.0f);
  PrevErrorRateRoll  = ErrorRateRoll;
  PrevItermRateRoll  = ItermRoll;

  // Pitch rate
  ErrorRatePitch  = DesiredRatePitch - RatePitch;
  PtermPitch      = PRatePitch * ErrorRatePitch;
  ItermPitch      = PrevItermRatePitch
                  + IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t/2.0f);
  ItermPitch      = constrain(ItermPitch, -400.0f, 400.0f);
  DtermPitch      = DRatePitch * (ErrorRatePitch - PrevErrorRatePitch) / t;
  InputPitch      = constrain(PtermPitch + ItermPitch + DtermPitch, -400.0f, 400.0f);
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw rate
  ErrorRateYaw    = DesiredRateYaw - RateYaw;
  PtermYaw        = PRateYaw * ErrorRateYaw;
  ItermYaw        = PrevItermRateYaw
                  + IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t/2.0f);
  ItermYaw        = constrain(ItermYaw, -400.0f, 400.0f);
  DtermYaw        = DRateYaw * (ErrorRateYaw - PrevErrorRateYaw) / t;
  InputYaw        = constrain(PtermYaw + ItermYaw + DtermYaw, -400.0f, 400.0f);
  PrevErrorRateYaw   = ErrorRateYaw;
  PrevItermRateYaw   = ItermYaw;

  // ── 9. MOTOR MIX ─────────────────────────────────────────
  //        FRONT
  //  M4(CW)13 ── M1(CCW)27
  //       \  X  /
  //  M3(CCW)12 ── M2(CW)14
  //        REAR

  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw; // FR CCW
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw; // RR CW
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw; // RL CCW
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw; // FL CW

  // Clamp motors
  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);

  // ── 10. WRITE MOTORS ─────────────────────────────────────
  mot1.writeMicroseconds((int)MotorInput1);
  mot2.writeMicroseconds((int)MotorInput2);
  mot3.writeMicroseconds((int)MotorInput3);
  mot4.writeMicroseconds((int)MotorInput4);

  // ── 11. SERIAL DEBUG ─────────────────────────────────────
  Serial.print("Roll:");   Serial.print(complementaryAngleRoll,  1);
  Serial.print(" Pitch:"); Serial.print(complementaryAnglePitch, 1);
  Serial.print(" Thr:");   Serial.print((int)InputThrottle);
  Serial.print(" | M1:");  Serial.print((int)MotorInput1);
  Serial.print(" M2:");    Serial.print((int)MotorInput2);
  Serial.print(" M3:");    Serial.print((int)MotorInput3);
  Serial.print(" M4:");    Serial.println((int)MotorInput4);

  // ── 12. STRICT 250Hz LOOP TIMING — bug fixed ─────────────
  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
