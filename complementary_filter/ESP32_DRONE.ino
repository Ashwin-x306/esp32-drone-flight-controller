#include <Wire.h>
#include <Arduino.h>

// ================== LOOP ==================
#define LOOP_DT 0.004

uint32_t looptimer;

// ================== RECEIVER (PWM) ==================
#define CH1_PIN 34  // Roll
#define CH2_PIN 35  // Pitch
#define CH3_PIN 32  // Throttle
#define CH4_PIN 33  // Yaw

volatile uint32_t start_time[4];
volatile uint16_t ReceiverValue[4] = {1500,1500,1000,1500};

// ================== ESC ==================
#define M1_PIN 13
#define M2_PIN 12
#define M3_PIN 14
#define M4_PIN 27

// ================== MPU ==================
#define MPU_ADDR 0x68

float acc_x, acc_y, acc_z;
float rateroll, ratepitch, rateyaw;
float rateroll_offset=0, ratepitch_offset=0, rateyaw_offset=0;
float ax_off=0, ay_off=0, az_off=0;

float angle_roll, angle_pitch;

// ================== KALMAN ==================
float kalman_roll=0, kalman_pitch=0;
float kalman_roll_unc=2.2, kalman_pitch_unc=2.2;

// ================== PID ==================
float DesiredAngleRoll=0, DesiredAnglePitch=0;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;

float ErrorAngleRoll, ErrorAnglePitch;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;

float PrevErrorAngleRoll=0, PrevErrorAnglePitch=0;
float PrevErrorRateRoll=0, PrevErrorRatePitch=0, PrevErrorRateYaw=0;

float ItermAngleRoll=0, ItermAnglePitch=0;
float ItermRateRoll=0, ItermRatePitch=0, ItermRateYaw=0;

float PAngle=2, IAngle=0, DAngle=0;
float PRate=0.6, IRate=3.5, DRate=0.03;
float PRateYaw=2, IRateYaw=12, DRateYaw=0;

// ================== MOTOR ==================
float InputThrottle;
float InputRoll, InputPitch, InputYaw;

float Motor1, Motor2, Motor3, Motor4;

// ================== ARM ==================
bool armed = false;

// ================== INTERRUPTS ==================
void IRAM_ATTR ch1_interrupt() {
  if (digitalRead(CH1_PIN)) start_time[0]=micros();
  else ReceiverValue[0]=micros()-start_time[0];
}
void IRAM_ATTR ch2_interrupt() {
  if (digitalRead(CH2_PIN)) start_time[1]=micros();
  else ReceiverValue[1]=micros()-start_time[1];
}
void IRAM_ATTR ch3_interrupt() {
  if (digitalRead(CH3_PIN)) start_time[2]=micros();
  else ReceiverValue[2]=micros()-start_time[2];
}
void IRAM_ATTR ch4_interrupt() {
  if (digitalRead(CH4_PIN)) start_time[3]=micros();
  else ReceiverValue[3]=micros()-start_time[3];
}

// ================== FUNCTIONS ==================
void mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08);
  Wire.endTransmission();
}

void read_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR,6);

  int16_t Ax=Wire.read()<<8|Wire.read();
  int16_t Ay=Wire.read()<<8|Wire.read();
  int16_t Az=Wire.read()<<8|Wire.read();

  acc_x=(Ax-ax_off)/4096.0;
  acc_y=(Ay-ay_off)/4096.0;
  acc_z=(Az-az_off)/4096.0;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR,6);

  int16_t Gx=Wire.read()<<8|Wire.read();
  int16_t Gy=Wire.read()<<8|Wire.read();
  int16_t Gz=Wire.read()<<8|Wire.read();

  rateroll=(Gx/65.5)-rateroll_offset;
  ratepitch=(Gy/65.5)-ratepitch_offset;
  rateyaw=(Gz/65.5)-rateyaw_offset;

  angle_roll = atan(acc_y / sqrt(acc_x*acc_x + acc_z*acc_z)) * (180.0/3.141592);
  angle_pitch = -atan(acc_x / sqrt(acc_y*acc_y + acc_z*acc_z)) * (180.0/3.141592);
}

void kalman_1d(float &state, float &unc, float rate, float meas){
  state += LOOP_DT*rate;
  unc += LOOP_DT*LOOP_DT*16;

  float gain = unc/(unc+9);
  state += gain*(meas-state);
  unc *= (1-gain);
}

float pid(float error, float &prev, float &iterm, float kp, float ki, float kd){
  float p = kp*error;
  iterm += ki*(error+prev)*LOOP_DT/2;
  iterm = constrain(iterm,-400,400);
  float d = kd*(error-prev)/LOOP_DT;
  float out = p+iterm+d;
  out = constrain(out,-400,400);
  prev = error;
  return out;
}

// ================== SETUP ==================
void setup(){
  Serial.begin(115200);

  Wire.begin(21,22);
  Wire.setClock(400000);

  // Receiver pins
  pinMode(CH1_PIN,INPUT);
  pinMode(CH2_PIN,INPUT);
  pinMode(CH3_PIN,INPUT);
  pinMode(CH4_PIN,INPUT);

  attachInterrupt(digitalPinToInterrupt(CH1_PIN),ch1_interrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN),ch2_interrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN),ch3_interrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4_PIN),ch4_interrupt,CHANGE);

  // ESC setup
  ledcSetup(0, 250, 11);
  ledcSetup(1, 250, 11);
  ledcSetup(2, 250, 11);
  ledcSetup(3, 250, 11);

  ledcAttachPin(M1_PIN,0);
  ledcAttachPin(M2_PIN,1);
  ledcAttachPin(M3_PIN,2);
  ledcAttachPin(M4_PIN,3);

  mpu_init();
  delay(2000);

  looptimer = micros();
}

// ================== LOOP ==================
void loop(){

  read_mpu();

  kalman_1d(kalman_roll,kalman_roll_unc,rateroll,angle_roll);
  kalman_1d(kalman_pitch,kalman_pitch_unc,ratepitch,angle_pitch);

  uint16_t ch[4];
  noInterrupts();
  for(int i=0;i<4;i++) ch[i]=ReceiverValue[i];
  interrupts();

  InputRoll = ch[0]-1500;
  InputPitch = ch[1]-1500;
  InputThrottle = ch[2];
  InputYaw = ch[3]-1500;

  // ================== ARM LOGIC ==================
  if (InputThrottle < 1050 && InputYaw < -400) armed = true;
  if (InputThrottle < 1050 && InputYaw > 400) armed = false;

  if(!armed){
    ledcWrite(0,1000);
    ledcWrite(1,1000);
    ledcWrite(2,1000);
    ledcWrite(3,1000);
    return;
  }

  DesiredAngleRoll = 0.1 * InputRoll;
  DesiredAnglePitch = 0.1 * InputPitch;
  DesiredRateYaw = 0.15 * InputYaw;

  ErrorAngleRoll = DesiredAngleRoll - kalman_roll;
  ErrorAnglePitch = DesiredAnglePitch - kalman_pitch;

  DesiredRateRoll = pid(ErrorAngleRoll,PrevErrorAngleRoll,ItermAngleRoll,PAngle,IAngle,DAngle);
  DesiredRatePitch = pid(ErrorAnglePitch,PrevErrorAnglePitch,ItermAnglePitch,PAngle,IAngle,DAngle);

  ErrorRateRoll = DesiredRateRoll - rateroll;
  ErrorRatePitch = DesiredRatePitch - ratepitch;
  ErrorRateYaw = DesiredRateYaw - rateyaw;

  InputRoll = pid(ErrorRateRoll,PrevErrorRateRoll,ItermRateRoll,PRate,IRate,DRate);
  InputPitch = pid(ErrorRatePitch,PrevErrorRatePitch,ItermRatePitch,PRate,IRate,DRate);
  InputYaw = pid(ErrorRateYaw,PrevErrorRateYaw,ItermRateYaw,PRateYaw,IRateYaw,DRateYaw);

  Motor1 = constrain(1.024*(InputThrottle - InputRoll - InputPitch - InputYaw),1180,2000);
  Motor2 = constrain(1.024*(InputThrottle - InputRoll + InputPitch + InputYaw),1180,2000);
  Motor3 = constrain(1.024*(InputThrottle + InputRoll + InputPitch - InputYaw),1180,2000);
  Motor4 = constrain(1.024*(InputThrottle + InputRoll - InputPitch + InputYaw),1180,2000);

  ledcWrite(0,Motor1);
  ledcWrite(1,Motor2);
  ledcWrite(2,Motor3);
  ledcWrite(3,Motor4);

  Serial.print("ARM: "); Serial.print(armed);
  Serial.print(" M1: "); Serial.print(Motor1);
  Serial.print(" M2: "); Serial.print(Motor2);
  Serial.print(" M3: "); Serial.print(Motor3);
  Serial.print(" M4: "); Serial.println(Motor4);

  while(micros()-looptimer<4000);
  looptimer=micros();
}