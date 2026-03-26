# 🚁 ESP32 Quadcopter Flight Controller

A fully custom, from-scratch flight controller built on the **ESP32** microcontroller. No libraries, no shortcuts — raw I2C, hand-tuned PID, and a Kalman filter running at 250Hz.

---

## ✨ Features

- **Cascaded PID Control** — Outer angle loop feeds into inner rate loop for smooth, stable flight
- **1D Kalman Filter** — Fuses gyroscope and accelerometer data for accurate angle estimation
- **Raw I2C Communication** — Talks directly to the MPU-6050 with no external IMU libraries
- **Auto-Calibration** — On every boot, averages 2000 samples to zero out gyro drift and accel bias
- **Quadcopter Motor Mixing** — Correct X-frame mixing for Roll, Pitch, and Yaw authority
- **Precise 4ms Control Loop** — Hard 250Hz timing using `micros()` for deterministic control

---

## 🛠️ Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32 (I2C on GPIO 21/22) |
| IMU | MPU-6050 (Gyro ±500°/s, Accel ±8g) |
| Motors | 4x Brushless DC (via ESC, PWM 1180–2000µs) |
| Transmitter | Integrated (ESP32 onboard) |

---

## 🧠 How It Works

### 1. Sensor Reading
The MPU-6050 is read directly over I2C at 400kHz. Raw 16-bit values are converted to degrees/s (gyro) and g-forces (accel). A hardware low-pass filter is configured on the MPU for noise reduction.

### 2. Kalman Filter
A 1D Kalman filter fuses the accelerometer angle (absolute but noisy) with the gyroscope rate (fast but drifts) to produce a clean, accurate angle estimate for both Roll and Pitch.

```
Prediction:  state += dt * gyro_rate
Update:      state += K * (accel_angle - state)
```

### 3. Cascaded PID
Two PID loops run in series:

```
Desired Angle → [Angle PID] → Desired Rate → [Rate PID] → Motor Output
```

- **Angle loop** corrects the drone's tilt
- **Rate loop** controls how fast it rotates — giving crisp, locked-in feel

### 4. Motor Mixing
Standard X-frame quadcopter mixing:

```
Motor1 (Front-Left)  = Throttle - Roll - Pitch - Yaw
Motor2 (Rear-Left)   = Throttle - Roll + Pitch + Yaw
Motor3 (Front-Right) = Throttle + Roll + Pitch - Yaw
Motor4 (Rear-Right)  = Throttle + Roll - Pitch + Yaw
```

---

## ⚙️ PID Gains (Tuned)

| Loop | P | I | D |
|---|---|---|---|
| Angle | 2.0 | 0.0 | 0.0 |
| Rate (Roll/Pitch) | 0.6 | 3.5 | 0.03 |
| Rate (Yaw) | 2.0 | 12.0 | 0.0 |

---

## 🔧 Setup & Flash

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Add ESP32 board support via Board Manager
3. Connect ESP32 via USB
4. Open `flight_controller.ino`
5. Select board: `ESP32 Dev Module`
6. Upload & open Serial Monitor at **115200 baud**

> ⚠️ **Safety:** Always remove propellers when testing on the bench!

---

## 📊 Serial Debug Output

```
Roll: 0.42  Pitch: -0.18  | M1: 1524  M2: 1516  M3: 1531  M4: 1509
```

---

## 📁 File Structure

```
esp32-drone-flight-controller/
└── flight_controller.ino   # Main firmware (all-in-one)
```

---

## 🚀 Built By

> Made from scratch with C++, a soldering iron, and a lot of crash landings.

---

## 📜 License

MIT License — use it, learn from it, build on it.
