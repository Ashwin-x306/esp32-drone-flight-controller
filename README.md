# 🚁 ESP32 Quadcopter Flight Controller

A fully custom, from-scratch flight controller built on the **ESP32** microcontroller. No libraries, no shortcuts — raw I2C, PWM receiver input via interrupts, hand-tuned cascaded PID, and a Kalman filter running at 250Hz.

---

## ✨ Features

- **Cascaded PID Control** — Outer angle loop feeds into inner rate loop for smooth, stable flight
- **Complimentary Filter** — Fuses gyroscope and accelerometer data for accurate angle estimation
- **Raw I2C Communication** — Talks directly to the MPU-6050 with no external IMU libraries
- **PWM Receiver Input** — 4-channel RC input read via hardware interrupts for zero-latency response
- **Arm/Disarm Logic** — Safe arming sequence (Throttle low + Yaw right to arm, Yaw left to disarm)
- **Auto-Calibration** — Averages 2000 samples on boot to zero out gyro drift and accel bias
- **Quadcopter Motor Mixing** — Correct X-frame mixing for Roll, Pitch, and Yaw authority
- **Precise 4ms Control Loop** — Hard 250Hz timing using `micros()` for deterministic control

---

## 🛠️ Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32 |
| IMU | MPU-6050 (Gyro ±500°/s, Accel ±8g) |
| Motors | 4x Brushless DC via ESC |
| ESC Signal | PWM 250Hz, 11-bit (1180–2000µs range) |
| Receiver | 4-channel PWM RC Receiver |

---

## 📌 Pin Map

### MPU-6050 (I2C)
| MPU-6050 Pin | ESP32 Pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| VCC | 3.3V |
| GND | GND |

### ESC / Motors
| Motor | Position | ESP32 Pin | LEDC Channel |
|---|---|---|---|
| Motor 1 | Front-Left | GPIO 13 | CH 0 |
| Motor 2 | Rear-Left | GPIO 12 | CH 1 |
| Motor 3 | Front-Right | GPIO 14 | CH 2 |
| Motor 4 | Rear-Right | GPIO 27 | CH 3 |

### RC Receiver (PWM Input)
| Channel | Function | ESP32 Pin |
|---|---|---|
| CH1 | Roll | GPIO 34 |
| CH2 | Pitch | GPIO 35 |
| CH3 | Throttle | GPIO 32 |
| CH4 | Yaw | GPIO 33 |

> ⚠️ GPIO 34 & 35 are **input-only** pins on ESP32 — perfect for receiver signals.

---

## 🧠 How It Works

### 1. Sensor Reading
The MPU-6050 is read directly over I2C at 400kHz. Raw 16-bit values are converted to °/s (gyro) and g-forces (accel). A hardware low-pass filter is configured on the MPU for noise reduction.

### 2. Kalman Filter
A 1D Kalman filter fuses the accelerometer angle (absolute but noisy) with gyroscope rate (fast but drifts) to produce a clean, accurate angle estimate for Roll and Pitch.

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

### 4. RC Input via Interrupts
Each receiver channel is read using `attachInterrupt()` on CHANGE — measuring the PWM pulse width in microseconds. Values are safely copied out of the ISR using `noInterrupts()`.

### 5. Arm / Disarm
| Action | Stick Position |
|---|---|
| ✅ Arm | Throttle LOW + Yaw RIGHT (>1900µs) |
| ❌ Disarm | Throttle LOW + Yaw LEFT (<1100µs) |

When disarmed, all motors are held at minimum. The drone will not respond to any input until armed.

### 6. Motor Mixing (X-Frame)
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
| Angle (Roll/Pitch) | 2.0 | 0.0 | 0.0 |
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
ARM: 1  M1: 1524  M2: 1516  M3: 1531  M4: 1509
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
