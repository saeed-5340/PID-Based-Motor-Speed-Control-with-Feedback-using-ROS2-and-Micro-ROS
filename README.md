# PID-Based-Motor-Speed-Control-with-Feedback-using-ROS2-and-Micro-ROS
# 🚗 PID-Based Motor Speed Control using ROS2 and micro-ROS

This repository implements a **closed-loop motor speed control system** using a **PID controller in ROS2** and **low-level motor actuation via micro-ROS (ESP32/Arduino)**.

The system is designed for **differential drive robots**, where wheel speeds are controlled using encoder feedback and PID regulation.

---

## 📌 Overview

This project follows a **distributed robotics architecture**:

- 🧠 **ROS2 (High-Level Controller)**
  - Computes wheel speed from encoder data
  - Runs PID control loop
  - Publishes motor PWM commands

- ⚙️ **micro-ROS (Low-Level Controller)** *(to be added)*
  - Reads encoder data
  - Receives PWM commands
  - Drives motors using motor driver


## 🧠 Control Strategy

A **PID (Proportional–Integral–Derivative) controller** is used to regulate motor speed:

$$
output = K_p e + K_i \int e\,dt + K_d \frac{de}{dt}
$$

Where:
- `e = desired_speed - actual_speed`
- Output → PWM signal sent to motor driver

Encoder feedback is used to compute the real-time velocity of each wheel.

---

## ⚙️ Features

- ROS2-based PID motor control  
- Encoder feedback (closed-loop system)  
- Differential drive support  
- micro-ROS integration (planned)  
- Real-time control loop

---

## 🔄 System Workflow

1. Encoder ticks are received from micro-ROS  
2. ROS2 node computes wheel velocity  
3. PID controller calculates control output  
4. PWM commands are published  
5. micro-ROS (ESP32) applies PWM to motors  

---

## 📡 Topics

| Topic | Type | Description |
|------|------|-------------|
| `/get_ticks` | `Int32MultiArray` | Encoder values (Left, Right) |
| `/get_pwm_values` | `Int32MultiArray` | PWM outputs (Left, Right) |

---

## ⚙️ Parameters

| Parameter | Description |
|----------|------------|
| `wheel_separation` | Distance between wheels |
| `wheel_radius` | Radius of wheel |
| `encoder_ticks_per_revolution` | Encoder resolution |
| `pid_kp` | Proportional gain |
| `pid_ki` | Integral gain |
| `pid_kd` | Derivative gain |

---

## 🚀 Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/saeed-5340/PID-Based-Motor-Speed-Control-with-Feedback-using-ROS2-and-Micro-ROS.git
```
```bash
colcon build
source install/setup.bash
```
```bash
ros2 run pid_based_motor_control motor_speed_control_pid
```
