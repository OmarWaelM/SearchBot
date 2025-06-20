# SearchBot
A 4-wheel differential drive unmanned ground vehicle (UGV) developed using **ROS**, **Python**, and **C++**, capable of operating in both manual and autonomous modes. The system uses only a monocular IP camera and IMU (MPU6050) for perception and localization.

## Features

- **Dual Operation Modes**
  - **Manual Teleoperation**: Via keyboard or joystick
  - **Autonomous Navigation**: Search and known obstacle avoidance

- **Intelligent Perception**
  - Object detection using **YOLO**
  - Visual odometry with **MPU6050**


## Tech Stack

| Component         | Technology Used      |
|------------------|----------------------|
| Core Framework    | ROS (Robot Operating System) |
| Programming       | Python, C++          |
| Perception        | YOLO, OpenCV         |
| IMU Integration   | MPU6050              |
| Simulation (optional) | Gazebo, RViz     |

---

## Installation

### 1. Clone the repository
```bash
git clone https://github.com/your-username/stalker-ugv.git
cd stalker-ugv
