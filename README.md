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
```
### 2. Setup ROS Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cp -r ~/stalker-ugv/* .
cd ..
catkin_make
source devel/setup.bash
```
### 3. Install Dependencies
```bash
sudo apt-get update
sudo apt-get install ros-<your-ros-distro>-slam-gmapping ros-<your-ros-distro>-joy
pip install opencv-python numpy pyttsx3 SpeechRecognition
```
## Usage

