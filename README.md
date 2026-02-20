# 🚜 Agro Weed Scout - Autonomous Precision Agriculture System

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-yellow)
![Python](https://img.shields.io/badge/Python-3.12-blue?logo=python)
![Hardware](https://img.shields.io/badge/Hardware-Raspberry_Pi_4_(8GB)-red?logo=raspberry-pi)

An autonomous robotic system designed for real-time weed detection and precision agriculture. Developed as a Mechatronics Engineering Capstone Project, this system utilizes a **Hardware-in-the-Loop (HIL)** architecture, offloading heavy visual processing to an edge-computing Raspberry Pi 4 while governing a high-fidelity 4WD skid-steer robot inside a Gazebo physics simulation.

---

## 🌟 Key Features

* **Edge AI Perception:** Powered by a YOLOv8 Nano (`yolov8n`) model optimized for the Raspberry Pi. Achieves an **89.2% mAP@50**, with a **97.4% recall for maize crops** and an **81.0% recall for weeds**.
* **Hardware-in-the-Loop Simulation:** A complete virtual twin of the robot built with URDF/Xacro, operating in **Gazebo Harmonic**. It features accurate inertial tensors, `.STL` collision meshes, and simulated wheel friction.
* **Virtual Sensor Fusion:** Simulates a 360-degree LiDAR (`/scan`), IMU (`/imu/data`), and optical camera (`/camera/image_raw`) to generate rich environmental data.
* **Closed-Loop Control:** Translates 2D bounding box data from the vision system into ROS 2 `geometry_msgs/Twist` commands to visually servo the robot toward targets.
* **Custom Control Terminal:** A Python Tkinter Graphical User Interface (GUI) featuring live telemetry, manual teleoperation overrides, autonomous toggling, and emergency stop protocols.

---

## 🏗️ System Architecture

The project operates on a distributed ROS 2 Publisher-Subscriber network:

1. **The "Brain" (Raspberry Pi 4 8GB):** Captures real-world physical camera data, runs the Neural Network inference, and publishes velocity commands (`/cmd_vel`) to the ROS network.
2. **The "Body" (Simulation Workstation):** Subscribes to the edge commands and actuates the `gz_ros2_control` hardware interface to move the virtual chassis in Gazebo.

---

## 🛠️ Prerequisites & Dependencies

To run the simulation and control system, ensure you have the following installed on your primary workstation:

* **OS:** Ubuntu 24.04 (Noble Numbat)
* **Middleware:** [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html)
* **Simulation:** Gazebo Harmonic (`ros-jazzy-ros-gz`)
* **Machine Learning:** PyTorch, OpenCV, Ultralytics YOLO (`pip install ultralytics`)

---

## 🚀 Installation & Setup

**1. Clone the repository:**
```bash
mkdir -p ~/agriscout_ws/src
cd ~/agriscout_ws/src
git clone [https://github.com/YOUR_USERNAME/agro-weed-scout.git](https://github.com/YOUR_USERNAME/agro-weed-scout.git)
