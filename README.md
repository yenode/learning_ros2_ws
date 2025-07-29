# Object & Color Detection Robot using ROS2, YOLOv8, and OpenCV

This project implements a real-time object and color detection system on a differential drive robot using **ROS2**, **Gazebo**, **Ultralytics YOLOv8**, and **OpenCV**. The robot can navigate a simulated environment and identify objects based on pre-trained models and specific color masks.

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Python](https://img.shields.io/badge/Python-3.12-yellow.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green.svg)
![Ultralytics](https://img.shields.io/badge/YOLOv8-red)

***

## 📋 Table of Contents

* [Key Features](#-key-features)
* [System Architecture](#-system-architecture)
* [Tech Stack](#-tech-stack)
* [Getting Started](#-getting-started)
    * [Prerequisites](#prerequisites)
    * [Installation & Setup](#installation--setup)
* [Usage](#-usage)
* [Results & Demonstration](#-results--demonstration)
    * [Simulated World](#a-simulated-world-in-gazebo)
    * [YOLOv8 Object Detection](#b-object-detection-with-yolov8)
    * [OpenCV Color Detection](#c-color-detection-with-opencv)

***

## ✨ Key Features

* **Real-time Object Detection**: Utilizes the powerful **YOLOv8** model from Ultralytics to detect a wide range of COCO objects.
* **Precise Color Detection**: Implements **HSV color space masking** with OpenCV to reliably detect objects of specific colors (e.g., bright red).
* **Realistic Simulation**: The robot and its environment are simulated using **Gazebo**, allowing for robust testing and development.
* **ROS2 Integration**: Built on ROS2, providing a modular and scalable framework for robotic applications.
* **Standardized Control**: Leverages `ros2_control` for streamlined hardware abstraction and `teleop_twist_keyboard` for easy manual control.

***

## 🏗️ System Architecture

### Robot Model

The robot is a differential drive model with two actuated wheels and a passive caster wheel for stability. Perception is handled by a forward-facing camera, and odometry is provided by an IMU sensor.

<img width="285" height="159" alt="Robot Model" src="https://github.com/user-attachments/assets/68bf383c-6d1a-4cd8-9e72-1a02f208d36b" />

### Control System

The robot's controllers are managed via the `ros2_control` framework. The node graph below shows the topics and services used for teleoperation, allowing keyboard commands to be translated into robot motion (`cmd_vel`).

<img width="562" height="439" alt="Control System Graph" src="https://github.com/user-attachments/assets/3d3eb6b3-f162-42d5-9da7-3ff978a5b2bf" />

***

## 🛠️ Tech Stack

* **Framework**: ROS2 Jazzy
* **Simulator**: Gazebo Harmonic
* **Object Detection**: Ultralytics YOLOv8
* **Computer Vision**: OpenCV
* **Robot Control**: `ros2_control`
* **Languages**: Python, XML (URDF/SDF)
* **Build System**: `colcon`

***

## 🚀 Getting Started

### Prerequisites

* Ubuntu 24.04
* ROS2 Jazzy Jalisco
* Gazebo for ROS2 (`ros-jazzy-ros-gz*`)
* Python 3.12 with pip
* `colcon` build tools

### Installation & Setup

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/yenode/ros2_yolo_opencv_robot.git
    cd <your_ws_name>
    ```

2.  **Install Python dependencies:**
    ```bash
    pip install requirements.txt
    ```

3.  **Build the ROS2 workspace:**
    ```bash
    cd <your_ws_name>
    colcon build 
    ```

4.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```

***

## 🎮 Usage

1.  **Launch the Simulation:**
    This command starts Gazebo with the custom world and spawns the robot.
    ```bash
    ros2 launch bot_description gazebo.launch.xml
    ```
    
2.  **Configure Robot Controller:**
    Open a new terminal, source the workspace, and run the bot_controller launch file.
    ```bash
    ros2 launch bot_controller bot_controller.launch.xml
    ```
    
3.  **Custom node for converting Twist messages into Twist Stamped messages:**
    Open a new terminal, source the workspace, and run the twist_stamper node.
    ```bash
    ros2 run depend_nodes twist_stamper
    ```
    
4.  **Control the Robot:**
    Open a new terminal, source the workspace, and run the teleop node.
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bot_controller/cmd_vel_unstamped
    ```
    
5.  **Run Object Detection Node:**
    ```bash
    ros2 run bot_vision object_detection
    ```
    
6.  **Run Color Detection Node:**
    ```bash
    ros2 run bot_vision color_detection
    ```

***

## 📈 Results & Demonstration

### A. Simulated World in Gazebo

The custom Gazebo world (`my_world.sdf`) is populated with various objects for testing both detection algorithms.

<img width="743" height="439" alt="Gazebo Simulation World" src="https://github.com/user-attachments/assets/bb32f47e-b9ca-440d-b18f-88d61ee36f14" />

### B. Object Detection with YOLOv8

The video below demonstrates real-time object detection as the robot moves through the environment. The YOLOv8 model successfully identifies and draws bounding boxes around objects like chairs, couches, and potted plants.

**[▶️ Watch YOLOv8 Detection Video](https://github.com/user-attachments/assets/6b2c4b91-7335-4944-b3c1-8cf80aca6e83)**

### C. Color Detection with OpenCV

This video shows the robot identifying a bright red ball using an HSV color mask. This method is highly effective for detecting objects with specific, consistent colors.

**[▶️ Watch Color Detection Video](https://github.com/user-attachments/assets/7a47e89b-eaa7-494f-85fc-9eaf3720daf5)**
