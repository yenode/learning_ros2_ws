# Implemented Object Detection And Color Detection Using OpenCV and Ultralytics YOLOv8 For a Differential Drive Robot 

## 1. Robot Model Details

A basic three wheel(Two motion wheels and one caster wheel) differential drive robot with a Camera sensor for perception and IMU Sensor for odometry.

<img width="285" height="159" alt="image" src="https://github.com/user-attachments/assets/68bf383c-6d1a-4cd8-9e72-1a02f208d36b" />

## 2. Robot Control

ROS2 package ros2_control is used for configuring the robot's joints for effective control.

Robot can be moved using keyboard with the use of teleop_twist_keyboard node.

<img width="562" height="439" alt="image" src="https://github.com/user-attachments/assets/3d3eb6b3-f162-42d5-9da7-3ff978a5b2bf" />


## 3. Simulation

Gazebo simulator is used for creating a world and launching the robot in a custom world.

A custom world, with the file name my_world.sdf, contains some COCO Objects detectable by YOLO and some coloured objects.

Camera gets a view of the gazebo world and then the object_detection node and color_detection uses it for detection algorithms.

## 4. Results

### A. Simulated World

Below is the custom world simulated in gazebo with our diff_drive robot spawned in it.

<img width="743" height="439" alt="image" src="https://github.com/user-attachments/assets/bb32f47e-b9ca-440d-b18f-88d61ee36f14" />

### B. Object Detection Using Ultralytics YOLOv8

Below Video shows real-time object detection using YOLO on robot movement.

[Demonstration.webm](https://github.com/user-attachments/assets/6b2c4b91-7335-4944-b3c1-8cf80aca6e83)

### C. Color Detection with the help of HSV format using OpenCV

Below Video shows Bright red Ball detection on robot movement.

[color.webm](https://github.com/user-attachments/assets/7a47e89b-eaa7-494f-85fc-9eaf3720daf5)





