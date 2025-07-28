# Implemented Object Detection And Color Detection Using OpenCV and Ultralytics YOLOv8

## 1. Robot Model Details

A basic three wheel(Two motion wheels and one caster wheel) differential drive robot with a Camera sensor for perception and IMU Sensor for odometry.

<img width="285" height="159" alt="image" src="https://github.com/user-attachments/assets/68bf383c-6d1a-4cd8-9e72-1a02f208d36b" />

## 2. Robot Control

ROS2 package ros2_control is used for configuring the robot's joints for effective control.

Robot can be moved using keyboard with the use of teleop_twist_keyboard node.

## 3. Simulation

Gazebo simulator is used for creating a world and launching the robot in a custom world.

A custom world, with the file name my_world.sdf, contains some COCO Objects detectable by YOLO and some coloured objects.

Camera gets a view of the gazebo world and then the object_detection node and color_detection uses it for detection algorithms.

## Theoretical Portion for URDF (Unified Robot Description Format)

#### URDF is XML-based format that describes a robot's physical structure, visual appearance, and collision properties. It's the foundation for robot modeling in ROS.

### Key URDF Concepts
#### Links and Joints: URDF models robots as a tree structure of links connected by joints. Links represent rigid bodies (like robot arms, wheels, sensors), while joints define how links move relative to each other (revolute, prismatic, continuous, fixed).
#### Visual and Collision Elements: Each link can have visual geometry (what you see in visualization tools) and collision geometry (simplified shapes for physics calculations). These often differ - collision meshes are typically simpler for computational efficiency.
#### Inertial Properties: Links include mass, center of mass, and inertia tensors needed for physics simulation in Gazebo or other simulators.
#### Materials and Colors: URDF supports material definitions for visual appearance, including colors and textures.

### Transform Frames (TFs)
#### The TF system manages coordinate frame relationships throughout a robot system. It automatically computes transformations between any two coordinate frames in the robot.
#### Core TF Concepts
#### Coordinate Frames: Every sensor, joint, and important robot component has its own coordinate frame. Frames have position and orientation relative to other frames.
#### Transform Tree: All frames form a single tree structure with no loops. Each frame has exactly one parent (except the root frame, often "base_link" or "map").

### Integration: URDF + TF
#### The magic happens when URDF and TF work together through the robot_state_publisher node. This node reads the URDF file and joint states, then publishes the corresponding TF transforms for all robot links.
