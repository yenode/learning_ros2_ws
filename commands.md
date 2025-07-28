# Project Specific Commands

### 1. For launching Rviz and Gazebo with our diff_drive robot : 
```
ros2 launch bot_description gazebo.launch.xml
```

### 2. For launching bot Controller and Configuring it : 
```
ros2 launch bot_controller bot_controller.launch.xml
```

### 3. For launching a Custom Node which converts Twist Messages to Twist Stamped : 
```
ros2 run depend_nodes twist_stamper
```

### 4. For launching Teleop Keyboard : 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bot_controller/cmd_vel_unstamped
```

### 5. For launching Color Detection node : 
```
ros2 run bot_vision color_detection
```

### 5. For launching Object Detection node : 
```
ros2 run bot_vision object_detection
```

---

# Basic Commands For ROS2

### 1. For creating a new folder namely my_folder : 
```
mkdir my_folder
```

### 2. For deleting a new folder namely my_folder :
```
rm -rf my_folder
#
```

### 3. To make an executable :
```
chmod +x my_first_node.py
```

### 4. Creating a package in ros2 (C++) :
```
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

### 5. Creating a package in ros2 (PYTHON) :
```
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

### 6. Building our workspace inside of "_ws" folder only :
```
colcon build
```

### 7. Sourcing the terminal after building, VERY IMPORTANT
```
source install/setup.bash
```

### 8. List of running nodes
```
ros2 node list
```

### 9. List of topics being published or subscribed :
```
ros2 topic list
```

### 10. Launching Multiple Nodes using a launch file :
```
ros2 launch bot_description gazebo.launch.xml
```

---
