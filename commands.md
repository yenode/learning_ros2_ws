# Basic Commands For ROS2

### 1. For creating a new folder namely my_folder : 
mkdir my_folder 
# 
### 2. For deleting a new folder namely my_folder :
rm -rf my_folder
#
### 3. To make an executable :
chmod +x my_first_node.py
#
### 4. Creating a package in ros2 (C++) :
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
#
### 5. Creating a package in ros2 (PYTHON) :
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
#
### 6. Building our workspace inside of "_ws" folder only :
colcon build
#
### 7. Sourcing the terminal after building, VERY IMPORTANT
source install/setup.bash
#
### 8. List of running nodes
ros2 node list
#
### 9. List of topics being published or subscribed :
ros2 topic list
#
### 10. Launching Multiple Nodes using a launch file :
ros2 launch bot_description gazebo.launch.xml

---
