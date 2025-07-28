Basic Commands For ROS2

mkdir my_folder: For creating a new folder namely my_folder

rm -rf my_folder: for deleting a new folder namely my_folder

chmod +x my_first_node.py: To make an executable 

ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp: Creating a package in ros2 (C++)

ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy: Creating a package in ros2 (PYTHON)

colcon build: Building our workspace inside of "_ws" folder only

source install/setup.bash: sourcing the terminal after building, VERY IMPORTANT

ros2 node list: list of running nodes

ros2 topic list: list of topics being published or subscribed

ros2 launch bot_description gazebo.launch.xml: Launching Multiple Nodes using a launch file
