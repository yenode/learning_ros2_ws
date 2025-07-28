# ROS2 Command Cheat Sheet 🤖

A quick reference guide for essential ROS2 commands. These commands help you create, build, and interact with your robotic applications.

---

## 📁 Workspace & Package Management

These commands are for setting up your ROS2 workspace and creating packages.

* **Create a directory (folder):**
    A general Linux command to create a new folder for your workspace.
    ```bash
    mkdir -p ros2_ws/src
    ```

* **Create a Python Package:**
    Creates a new Python-based ROS2 package.
    ```bash
    ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
    ```

* **Create a C++ Package:**
    Creates a new C++ based ROS2 package.
    ```bash
    ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
    ```

---

## ⚙️ Building & Sourcing

After creating or modifying your packages, you need to build them and source the workspace.

* **Build Your Workspace:**
    Compile all packages in the current workspace using `colcon`. Run this from the root of your workspace (e.g., `ros2_ws`).
    ```bash
    colcon build
    ```

* **Build a Specific Package:**
    Save time by compiling only one package.
    ```bash
    colcon build --packages-select <your_package_name>
    ```

* **Source the Workspace:**
    **Crucial step!** Updates your terminal session to find and use your new packages. You must run this in every new terminal you open.
    ```bash
    source install/setup.bash
    ```

---

## ▶️ Execution Commands

Commands to run your nodes and launch multiple nodes at once.

* **Make a Python Node Executable:**
    Give a Python script permission to be executed as a program.
    ```bash
    chmod +x my_python_node.py
    ```

* **Run a Single Node:**
    Execute a specific node from a package.
    ```bash
    ros2 run <package_name> <executable_name>
    ```

* **Launch Multiple Nodes:**
    Use a launch file to start up a complex system with multiple nodes.
    ```bash
    ros2 launch <package_name> <launch_file_name>
    ```

---

## 🔎 Introspection & Debugging

Use these commands to inspect the state of your running ROS2 system.

* **List Running Nodes:**
    See all active nodes in the ROS2 graph.
    ```bash
    ros2 node list
    ```

* **List Active Topics:**
    See all topics that are currently being published or subscribed to.
    ```bash
    ros2 topic list
    ```

* **Echo a Topic:**
    Display the messages being published on a specific topic in real-time.
    ```bash
    ros2 topic echo /topic_name
    ```

* **Get Info on a Node:**
    Shows a node's subscriptions, publications, services, and actions.
    ```bash
    ros2 node info /node_name
    ```
