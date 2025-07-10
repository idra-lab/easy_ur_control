# Easy Universal Robot Control
This repository provides a simple and easy-to-use guide and a ROS2 package to control the Universal Robot robots using the Universal Robot ROS2 driver.

# 📦 Installation
1. Install ROS2 dependencies:
   ```bash
   sudo apt install ros-<distro>-ur-* ros-<distro>-ros2-control ros-<distro>-ros2-controllers
   ```
2. Create ros2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
3. Clone the repository:
    ```bash
    git clone
    ```
4. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```