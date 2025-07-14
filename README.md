# Easy Universal Robot Control
This repository provides a simple and easy-to-use guide and a ROS2 package to control the Universal Robot robots using the Universal Robot ROS2 driver.

# 📦 Installation
**Note: currenly this repo only support ROS2 humble since official driver files are a bit different in jazzy**
1. Install ROS2 dependencies:
   ```bash
   sudo apt install ros-humble-ur-* ros-humble-ros2-control ros-humble-ros2-controllers
   ```
2. Define your workspace location and create ros2 workspace:
   ```bash
   export ROS2_WS=~/ros2_ws # or any other location you prefer
   mkdir -p ${ROS2_WS}/src
   cd ${ROS2_WS}/src
   ```
3. Clone the repository:
    ```bash
    git clone https://github.com/idra-lab/easy_ur_control.git
    ```
4. Connect to the robot via ethernet, make sure the robot is powered on and connected to the same network as your computer. You can check the robot's IP address on the teach pendant (tablet) pressing `Top left options button`->`Settings`->`System`->`Network` and read `IP Address` and then usually just set yours as the same +1 in the last digit. Also annoate the robot ip address, you will need it later.
   *For example, if the robot's IP is `192.168.1.100`, you can set your computer's IP to `192.168.1.101` with the same subnet mask `255.255.255.0`*
5. Calibrate the robot and save the calibration data in the config folder of this package, set the robot IP address you noted before:
   ```bash
   ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=<robot_ip> target_filename:="${ROS2_WS}/src/easy_ur_control/config/calibration.yaml"
   ```
6. Build the workspace:
   ```bash
   cd ${ROS2_WS}
   colcon build --symlink-install
   ```
7. Remember to source the workspace after building:
   ```bash
   source ${ROS2_WS}/install/setup.bash
   ```
## 🕹 Cartesian Control
If you want to control the robot end effector in Cartesian space, you need to perform inverse kinematics to compute the joint angles from the desired end effector pose.  
I suggest you to use the [Cartesian Controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers) repository for this purpose.  
You can install the controllers with:
```
mkdir -p ~/controller_ws/src
cd ~/controller_ws/src 
git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
cd ..
colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Remember to source the workspace after building:
```bash
source ~/controller_ws/install/setup.bash
```

## 🚀 Start controlling the robot
1. Put the robot in `Remote Control` mode from the teach pendant (tablet) pressing `Top left options button`->`Locale`->`Controllo remoto`
2. Run the prepared launch file
```
ros2 launch easy_ur_control easy_ur_launcher.launch.py robot_ip:=<robot_ip> ur_type:=<ur_type> # ur3e, ur5e, ur10e, ur16e
```

## Move the robot via Rviz
You can move the robot using the Rviz interface, by mean of the `motion_control_handle` controller.
1. Launch the motion control handle with the `rqt_controller_gui`
```
ros2 run rqt_controller_manager rqt_controller_manager
```
Installable with:
```
sudo apt install ros-humble-rqt-controller-manager
```
2. Select the `motion_control_handle` controller and click on `Start`

TODO...