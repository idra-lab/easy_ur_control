# Easy Universal Robot Control
This repository provides a simple and easy-to-use guide and a ROS2 package to control the Universal Robot robots using the Universal Robot ROS2 driver.  

**Note: currenly this repo only support ROS2 humble since official driver files are a bit different in jazzy**  
# 📦 Installation
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
Finally you need to modify the `ur_robot_driver` packages to include topics remapping so that your controller knows the right topic for the force/torque readings and the input pose topic. You can do this by editing the `/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py ` file and adding the following lines to the `ur_control_node` definition:
```python
    # Remap for force/torque sensor
    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        ## ADD THIS PART
        remappings=[
            ("/cartesian_motion_controller/target_frame", "/target_frame"),
            ("/cartesian_compliance_controller/target_frame", "/target_frame"),
            ("/cartesian_compliance_controller/ft_sensor_wrench","/force_torque_sensor_broadcaster/wrench"),
            ("/cartesian_compliance_controller/target_wrench", "/target_wrench"),
            ("/cartesian_compliance_controller/target_wrench", "/target_wrench"),
            ("/cartesian_force_controller/target_frame", "/target_frame"),
            ("/motion_control_handle/target_frame", "/target_frame"),
        ],
        ##
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )
```

## 🚀 Start controlling the robot
1. Put the robot in `Remote Control` mode from the teach pendant (tablet) pressing `Top left options button`->`Local`->`Remote Control`
2. Run the prepared launch file:
Different control techniques are supported:
   - [Cartesian control](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2/cartesian_motion_controller): `ctrl:=cartesian_motion_controller`
   - [Simulated cartesian impedance control](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2/cartesian_compliance_controller): `ctrl:=cartesian_compliance_controller`
   - [Joint trajectory control](https://wiki.ros.org/scaled_joint_trajectory_controller): `ctrl:=scaled_joint_trajectory_controller`
```
ros2 launch easy_ur_control easy_ur_launcher.launch.py robot_ip:=<robot_ip> ur_type:=<ur_type> ctrl:=<control-method> # ur3e, ur5e, ur10e, ur16e
```
3. Publish your command on topic:
   - Cartesian control receives commands on `/cartesian_motion_controller/target_frame`
   - Simulated cartesian impedance control receives commands on `/cartesian_compliance_controller/target_frame`
   - Joint position control receives commands on `/scaled_joint_trajectory_controller/joint_trajectory`

### Using fake hardware

For a simple simulation of your executable, you can easily use a fake hardware configuration which doesn't connect to the robot. Simply call
```
ros2 launch easy_ur_control easy_ur_launcher.launch.py use_fake_hardware:=true ur_type:=<ur_type>  # and additional arguments
```

**Note:** if you want to correctly use the cartesian control with the fake hardware, you must set the corresponding remappings in the `control_node` definition in `/opt/ros/humble/share/ur_robot_driver/launch/ur_control.launch.py`;

```python
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        # Import these mappings:
        remappings=[
            ("/cartesian_motion_controller/target_frame", "/target_frame"),
            ("/cartesian_compliance_controller/target_frame", "/target_frame"),
            ("/cartesian_compliance_controller/ft_sensor_wrench","/force_torque_sensor_broadcaster/wrench"),
            ("/cartesian_compliance_controller/target_wrench", "/target_wrench"),
            ("/cartesian_compliance_controller/target_wrench", "/target_wrench"),
            ("/cartesian_force_controller/target_frame", "/target_frame"),
            ("/motion_control_handle/target_frame", "/target_frame"),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )
```
