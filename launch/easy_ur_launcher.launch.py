import os.path
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_path


def declare_args():
    # Set fixed values for launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
            default_value="192.168.100.14",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctrl",
            description="Name of the controller to be activated.",
            default_value="cartesian_motion_controller",
            choices=[
                "cartesian_motion_controller",
                "cartesian_compliance_controller",
                "joint_trajectory_controller",
                "cartesian_force_controller",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            description="Whether to start or not RViz2",
            default_value="true",
        )
    )
    this_package_share = get_package_share_path("easy_ur_control")
    default_rviz_path = os.path.join(this_package_share, "rviz", "rviz.rviz")
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            description="Full path of the RViz2 config",
            default_value=default_rviz_path,
        )
    )
    return declared_arguments


def launch_setup(context, *args, **kwargs):
    # Include the UR driver launch file with fixed arguments
    print_green = "\033[92m"
    print_bold = "\033[1m"
    print_reset = "\033[0m"
    print(
        print_green + print_bold + "Launching easy_ur_control with UR type:",
        context.launch_configurations["ur_type"],
        ", robot IP:",
        context.launch_configurations["robot_ip"],
        ", controller:",
        context.launch_configurations["ctrl"],
        print_reset,
    )
    base_launch_arguments={
        "ur_type": LaunchConfiguration("ur_type"),
        "robot_ip": LaunchConfiguration("robot_ip"),
        "description_package": "easy_ur_control",
        "description_file": "ur_wrapper.xacro",
        "headless_mode": "true",
        "runtime_config_package": "easy_ur_control",
        "launch_rviz": "false",
        # disable joint controller activation so we can activate our custom controllers from this launch file
        "activate_joint_controller": "false",
        # "initial_joint_controller": "joint_trajectory_controller",
    }

    this_package_share = get_package_share_path("easy_ur_control")
    calibration_file = os.path.join(this_package_share, "config", "calibration.yaml")
    if os.path.exists(calibration_file):
        print("Using calibration data from", calibration_file)
        base_launch_arguments["kinematics_params_file"] = calibration_file

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("easy_ur_control"),
                    "launch",
                    "ur_control.launch.py",
                ]
            )
        ),
        # override the default launch arguments to point to the easy_ur_control package
        launch_arguments=base_launch_arguments.items(),
    )
    rviz_spawner = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    controller = LaunchConfiguration("ctrl").perform(context)
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "-c", "/controller_manager"],
    )

    return [
        base_launch,
        controller_spawner,
        rviz_spawner,
    ]


def generate_launch_description():
    return LaunchDescription(declare_args() + [OpaqueFunction(function=launch_setup)])
