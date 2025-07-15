from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


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
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "launch",
                    "ur_control.launch.py",
                ]
            )
        ),
        # override the default launch arguments to point to the easy_ur_control package
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "description_package": "easy_ur_control",
            "description_file": "ur_wrapper.xacro",
            "kinematics_params_file": PathJoinSubstitution(
                [FindPackageShare("easy_ur_control"), "config", "calibration.yaml"]
            ),
            "headless_mode": "true",
            "rviz_config_file": "easy_ur_control",
            "runtime_config_package": "easy_ur_control",
            "rviz_config_file": "config/ur_rviz_config.rviz",
            # disable joint controller activation so we can activate our custom controllers from this launch file
            "activate_joint_controller": "false",
        }.items(),
    )
    controller = LaunchConfiguration("ctrl").perform(context)
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "-c", "/controller_manager"],
        remappings=[
            ("/cartesian_motion_controller/target_frame", "/target_frame"),
        ],
    )

    return [
        base_launch,
        controller_spawner,
    ]


def generate_launch_description():
    return LaunchDescription(declare_args() + [OpaqueFunction(function=launch_setup)])
