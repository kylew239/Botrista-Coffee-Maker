from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("botrista"),
                "launch",
                "realsense.launch.py"
            ])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("botrista"),
                "launch",
                "open_franka.launch.xml"
            ]),
            launch_arguments={
                'hardware_type': 'real'
            }.items(),
        ),
        Node(
            package="botrista",
            executable="pouring"
        )
    ])
