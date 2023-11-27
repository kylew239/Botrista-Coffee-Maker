from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    EqualsSubstitution
)
from launch.conditions import IfCondition


# this is a launch file launches both the d435i and the d405, the april
# tag node, and the camera_localizer node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_type',
            default_value='real',
            description='whether to use fake or real hardware'
        ),
        GroupAction(
            actions=[
                SetRemap(src='/d435i/color/image_raw',
                         dst='/image_raw'),
                SetRemap(src='/d435i/color/camera_info', dst='/camera_info'),
                IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare("realsense2_camera"),
                        "launch",
                        "rs_launch.py"
                    ]),
                    launch_arguments={
                        'camera_name': 'd435i',
                        'device_type': 'd435i',
                        'rgb_camera.profile': '1920x1080x6',
                        'rgb_camera.enable_auto_exposure': 'false',
                        'enable_depth': 'false',
                    }.items(),
                ),
            ],
            condition=IfCondition(
                EqualsSubstitution(
                    left=LaunchConfiguration('hardware_type'),
                    right='real'
                )
            )
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ]),
            launch_arguments={
                'camera_name': 'd405',
                'device_type': 'd405',
                'pointcloud.enable': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("image_proc"),
                "launch",
                "image_proc.launch.py"
            ])
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            ros_arguments=[
                "--params-file",
                PathJoinSubstitution([
                    FindPackageShare("botrista"),
                    "config",
                    "tag.yaml"
                ])
            ]
        ),
        Node(
            package="botrista",
            executable="camera_localizer",
        )
    ])
