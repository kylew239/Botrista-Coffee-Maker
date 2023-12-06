from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    EqualsSubstitution
)
from launch.conditions import IfCondition


"""
Description:
    This launch file launches both the d435i and the d405, the april tag node, and the camera_localizer node
"""

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_type',
            default_value='real',
            description='whether to use fake or real hardware'
        ),
        GroupAction(
            actions=[
                SetRemap(src='/camera/d435i/color/image_raw',
                         dst='/image_raw'),
                SetRemap(src='/camera/d435i/color/camera_info',
                         dst='/camera_info'),
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
                        'depth_module.profile': '1280x720x6',
                        'enable_depth': 'true',
                        'align_depth.enable': 'true',
                        'depth_module.enable_auto_exposure': 'false',
                        'rgb_camera.enable_auto_exposure': 'false',
                        'pointcloud.enable': 'true',
                        'spatial_filter.enable': 'true',
                        'temporal_filter.enable': 'true',
                        'decimation_filter.enable': 'true',
                        'depth_module.exposure': '3000',
                        'json_file_path': PathJoinSubstitution([
                            FindPackageShare("botrista"),
                            "config",
                            "d435i_config.json"
                        ]),
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
                'enable_depth': 'true',
                'align_depth.enable': 'true',
                'spatial_filter.enable': 'true',
                'temporal_filter.enable': 'true',
                'decimation_filter.enable': 'true',
                'depth_module.enable_auto_exposure': 'true',
                # 'depth_module.exposure': '30000'
                # 'json_file_path': PathJoinSubstitution([
                #     FindPackageShare("botrista"),
                #     "config",
                #     "d405_config.json"
                # ]),
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
