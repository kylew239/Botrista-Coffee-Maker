from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution


# this is a launch file launches both the d435i and the d405, the april
# tag node, and the camera_localizer node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ]),
            launch_arguments={
                'camera_name': 'd435i',
                'device_type': 'd435i',
                'pointcloud.enable': 'true',
            }.items(),
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
    ])
