from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


<<<<<<< HEAD
=======
# this is a launch file launches both the d435i and the d405, the april
# tag node, and the camera_localizer node

>>>>>>> refs/remotes/origin/anuj/cup
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="botrista",
            executable="cup_detection",
            ros_arguments=[
                "--params-file",
                PathJoinSubstitution([
                    FindPackageShare("botrista"),
                    "config",
                    "cam_cal.yaml"
                ])
            ],
        )
    ])