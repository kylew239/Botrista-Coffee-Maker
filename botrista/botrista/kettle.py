import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from geometry_msgs.msg import Pose, Point, Quaternion
from control_msgs.msg import GripperCommand
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup


class Kettle(Node):

    def __init__(self):
        super().__init__("kettle")

        self.moveit_api = MoveItApi(self,
                                    "panda_link0",
                                    "panda_hand_tcp",
                                    "panda_manipulator",
                                    "/franka/joint_states")

        self.grasp_planner = GraspPlanner(self.moveit_api,
                                          "panda_gripper/gripper_action")
        orientation = Quaternion(
            x=1.0,
            y=0.0,
            z=0.0,
            w=0.0
        )
        self.grasp_plan = GraspPlan(
            Pose(
                position=Point(
                    x=0.4,
                    y=0.0,
                    z=0.5
                ),
                orientation=orientation,
            ),
            Pose(
                position=Point(
                    x=0.5,
                    y=0.0,
                    z=0.5
                ),
                orienation=orientation
            ),
            GripperCommand(
                position=0.20,
                max_effort=20.0
            ),
            Pose(
                position=Point(
                    x=0.5,
                    y=0.0,
                    z=0.4
                ),
                orientation=orientation
            )
        )

        self.cb_group = ReentrantCallbackGroup()

        self.create_service(Empty, "test_grasp", self.grasp,
                            callback_group=self.cb_group)

    async def grasp(self, request, response):
        await self.grasp_planner.execute_grasp_plan(self.grasp_plan)
        return response


def kettle_entry(args=None):
    rclpy.init(args=args)
    kettle = Kettle()
    rclpy.spin(kettle)
    rclpy.shutdown()
