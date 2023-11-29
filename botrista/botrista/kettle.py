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
from franka_msgs.action import Grasp
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup


class Kettle(Node):

    def __init__(self):
        super().__init__("kettle")
        pass

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.moveit_api = MoveItApi(
            self, "panda_link0", "panda_hand_tcp", "panda_manipulator", "/franka/joint_states")
        self.grasp_planner = GraspPlanner(
            self.moveit_api, "panda_gripper/grasp")

        self.grab_srv = self.create_service(
            Empty, "grab", self.grab, callback_group=ReentrantCallbackGroup())

        self.release_srv = self.create_service(
            Empty, "place", self.place, callback_group=ReentrantCallbackGroup())

        # measured poses
        self.approach_pose = Pose(
            position=Point(x=0.006, y=0.05, z=0.20),
            orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        )
        self.grasp_pose = Pose(
            position=Point(x=0.097, y=0.043, z=0.15),
            orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        )
        self.retreat_pose = Pose(
            position=Point(x=0.097, y=0.043, z=0.25),
            orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        )

    async def grab(self, request, response):
        """
        Grabs the kettle from its stand.
        """
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_kettle_tag", Time())

        approach_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, tf)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(self.grasp_pose, tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            self.retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.02,
                force=50.0,
                speed=0.05,
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        return response

    async def place(self, request, response):
        """
        Places the kettle on its stand.
        """
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_kettle_tag", Time())

        # play the grasp plan backwards to place the kettle
        approach_pose = tf2_geometry_msgs.do_transform_pose(
            self.retreat_pose, tf)

        grasp_pose = tf2_geometry_msgs.do_transform_pose(self.grasp_pose, tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.04,  # open the gripper wider to release the kettle
                force=50.0,
                speed=0.05,
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        return response


def kettle_entry(args=None):
    rclpy.init(args=args)
    kettle = Kettle()
    rclpy.spin(kettle)
    rclpy.shutdown()
