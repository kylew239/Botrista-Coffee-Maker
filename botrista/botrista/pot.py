import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from franka_msgs.action import Grasp
from rclpy.time import Time
from time import sleep
from franka_msgs.msg import GraspEpsilon
from botrista_interfaces.action import EmptyAction, GraspProcess
from rclpy.action import ActionServer, ActionClient


class Pot(Node):

    def __init__(self):
        super().__init__("pot")

        self.pot_actual_place = TransformStamped()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.moveit_api = MoveItApi(
            self, "panda_link0", "panda_hand_tcp", "panda_manipulator", "/franka/joint_states")
        self.grasp_planner = GraspPlanner(
            self.moveit_api, "panda_gripper/grasp")

        self.delay_client = self.create_client(
            Empty, "delay", callback_group=ReentrantCallbackGroup()
        )
        self.pick_pot_client = ActionServer(self,
                                            EmptyAction,
                                            "pick_pot",
                                            self.pick_pot_cb,
                                            callback_group=ReentrantCallbackGroup())
        self.place_pot_client = ActionServer(self,
                                             EmptyAction,
                                             "place_pot",
                                             self.place_pot_cb,
                                             callback_group=ReentrantCallbackGroup())
        self.grasp_process = ActionClient(self,
                                          GraspProcess,
                                          'grasp_process',
                                          callback_group=ReentrantCallbackGroup())

        while not self.delay_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for delay service")

        self.observe_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.40),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        # grasp points in handle frame
        self.refine_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.15),
            orientation=Quaternion())
        self.approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.10),
            orientation=Quaternion())
        self.grasp_pose = Pose(
            position=Point(x=0.02, y=0.0, z=0.0),
            orientation=Quaternion())
        self.retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.10),
            orientation=Quaternion())

    async def pick_pot_cb(self, goal_handle):
        """
        Grabs the pot from its stand.
        """
        # TFs
        # pour_over_tag
        try:
            tf = self.buffer.lookup_transform(
                "panda_link0", "filtered_pour_over_tag", Time())

        except Exception as e:
            self.get_logger().error("No transform found")
            return

        observe_pose = tf2_geometry_msgs.do_transform_pose(
            self.observe_pose, tf)

        goal_msg = GraspProcess.Goal(
            observe_pose=observe_pose,
            refinement_pose=self.refine_pose,
            approach_pose=self.approach_pose,
            grasp_pose=self.grasp_pose,
            width=0.03,
            force=50.0,
            speed=0.05,
            epsilon=GraspEpsilon(inner=0.01, outer=0.01),
            retreat_pose=self.retreat_pose
        )

        goal = await self.grasp_process.send_goal_async(goal_msg)
        res = await goal.get_result_async()
        self.pot_actual_place = res.result.actual_grasp_pose

        goal_handle.succeed()
        return EmptyAction.Result()

    async def place_pot_cb(self, goal_handle):
        """
        Places the pot on its stand.
        """

        approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.1),
            orientation=Quaternion()
        )

        grasp_pose = Pose(
            position=Point(x=0.02, y=0.0, z=-0.02),
            orientation=Quaternion()
        )

        retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.1),
            orientation=Quaternion()
        )

        approach_pose = tf2_geometry_msgs.do_transform_pose(
            approach_pose, self.pot_actual_place)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            grasp_pose, self.pot_actual_place)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            retreat_pose, self.pot_actual_place)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.04,  # open the gripper wider to release the pot
                force=50.0,
                speed=0.2,
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        goal_handle.succeed()
        return EmptyAction.Result()


def pot_entry(args=None):
    rclpy.init(args=args)
    pot = Pot()
    rclpy.spin(pot)
    rclpy.shutdown()
