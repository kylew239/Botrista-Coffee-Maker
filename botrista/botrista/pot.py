import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from franka_msgs.action import Grasp
from rclpy.time import Time
from time import sleep
from franka_msgs.msg import GraspEpsilon
from botrista_interfaces.action import EmptyAction, GraspProcess
from botrista_interfaces.srv import DelayTime
from rclpy.action import ActionServer, ActionClient
from std_msgs.msg import Header
import numpy as np


class Pot(Node):

    def __init__(self):
        super().__init__("pot")

        self.pot_actual_place = TransformStamped()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.moveit_api = MoveItApi(
            self, "panda_link0", "panda_hand_tcp", "panda_manipulator", "/franka/joint_states")
        self.grasp_planner = GraspPlanner(
            self.moveit_api, "panda_gripper/grasp")

        self.delay_client = self.create_client(
            DelayTime, "delay", callback_group=ReentrantCallbackGroup()
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
        self.pour_pot_server = ActionServer(self,
                                            EmptyAction,
                                            "pour_pot",
                                            self.pour_pot_cb,
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

        self.top_tf = TransformStamped(
            header=Header(
                frame_id="filtered_pour_over_tag",
                stamp=self.get_clock().now().to_msg()
            ),
            child_frame_id="pot_top",
            transform=Transform(
                translation=Vector3(x=0.155, y=0.035, z=0.165),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
        )

        self.static_broadcaster.sendTransform(self.top_tf)

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

    async def pour_pot_cb(self, goal_handle):
        try:
            cup_tf = self.buffer.lookup_transform(
                "panda_link0", "cup_location", Time())
        except Exception as e:
            self.get_logger().error("No transform found")
            return

        spout_tf = TransformStamped(
            header=Header(
                frame_id="panda_link0",
                stamp=self.get_clock().now().to_msg()
            ),
            transform=Transform(
                translation=Vector3(x=0.0, y=-0.25, z=0.02),
                rotation=Quaternion()
            )
        )

        cup_tf_approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.30),
            orientation=Quaternion(x=0.7071068, y=0.7071068, z=0.0, w=0.0)
        )

        cup_tf_pour_pose = Pose(
            position=Point(x=0.0, y=0.22, z=0.2),
            orientation=Quaternion(
                x=-0.5, y=-0.5, z=0.5, w=-0.50)
        )

        cup_tf_pour_pose_2 = Pose(
            position=Point(x=0.0, y=0.31, z=0.25),
            orientation=Quaternion(
                x=0.4055798, y=0.4055798, z=-0.579228, w=0.579228)
        )

        retreat_pose = Pose(
            position=Point(x=0.0, y=-0.1, z=0.5),
            orientation=Quaternion(
                x=-0.5, y=-0.5, z=0.5, w=-0.50)
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            retreat_pose, spout_tf
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            retreat_pose, cup_tf
        )

        # Transform to spout
        spout_approach_pose = tf2_geometry_msgs.do_transform_pose(
            cup_tf_approach_pose, spout_tf
        )
        spout_pour_pose = tf2_geometry_msgs.do_transform_pose(
            cup_tf_pour_pose, spout_tf
        )
        spout_pour_pose_2 = tf2_geometry_msgs.do_transform_pose(
            cup_tf_pour_pose_2, spout_tf
        )

        # Transform to cup
        approach_pose = tf2_geometry_msgs.do_transform_pose(
            spout_approach_pose, cup_tf
        )
        pour_pose = tf2_geometry_msgs.do_transform_pose(
            spout_pour_pose, cup_tf
        )
        pour_pose_2 = tf2_geometry_msgs.do_transform_pose(
            spout_pour_pose_2, cup_tf
        )

        # rotate handle
        joint_states = self.moveit_api.get_current_joint_state()
        joint_states["panda_joint7"] += np.pi
        joint_states["panda_joint7"] = min(
            joint_states["panda_joint7"], 150.0 * np.pi / 180.0)

        names = list(joint_states.keys())
        pos = list(joint_states.values())
        await self.moveit_api.plan_joint_async(
            names, pos, execute=True)

        # Approach
        result = await self.moveit_api.plan_async(
            point=approach_pose.position,
            orientation=approach_pose.orientation,
            execute=True,
        )

        # Pouring pose 1
        self.get_logger().info(
            f"Pouring: {pour_pose.position} {pour_pose.orientation}")
        result = await self.moveit_api.plan_async(
            point=pour_pose.position,
            orientation=pour_pose.orientation,
            execute=True
        )

        # Pouring pose 2
        self.get_logger().info(
            f"Pouring: {pour_pose_2.position} {pour_pose_2.orientation}")
        result = await self.moveit_api.plan_async(
            point=pour_pose_2.position,
            orientation=pour_pose_2.orientation,
            execute=True
        )

        await self.delay_client.call_async(DelayTime.Request(time=5.0))

        # Returning Pouring pose 1
        self.get_logger().info(
            f"Pouring: {pour_pose.position} {pour_pose.orientation}")
        result = await self.moveit_api.plan_async(
            point=pour_pose.position,
            orientation=pour_pose.orientation,
            execute=True
        )

        # Retreat Pose
        result = await self.moveit_api.plan_async(
            point=retreat_pose.position,
            orientation=retreat_pose.orientation,
            execute=True
        )

        goal_handle.succeed()
        return EmptyAction.Result()


def pot_entry(args=None):
    rclpy.init(args=args)
    pot = Pot()
    rclpy.spin(pot)
    rclpy.shutdown()
