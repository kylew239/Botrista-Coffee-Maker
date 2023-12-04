"""
Node which include actions to pick up filter and place filter

Uses the wrapper class made in moveitapi. Moveit moves the robot to various points
and controls the gripper as well.

SERVICES:
  + pick_filter (botrista_interfaces/action/Emptyaction) - pick up filter
  + place_filter (botrista_interfaces/action/Emptyaction) - place the filter back to the filter stand
  + place_filter_in_pot (botrista_interfaces/action/Emptyaction) - place the filter on pot
  + pick_filter_in_pot (botrista_interfaces/action/Emptyaction) - pick up the filter from the pot

Client
------
  + delay (botrista_interfaces/DelayTime) - timer for delay in seconds.
  + grasp_process (botrista_interfaces/action/GraspProcess) - timer for delay in seconds.

"""

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from franka_msgs.msg import GraspEpsilon
from geometry_msgs.msg import Point, Quaternion, Pose, TransformStamped
from rclpy.action import ActionServer, ActionClient
from botrista_interfaces.action import EmptyAction, GraspProcess
from botrista_interfaces.srv import DelayTime
from std_srvs.srv import Empty
import tf2_geometry_msgs
from franka_msgs.action import Grasp


class Pick_filter(Node):
    def __init__(self):
        """
        pick up and place filter
        """
        super().__init__("pick_filter")
        self.cb = ReentrantCallbackGroup()

        # Creating tf listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.delay_client = self.create_client(
            DelayTime, "delay", callback_group=ReentrantCallbackGroup()
        )

        base_frame = "panda_link0"
        end_effector_frame = "panda_hand_tcp"
        group_name = "panda_manipulator"
        self.moveit_api = MoveItApi(
            self,
            base_frame=base_frame,
            end_effector_frame=end_effector_frame,
            group_name=group_name,
            joint_state_topic="joint_states",
        )
        self.grasp_planner = GraspPlanner(
            self.moveit_api, "panda_gripper/grasp")

        # Creating action server
        self.pick_filter_client = ActionServer(
            self,
            EmptyAction,
            "pick_filter",
            self.pick_filter_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self.place_fliter_client = ActionServer(
            self,
            EmptyAction,
            "place_filter",
            self.place_filter_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self.place_fliter_pot_client = ActionServer(
            self,
            EmptyAction,
            "place_filter_in_pot",
            self.place_filter_in_pot,
            callback_group=ReentrantCallbackGroup(),
        )

        self.pick_fliter_pot_client = ActionServer(
            self,
            EmptyAction,
            "pick_filter_in_pot",
            self.pickup_filter_from_pot,
            callback_group=ReentrantCallbackGroup(),
        )

        self.grasp_process = ActionClient(
            self, GraspProcess, "grasp_process", callback_group=ReentrantCallbackGroup()
        )

        while not self.delay_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for delay service")

        self.observe_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.25),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        )
        # grasp points in handle frame
        self.approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.10), orientation=Quaternion()
        )
        self.grasp_pose = Pose(
            position=Point(x=0.03, y=0.0, z=-0.01), orientation=Quaternion()
        )
        self.retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.20), orientation=Quaternion()
        )

        self.filter_pot_tf = TransformStamped()

    async def pick_filter_cb(self, goal_handle):
        """pick up filter from filter stand

        Args:
            goal_handle (Empty_Goal): goal of the action

        Returns:
            Empty_result: result of the action
        """
        try:
            tf = self.buffer.lookup_transform(
                "panda_link0", "filtered_filter_tag", Time()
            )

        except Exception as e:
            self.get_logger().error("No transform found")
            return

        observe_pose = tf2_geometry_msgs.do_transform_pose(
            self.observe_pose, tf)

        refinement_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.15), orientation=Quaternion()
        )

        goal_msg = GraspProcess.Goal(
            observe_pose=observe_pose,
            refinement_pose=refinement_pose,
            approach_pose=self.approach_pose,
            grasp_pose=self.grasp_pose,
            width=0.03,
            force=50.0,
            speed=0.05,
            epsilon=GraspEpsilon(inner=0.01, outer=0.01),
            retreat_pose=self.retreat_pose,
        )

        goal = await self.grasp_process.send_goal_async(goal_msg)
        res = await goal.get_result_async()
        self.filter_actual_place = res.result.actual_grasp_pose

        goal_handle.succeed()
        return EmptyAction.Result()

    async def place_filter_cb(self, goal_handle):
        """place filter back to filter stand

        Args:
            goal_handle (Empty_Goal): goal of the action

        Returns:
            Empty_result: result of the action
        """
        approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.20), orientation=Quaternion()
        )

        grasp_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.03), orientation=Quaternion()
        )

        retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.20), orientation=Quaternion()
        )

        approach_pose = tf2_geometry_msgs.do_transform_pose(
            approach_pose, self.filter_actual_place
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            grasp_pose, self.filter_actual_place
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            retreat_pose, self.filter_actual_place
        )

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.04,  # open the gripper wider to release the kettle
                force=50.0,
                speed=0.2,
            ),
            retreat_pose=approach_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        goal_handle.succeed()
        return EmptyAction.Result()

    async def place_filter_in_pot(self, goal_handle):
        """place filter on pot top

        Args:
            goal_handle (Empty_Goal): goal of the action

        Returns:
            Empty_result: result of the action
        """
        try:
            tf = self.buffer.lookup_transform("panda_link0", "pot_top", Time())
        except Exception as e:
            self.get_logger().error("No transform found")
            return

        approach_pose = Pose(
            position=Point(x=0.09, y=0.0, z=0.15),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
        )

        grasp_pose = Pose(
            position=Point(x=0.09, y=0.02, z=0.04),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
        )

        retreat_pose = Pose(
            position=Point(x=0.09, y=0.02, z=0.20),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
        )

        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, tf)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.04,  # open the gripper wider to release the kettle
                force=50.0,
                speed=0.2,
            ),
            retreat_pose=approach_pose,
        )

        self.filter_pot_tf = await self.grasp_planner.execute_grasp_plan(grasp_plan)

        goal_handle.succeed()
        return EmptyAction.Result()

    async def pickup_filter_from_pot(self, goal_handle):
        """pick up filter from pot top

        Args:
            goal_handle (Empty_Goal): goal of the action

        Returns:
            Empty_result: result of the action
        """
        approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.15), orientation=Quaternion()
        )

        grasp_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.005), orientation=Quaternion()
        )

        retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.10), orientation=Quaternion()
        )

        approach_pose = tf2_geometry_msgs.do_transform_pose(
            approach_pose, self.filter_pot_tf
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            grasp_pose, self.filter_pot_tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            retreat_pose, self.filter_pot_tf
        )

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.04,  # open the gripper wider to release the kettle
                force=50.0,
                speed=0.2,
                epsilon=GraspEpsilon(inner=0.01, outer=0.01),
            ),
            retreat_pose=approach_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        goal_handle.succeed()
        return EmptyAction.Result()


def main(args=None):
    rclpy.init(args=args)
    node = Pick_filter()
    rclpy.spin(node)
    rclpy.shutdown()
