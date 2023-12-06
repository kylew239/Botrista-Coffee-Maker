"""Performs the grasping action of the standard handles.

Action Servers:
  grasp_process (botrista_interfaces/action/GraspProcess) - grasp action

Client:
  delay (botrista_interfaces/DelayTime) - timer for delay in seconds.

"""

import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from franka_msgs.action import Grasp
from rclpy.time import Time
from botrista_interfaces.action import GraspProcess
from botrista_interfaces.srv import DelayTime
from rclpy.action import ActionServer
from moveit_wrapper.moveitapi import MoveItApi
from rclpy.callback_groups import ReentrantCallbackGroup


class GraspNode(Node):
    def __init__(self):
        super().__init__("GraspNode")

        self.grasp_process_action_server = ActionServer(
            self,
            GraspProcess,
            "grasp_process",
            self.grasp_process,
            callback_group=ReentrantCallbackGroup(),
        )

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.moveit_api = MoveItApi(
            self,
            "panda_link0",
            "panda_hand_tcp",
            "panda_manipulator",
            "/franka/joint_states",
        )

        self.delay_client = self.create_client(
            DelayTime, "delay", callback_group=ReentrantCallbackGroup()
        )
        self.grasp_planner = GraspPlanner(self.moveit_api, "panda_gripper/grasp")

        self.payloads = {
            0: None,  # None
            1: (1.8, [0.05, 0.0, 0.10]),  # Kettle
        }

    async def grasp_process(self, goal_handle):
        """Grabs a specified object

        Args:
            goal_handle (Empty_Goal): goal of the action

        Returns:
            Empty_result: result of the action
        """
        self.get_logger().warn("GRASP PROCESS")

        observe_pose = goal_handle.request.observe_pose
        refinement_offset = goal_handle.request.refinement_pose
        approach_offset = goal_handle.request.approach_pose
        grasp_offset = goal_handle.request.grasp_pose
        retreat_offset = goal_handle.request.retreat_pose
        gripper_command = Grasp.Goal(
            width=goal_handle.request.width,
            force=goal_handle.request.force,
            speed=goal_handle.request.speed,
            epsilon=goal_handle.request.epsilon,
        )
        # move to observe point
        await self.moveit_api.plan_async(
            point=observe_pose.position,
            orientation=observe_pose.orientation,
            execute=True,
        )
        await self.delay_client.call_async(DelayTime.Request(time=3.0))

        # get the handle tf
        handle_tf = self.buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0)
        )
        self.get_logger().warn(f"HANDLE TF: {handle_tf}")

        # move to refinement point
        refinement_point = tf2_geometry_msgs.do_transform_pose(
            refinement_offset, handle_tf
        )

        self.get_logger().warn(f"REFINEMNENT POINT: {refinement_point}")

        await self.moveit_api.plan_async(
            point=refinement_point.position,
            orientation=refinement_point.orientation,
            execute=True,
        )
        await self.delay_client.call_async(DelayTime.Request(time=3.0))
        handle_tf = self.buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0)
        )

        # put the grasp points in into panda frame
        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_offset, handle_tf)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_offset, handle_tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_offset, handle_tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            retreat_pose=retreat_pose,
            grasp_command=gripper_command,
        )

        if (payload := self.payloads[goal_handle.request.object]) is not None:
            grasp_plan.mass = payload[0]
            grasp_plan.com = payload[1]

        if goal_handle.request.reset_load:
            grasp_plan.reset_load = True

        actual_grasp_pose = await self.grasp_planner.execute_grasp_plan(grasp_plan)

        result = GraspProcess.Result()
        result.actual_grasp_pose = actual_grasp_pose
        self.get_logger().warn(f"finished grasp: {result}")
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    gn = GraspNode()
    rclpy.spin(gn)
    rclpy.shutdown()
