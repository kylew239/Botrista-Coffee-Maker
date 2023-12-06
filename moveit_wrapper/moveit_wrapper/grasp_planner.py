from geometry_msgs.msg import Pose
from moveit_wrapper.moveitapi import MoveItApi
from franka_msgs.action import Grasp
from rclpy.action import ActionClient
from typing import List
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
from controller_manager_msgs.srv import SwitchController
from franka_msgs.srv import SetLoad


class GraspPlan:
    def __init__(
        self,
        approach_pose: Pose,
        grasp_pose: Pose,
        grasp_command: Grasp.Goal,
        retreat_pose: Pose,
        reset_load: bool = False,
        mass: float = None,
        com: List[float] = None,
    ):
        self.approach_pose = approach_pose
        self.grasp_pose = grasp_pose
        self.grasp_command = grasp_command
        self.retreat_pose = retreat_pose
        self.mass = mass
        self.com = com
        self.reset_load = reset_load


class GraspPlanner:
    """
    Plans and executes a grasp action
    """

    def __init__(self, moveit_api: MoveItApi, gripper_action_name: str):
        self.moveit_api = moveit_api
        self.node = self.moveit_api.node

        self.grasp_client = ActionClient(
            self.node,
            Grasp,
            gripper_action_name,
            callback_group=ReentrantCallbackGroup(),
        )

        # Create tf listener
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self.node)
        self.tf_parent_frame = "panda_link0"

        self.controller_client = self.node.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
            callback_group=ReentrantCallbackGroup(),
        )

        self.load_client = self.node.create_client(
            SetLoad, "/service_server/set_load", callback_group=ReentrantCallbackGroup()
        )

    async def execute_grasp_plan(self, grasp_plan: GraspPlan):
        self.node.get_logger().warn("going to approach point!")
        curr_poseStamped = await self.moveit_api.get_end_effector_pose()
        curr_pose = curr_poseStamped.pose

        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.approach_pose.position,
            orientation=grasp_plan.approach_pose.orientation,
            execute=True,
        )

        self.node.get_logger().warn("going to grasp point!")
        self.node.get_logger().warn(f"grasp pose: {grasp_plan.grasp_pose.orientation}")
        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.grasp_pose.position,
            orientation=grasp_plan.grasp_pose.orientation,
            execute=True,
        )

        if grasp_plan.reset_load:
            await self.set_load(grasp_plan)

        self.node.get_logger().warn("grasping...")
        goal_handle = await self.grasp_client.send_goal_async(grasp_plan.grasp_command)
        grasp_command_result = await goal_handle.get_result_async()
        self.node.get_logger().warn("finished grasp")

        if not grasp_plan.reset_load:
            await self.set_load(grasp_plan)

        actual_grasp_position = self.buffer.lookup_transform(
            "panda_link0", "panda_hand_tcp", Time()
        )

        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.retreat_pose.position,
            orientation=grasp_plan.retreat_pose.orientation,
            execute=True,
        )

        return actual_grasp_position

    async def set_load(self, plan: GraspPlan):
        if (plan.mass is None or plan.com is None) and not plan.reset_load:
            return

        if plan.reset_load:
            mass = 0.0
            com = [0.0, 0.0, 0.0]
        else:
            mass = plan.mass
            com = plan.com

        self.node.get_logger().warn(f"setting load to {mass}, {com}")

        # turn off controller
        switch_req = SwitchController.Request(
            deactivate_controllers=["panda_arm_controller"],
        )
        res = await self.controller_client.call_async(switch_req)
        self.node.get_logger().warn(f"switch controller result: {res}")
        # set payload
        set_load_req = SetLoad.Request(
            mass=mass,
            center_of_mass=com,
        )
        res = await self.load_client.call_async(set_load_req)
        self.node.get_logger().warn(f"result: {res}")

        # turn on controller
        switch_req = SwitchController.Request(
            activate_controllers=["panda_arm_controller"],
        )
        res = await self.controller_client.call_async(switch_req)
        self.node.get_logger().warn(f"result: {res}")
