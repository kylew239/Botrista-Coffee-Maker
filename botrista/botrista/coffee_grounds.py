"""
Manages the scoop action, which picks up the scoop, scoops coffee, move to the coffee pot, and dumps the coffee

ACTION CLIENTS:
  + grasp_process (GraspProcess) - A seriers of movements and gripper commands for the robot to complete a grasp action

ACTION SERVERS:
  + scoop (EmptyAction) - The scooping routine
"""

from rclpy.node import Node
import rclpy
import numpy as np
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan
import tf2_geometry_msgs
from franka_msgs.msg import GraspEpsilon
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)
from franka_msgs.action import Grasp
from rclpy.action import ActionServer, ActionClient
from tf2_ros import TransformListener, Buffer
from botrista_interfaces.action import EmptyAction
from botrista_interfaces.action import GraspProcess
from rclpy.time import Time
from moveit_wrapper.grasp_planner import GraspPlanner, GraspPlan


class CoffeeGrounds(Node):
    """
    Measures coffee depth, scoops coffee, and dumps coffee in coffee maker.
    Also dumps used coffee grounds from filter.
    """

    def __init__(self):
        """
        Description:
            Initializes the CoffeeGrounds node
        """
        super().__init__('coffee_grounds')

        # Define observation poses (relative to april tag)
        self.scoop_offset_pose_observe = Pose(
            position=Point(
                x=-0.1, y=0.02, z=0.4),
            orientation=Quaternion(
                x=.707388, y=-0.706825, z=0.0005629, w=0.0005633)
        )

        # Define refinement, approach, grasp, and retreat poses (relative to handle)
        self.scoop_offset_pose_refine = Pose(
            position=Point(
                x=0.0, y=0.0, z=-0.20),
            orientation=Quaternion()
        )
        self.scoop_offset_pose_approach = Pose(
            position=Point(
                x=-0.15, y=0.0, z=-0.10),
            orientation=Quaternion(
                x=0.5, y=0.5, z=0.5, w=0.5)
        )
        self.scoop_offset_pose_grasp = Pose(
            position=Point(
                x=-0.022, y=-0.01, z=0.0),
            orientation=Quaternion(
                x=0.5, y=0.5, z=0.5, w=0.5)
        )
        self.scoop_offset_pose_retreat = Pose(
            position=Point(
                x=0.1, y=0.0, z=-0.3),
            orientation=Quaternion(
                x=0.5, y=0.5, z=0.5, w=0.5)
        )

        # Define scooping poses (relative to coffee grounds april tag)
        self.scoop_pose_measure = Pose(
            position=Point(
                x=-0.15, y=0.04, z=0.2),
            orientation=Quaternion(
                x=1.0, y=0.0, z=0.0, w=1.0))
        self.scoop_pose_1 = Pose(
            position=Point(
                x=-0.20, y=0.0, z=0.2),
            orientation=Quaternion(
                x=0.7071068, y=0.7071068, z=0.0, w=0.0))
        self.scoop_pose_2 = Pose(
            position=Point(
                x=-0.15, y=0.04, z=0.2),
            orientation=Quaternion(
                x=1.0, y=0.0, z=0.0, w=1.0))
        self.scoop_pose_3 = Pose(
            position=Point(
                x=-0.15, y=0.02, z=0.2),
            orientation=Quaternion(
                x=-0.7071068, y=-0.0, z=0.0, w=0.7071068))

        # Define dumping poses (relative to pot top)
        self.dump_pose_approach = Pose(
            position=Point(
                x=0.0, y=0.0, z=0.2),
            orientation=Quaternion())

        # Initialize actual pickup positions
        self.scoop_actual_pickup_pose = Pose()

        # Define grasp commands
        self.grasp_command_scoop = Grasp.Goal(
            width=0.02, force=50.0, speed=0.05)
        self.grasp_command_open = Grasp.Goal(
            width=0.02, force=50.0, speed=0.05)

        # Start action server for scooping routine
        self.scoop_action_server = ActionServer(
            self,
            EmptyAction,
            'scoop',
            self.scoop_coffee_grounds)

        # Start action client for grasp process action
        self.grasp_process = ActionClient(self, GraspProcess, 'grasp_process')

        # Create tf listener
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.tf_parent_frame = "panda_link0"

        # Moveit Wrapper Object
        self.moveit_api = MoveItApi(self,
                                    "panda_link0",
                                    "panda_hand_tcp",
                                    "panda_manipulator",
                                    "joint_states",
                                    "panda")

        self.grasp_planner = GraspPlanner(
            self.moveit_api, "panda_gripper/grasp")

    async def scoop_coffee_grounds(self, goal_handle):
        """
        Description:
            Action callback for the scooping routine, scoops coffee and dumps it in the filter
        """
        await self.grab_scoop()
        # await self.scoop_grounds()
        await self.move_to_pot()

        # Go home
        await self.moveit_api.go_home()
        await self.return_scoop()

        goal_handle.succeed()
        return EmptyAction.Result()

    async def move_to_pot(self):
        """
        Moves the scoop to the pot to dump grounds.
        """

        # Create horizontal path constraint
        current_pose = await self.moveit_api.get_end_effector_pose()
        path_constraints = self.moveit_api.create_path_constraints(
            current_pose.pose.orientation,
            y_tol=2 * np.pi,
            x_tol=np.pi / 12,
            z_tol=np.pi / 12,
        )

        # Get the pot top pose
        pot_tf = await self.buffer.lookup_transform_async(
            "panda_link0", "pot_top", Time())

        # self.dump_pose_approach.orientation = Quaternion(
        #     x=-0.6987058,
        #     y=0.0,
        #     z=-0.2329019,
        #     w=0.6764369
        # )

        self.dump_pose_approach.orientation = Quaternion(
            x=-0.819152,
            y=0.0,
            z=0.0,
            w=0.5735764
        )

        dump_pose = tf2_geometry_msgs.do_transform_pose(
            self.dump_pose_approach, pot_tf)

        # MOTION 1) Move to above the pot
        await self.moveit_api.plan_async(
            point=dump_pose.position,
            orientation=dump_pose.orientation,
            path_constraints=path_constraints,
            use_jc=False,
            execute=True,
            y_tol=np.pi / 16,
            x_tol=np.pi / 16,
            z_tol=np.pi / 16,
        )

        # have the end effector retreat from the center a bit
        joint_states = self.moveit_api.get_current_joint_state()

        if joint_states["panda_joint7"] > 0:
            x_offset = 0.02
        else:
            x_offset = -0.05

        end_effector_tf = await self.buffer.lookup_transform_async(
            "panda_link0", "panda_hand_tcp", Time())
        retreat = Pose(
            position=Point(x=x_offset, y=0.05, z=-0.09),
            orientation=Quaternion()
        )
        retreat = tf2_geometry_msgs.do_transform_pose(
            retreat, end_effector_tf)

        # MOTION 2) Retreat backwards to center scoop
        await self.moveit_api.plan_async(
            point=retreat.position,
            orientation=retreat.orientation,
            use_jc=False,
            x_tol=0.1,
            y_tol=0.1,
            z_tol=0.1,
            execute=True,
        )

        # MOTION 3) DUMP
        joint_states = self.moveit_api.get_current_joint_state()
        joint_states_start = joint_states.copy()

        if joint_states["panda_joint7"] > 0:
            joint_states["panda_joint7"] -= np.pi
        else:
            joint_states["panda_joint7"] += np.pi

        names = list(joint_states.keys())
        pos = list(joint_states.values())
        await self.moveit_api.plan_joint_async(
            names, pos, execute=True)

        # MOTION 4) Undo rotate
        names = list(joint_states_start.keys())
        pos = list(joint_states_start.values())
        await self.moveit_api.plan_joint_async(
            names, pos, execute=True)

        # MOTION 5) Retreat from pot
        above_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.35),
            orientation=Quaternion()
        )
        above_pose = tf2_geometry_msgs.do_transform_pose(
            above_pose, pot_tf)
        await self.moveit_api.plan_async(
            point=above_pose.position,
            orientation=retreat.orientation,
            execute=True,
        )

    async def grab_scoop(self):
        """
        Description:
            Function to pick up the coffee scoop
        """
        try:
            tf = self.buffer.lookup_transform(
                "panda_link0", "filtered_coffee_scoop", Time(seconds=0.0))

        except Exception as e:
            self.get_logger().error("No transform found")
            return

        observe_pose = tf2_geometry_msgs.do_transform_pose(
            self.scoop_offset_pose_observe, tf)

        goal_msg = GraspProcess.Goal(
            observe_pose=observe_pose,
            refinement_pose=self.scoop_offset_pose_refine,
            approach_pose=self.scoop_offset_pose_approach,
            grasp_pose=self.scoop_offset_pose_grasp,
            width=0.005,
            force=50.0,
            speed=0.05,
            epsilon=GraspEpsilon(inner=0.01, outer=0.01),
            retreat_pose=self.scoop_offset_pose_retreat
        )

        self.get_logger().warn("MADE GRASP PROCESS GOAL")

        goal = await self.grasp_process.send_goal_async(goal_msg)
        res = await goal.get_result_async()
        self.scoop_actual_pickup_pose = res.result.actual_grasp_pose

    async def scoop_grounds(self):
        """
        Description:
            Function to scoop coffee grounds from the coffee pot
        """
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_coffee_grounds", Time())
        await self.moveit_api.go_home()
        scoop_pose_1 = tf2_geometry_msgs.do_transform_pose(
            self.scoop_pose_1, tf)
        self.get_logger().warn(f"MADE SCOOP POSE 1: {scoop_pose_1}")
        scoop_pose_2 = tf2_geometry_msgs.do_transform_pose(
            self.scoop_pose_2, tf)
        scoop_pose_3 = tf2_geometry_msgs.do_transform_pose(
            self.scoop_pose_3, tf)

        await self.moveit_api.plan_async(
            point=scoop_pose_1.position,
            # orientation=scoop_pose_1.orientation,
            execute=True,
            use_jc=False
        )

        await self.moveit_api.plan_async(
            point=scoop_pose_2.position,
            orientation=scoop_pose_2.orientation,
            execute=True,
            use_jc=True
        )

        await self.moveit_api.plan_async(
            point=scoop_pose_3.position,
            orientation=scoop_pose_3.orientation,
            execute=True,
            use_jc=True
        )

    async def return_scoop(self):
        """
        Description:
            Function to place the coffee scoop on the scoop tower
        """
        approach_pose = Pose(
            position=Point(x=0.0, y=-0.20, z=0.0),
            orientation=Quaternion()
        )

        grasp_pose = Pose(
            position=Point(x=0.0, y=-0.01, z=0.0),
            orientation=Quaternion()
        )

        retreat_pose = Pose(
            position=Point(x=0.0, y=-0.3, z=-0.18),
            orientation=Quaternion()
        )

        approach_pose = tf2_geometry_msgs.do_transform_pose(
            approach_pose, self.scoop_actual_pickup_pose)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            grasp_pose, self.scoop_actual_pickup_pose)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            retreat_pose, self.scoop_actual_pickup_pose)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.04,  # open the gripper wider to release the kettle
                force=50.0,
                speed=0.2,
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)


def coffee_grounds_entry(args=None):
    rclpy.init(args=args)
    coffee_grounds = CoffeeGrounds()
    rclpy.spin(coffee_grounds)
    rclpy.shutdown()
