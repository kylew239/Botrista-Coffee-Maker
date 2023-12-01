import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, TransformStamped, Transform
from std_msgs.msg import Empty
import tf2_geometry_msgs
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from franka_msgs.action import Grasp
from rclpy.time import Time
from botrista_interfaces.action import GraspProcess
from rclpy.action import ActionServer, ActionClient


class GraspNode(Node):
    """
    Performs the grasping action of the standard handles.

    Actions:
        + /grasp 
    """

    def __init__(self):
        super().__init__('GraspNode')

        self.grasp_process_action_server = ActionServer(
            self,
            GraspProcess,
            'grasp_process',
            self.grasp_process)


    async def grasp_process(self, goal_handle):
        """
        Grabs a specified object.
        """
        observe_pose = goal_handle.request.observe_pose
        refinement_offset = goal_handle.request.refinement_pose
        approach_offset = goal_handle.request.approach_pose
        grasp_offset = goal_handle.request.grasp_pose
        retreat_offset = goal_handle.request.retreat_pose
        gripper_command = goal_handle.request.gripper_command

        # move to observe point
        await self.moveit_api.plan_async(point=observe_pose.position, orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0), execute=True)
        await self.delay_client.call_async(Empty.Request())

        # get the handle tf
        handle_tf = self.buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0))
        self.get_logger().warn(f"HANDLE TF: {handle_tf}")

        # move to refinement point
        refinement_point = tf2_geometry_msgs.do_transform_pose(
            refinement_offset, handle_tf)

        self.get_logger().warn(f"REFINEMNENT POINT: {refinement_point}")
        await self.moveit_api.plan_async(point=refinement_point.position, orientation=refinement_point.orientation, execute=True)
        await self.delay_client.call_async(Empty.Request())
        handle_tf = self.buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0))

        # put the grasp points in into panda frame
        approach_pose = tf2_geometry_msgs.do_transform_pose(
            approach_offset, handle_tf)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            grasp_offset, handle_tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            retreat_offset, handle_tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            retreat_pose=retreat_pose,
            grasp_command=gripper_command
            )

        actual_grasp_pose = await self.grasp_planner.execute_grasp_plan(grasp_plan)

        result = GraspProcess.Result()
        result.actual_grasp_pose = actual_grasp_pose
        return result
