import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from franka_msgs.msg import GraspEpsilon
from geometry_msgs.msg import Point, Quaternion, Pose
from rclpy.action import ActionServer
from botrista_interfaces.action import EmptyAction
from std_srvs.srv import Empty
import tf2_geometry_msgs
from franka_msgs.action import Grasp


class Pick_filter(Node):
    def __init__(self):
        super().__init__("pick_filter")
        self.cb = ReentrantCallbackGroup()

        # Creating tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_parent_frame = "panda_link0"

        base_frame = "panda_link0"
        end_effector_frame = "panda_hand_tcp"
        group_name = "panda_manipulator"
        self.moveit = MoveItApi(
            self,
            base_frame=base_frame,
            end_effector_frame=end_effector_frame,
            group_name=group_name,
            joint_state_topic="joint_states",
        )
        self.grasp_planner = GraspPlanner(
            self.moveit, "panda_gripper/grasp")
        
        self.delay_client = self.create_client(
            Empty, "delay", callback_group=ReentrantCallbackGroup()
        )

        # Creating action server
        self._action_server = ActionServer(self,
                                           EmptyAction,
                                           'pick_filter_action',
                                           self.pick_filter_callback,
                                           callback_group=self.cb)
        
        self.observe_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.40),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        # grasp points in handle frame
        self.approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.15),
            orientation=Quaternion())
        self.grasp_pose = Pose(
            position=Point(x=0.02, y=0.0, z=0.0),
            orientation=Quaternion())
        self.retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.10),
            orientation=Quaternion())
        
    async def pick_filter_callback(self, goal):
        try:
            tf = self.tf_buffer.lookup_transform(
                "panda_link0", "filtered_filter_tag", Time())
            # tf2 = self.tf_buffer.lookup_transform(
            #     "panda_link0", "filtered_pour_over_tag", Time())

        except Exception as e:
            self.get_logger().error("No transform found")
            return

        # go to the observe pose
        observe_pose = tf2_geometry_msgs.do_transform_pose(
            self.observe_pose, tf)
        await self.moveit.plan_async(point=observe_pose.position, orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0), execute=True)
        await self.delay_client.call_async(Empty.Request())

        # get the handle tf
        handle_tf = self.tf_buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0))
        # self.get_logger().warn(f"HANDLE TF: {handle_tf}")

        observe_point = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, handle_tf)

        # self.get_logger().warn(f"OBSERVE POINT: {observe_point}")
        await self.moveit.plan_async(point=observe_point.position, orientation=observe_point.orientation, execute=True)
        await self.delay_client.call_async(Empty.Request())
        handle_tf = self.tf_buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0))

        # put the grasp points in into panda frame
        approach_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, handle_tf)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            self.grasp_pose, handle_tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            self.retreat_pose, handle_tf)

        # Store the grasping pose for placing
        self.kettle_place_pose = grasp_pose

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.02985,
                force=50.0,
                speed=0.05,
                epsilon=GraspEpsilon(inner=0.01, outer=0.01)
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)
        
        """
        # place filter above the kettle
        observe_pose_kettle = tf2_geometry_msgs.do_transform_pose(
            self.observe_pose, tf2)
        await self.moveit.plan_async(point=observe_pose_kettle.position, orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0), execute=True)
        await self.delay_client.call_async(Empty.Request()) 

        # get the handle tf
        handle_tf_kettle = self.tf_buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0))
        # self.get_logger().warn(f"HANDLE TF: {handle_tf}")

        observe_point_kettle = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, handle_tf_kettle)
        
        # self.get_logger().warn(f"OBSERVE POINT: {observe_point}")
        await self.moveit.plan_async(point=observe_point_kettle.position, orientation=observe_point_kettle.orientation, execute=True)
        await self.delay_client.call_async(Empty.Request())
        handle_tf_kettle = self.tf_buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0))

        # put the grasp points in into panda frame
        approach_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, handle_tf_kettle)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(
            self.grasp_pose, handle_tf_kettle)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            self.retreat_pose, handle_tf_kettle)

        # Store the grasping pose for placing
        self.kettle_place_pose = grasp_pose

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.02985,
                force=50.0,
                speed=0.05,
                epsilon=GraspEpsilon(inner=0.01, outer=0.01)
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)
        """


def main(args=None):
    rclpy.init(args=args)
    node = Pick_filter()
    rclpy.spin(node)
    rclpy.shutdown()

