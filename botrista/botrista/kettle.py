import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
from control_msgs.msg import GripperCommand
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from franka_msgs.action import Grasp
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep


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
        
        self.delay_client = self.create_client(
            Empty, "delay", callback_group=ReentrantCallbackGroup()
        )
        while not self.delay_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for delay service")

        self.observe_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.40),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        # grasp points in handle frame
        self.approach_point = Point(x=0.0, y=0.0, z=0.15)
        self.grasp_point = Point(x=0.02, y=0.0, z=0.0)
        self.retreat_point = Point(x=0.0, y=0.0, z=0.10)


    async def grab(self, request, response):
        """
        Grabs the kettle from its stand.
        """
        try:
            tf = self.buffer.lookup_transform(
                "panda_link0", "filtered_kettle_tag", Time())

        except Exception as e:
            return response
            self.get_logger().warn(e)

        

        # go to the observe pose
        observe_pose = tf2_geometry_msgs.do_transform_pose(
            self.observe_pose, tf)
        await self.moveit_api.plan_async(point=observe_pose.position, orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0), execute=True)
        await self.delay_client.call_async(Empty.Request())
        # get the handle tf
        handle_tf = self.buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0))
        self.get_logger().warn(f"HANDLE TF: {handle_tf}")

        observe_point = tf2_geometry_msgs.do_transform_point(
            PointStamped(
                header=Header(
                    frame_id="handle",
                    stamp=self.get_clock().now().to_msg()
                ),
                point=Point(
                    x=self.approach_point.x,
                    y=self.approach_point.y,
                    z=0.15
                )), handle_tf)

        await self.moveit_api.plan_async(point=observe_point.point, orientation=Quaternion(
            x=1.0, y=0.0, z=0.0, w=0.0), execute=True)
        await self.delay_client.call_async(Empty.Request())
        handle_tf = self.buffer.lookup_transform(
            "panda_link0", "handle", time=Time(seconds=0.0))
        # sleep(2)

        # put the grasp points in into panda frame
        approach_point = tf2_geometry_msgs.do_transform_point(
            PointStamped(
                header=Header(
                    frame_id="handle",
                    stamp=self.get_clock().now().to_msg()
                ),
                point=self.approach_point), handle_tf)
        grasp_point = tf2_geometry_msgs.do_transform_point(
            PointStamped(
                header=Header(
                    frame_id="handle",
                    stamp=self.get_clock().now().to_msg()
                ),
                point=self.grasp_point), handle_tf)
        retreat_point = tf2_geometry_msgs.do_transform_point(
            PointStamped(
                header=Header(
                    frame_id="handle",
                    stamp=self.get_clock().now().to_msg()
                ),
                point=self.retreat_point), handle_tf)

        approach_pose = Pose(
            position=approach_point.point,
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        grasp_pose = Pose(
            position=grasp_point.point,
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        retreat_pose = Pose(
            position=retreat_point.point,
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.03,
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

        hover_pose = Pose(
            position=Point(
                x=0.09,
                y=0.043,
                z=0.18),
            orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(hover_pose, tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, tf)

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

        return response


def kettle_entry(args=None):
    rclpy.init(args=args)
    kettle = Kettle()
    rclpy.spin(kettle)
    rclpy.shutdown()
