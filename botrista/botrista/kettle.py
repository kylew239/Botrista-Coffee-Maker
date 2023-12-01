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
from franka_msgs.msg import GraspEpsilon
from rclpy.callback_groups import ReentrantCallbackGroup
from time import sleep
from franka_msgs.msg import GraspEpsilon
from botrista_interfaces.action import EmptyAction, GraspProcess
from rclpy.action import ActionServer, ActionClient


class Kettle(Node):

    def __init__(self):
        super().__init__("kettle")
        
        self.kettle_actual_place = TransformStamped()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.moveit_api = MoveItApi(
            self, "panda_link0", "panda_hand_tcp", "panda_manipulator", "/franka/joint_states")
        self.grasp_planner = GraspPlanner(
            self.moveit_api, "panda_gripper/grasp")

        self.delay_client = self.create_client(
            Empty, "delay", callback_group=ReentrantCallbackGroup()
        )
        self.pick_kettle_client = ActionServer(self,
                                               EmptyAction,
                                               "pick_kettle",
                                               self.pick_kettle_cb,
                                               callback_group=ReentrantCallbackGroup())
        self.place_kettle_client = ActionServer(self,
                                                EmptyAction,
                                                "place_kettle",
                                                self.place_kettle_cb,
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
        self.approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.10),
            orientation=Quaternion())
        self.grasp_pose = Pose(
            position=Point(x=0.02, y=0.0, z=0.0),
            orientation=Quaternion())
        self.retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.10),
            orientation=Quaternion())

    async def pick_kettle_cb(self, goal_handle):
        """
        Grabs the kettle from its stand.
        """
        #### TFs
        # pour_over_tag
        try:
            tf = self.buffer.lookup_transform(
                "panda_link0", "filtered_kettle_tag", Time())

        except Exception as e:
            self.get_logger().error("No transform found")
            return
        
        observe_pose = tf2_geometry_msgs.do_transform_pose(
            self.observe_pose, tf)

        refinement_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.15),
            orientation=Quaternion())

        goal_msg = GraspProcess.Goal(
            observe_pose=observe_pose,
            refinement_pose=refinement_pose,
            approach_pose=self.approach_pose,
            grasp_pose=self.grasp_pose,
            width=0.03,
            force=50.0,
            speed=0.05,
            epsilon=GraspEpsilon(inner=0.01, outer=0.01),
            retreat_pose=self.retreat_pose
        )

        self.get_logger().warn("MADE GRASP PROCESS GOAL")

        goal = await self.grasp_process.send_goal_async(goal_msg)
        res = await goal.get_result_async()
        self.kettle_actual_place = res.result.actual_grasp_pose
        
        goal_handle.succeed()
        return EmptyAction.Result()
        # try:
        #     tf = self.buffer.lookup_transform(
        #         "panda_link0", "filtered_pour_over_tag", Time())

        # except Exception as e:
        #     self.get_logger().error("No transform found")
        #     return

        # # go to the observe pose
        # observe_pose = tf2_geometry_msgs.do_transform_pose(
        #     self.observe_pose, tf)
        # await self.moveit_api.plan_async(point=observe_pose.position, orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0), execute=True)
        # await self.delay_client.call_async(Empty.Request())

        # # get the handle tf
        # handle_tf = self.buffer.lookup_transform(
        #     "panda_link0", "handle", time=Time(seconds=0.0))
        # self.get_logger().warn(f"HANDLE TF: {handle_tf}")

        # observe_point = tf2_geometry_msgs.do_transform_pose(
        #     self.approach_pose, handle_tf)

        # self.get_logger().warn(f"OBSERVE POINT: {observe_point}")
        # await self.moveit_api.plan_async(point=observe_point.position, orientation=observe_point.orientation, execute=True)
        # await self.delay_client.call_async(Empty.Request())
        # handle_tf = self.buffer.lookup_transform(
        #     "panda_link0", "handle", time=Time(seconds=0.0))

        # # put the grasp points in into panda frame
        # approach_pose = tf2_geometry_msgs.do_transform_pose(
        #     self.approach_pose, handle_tf)
        # grasp_pose = tf2_geometry_msgs.do_transform_pose(
        #     self.grasp_pose, handle_tf)
        # retreat_pose = tf2_geometry_msgs.do_transform_pose(
        #     self.retreat_pose, handle_tf)


        # grasp_plan = GraspPlan(
        #     approach_pose=approach_pose,
        #     grasp_pose=grasp_pose,
        #     grasp_command=Grasp.Goal(
        #         width=0.03,
        #         force=50.0,
        #         speed=0.05,
        #         epsilon=GraspEpsilon(inner=0.01, outer=0.01)
        #     ),
        #     retreat_pose=retreat_pose,
        # )

        # await self.grasp_planner.execute_grasp_plan(grasp_plan)

    async def place_kettle_cb(self, goal_handle):
        """
        Places the kettle on its stand.
        """
        # tf = self.buffer.lookup_transform(
        #     "panda_link0", "filtered_kettle_tag", Time())

        # # play the grasp plan backwards to place the kettle
        # approach_pose = tf2_geometry_msgs.do_transform_pose(
        #     self.retreat_pose, tf)

        # hover_pose = Pose(
        #     position=Point(
        #         x=0.09,
        #         y=0.043,
        #         z=0.18),
        #     orientation=Quaternion(x=0.88, y=-0.035, z=0.47, w=0.01)
        # )
        # # Using stored grasp pose instead of calculated one
        # # grasp_pose = tf2_geometry_msgs.do_transform_pose(hover_pose, tf)
        # retreat_pose = tf2_geometry_msgs.do_transform_pose(
        #     self.approach_pose, tf)
        
        approach_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.1),
            orientation=Quaternion()
        )

        grasp_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.04),
            orientation=Quaternion()
        )

        retreat_pose = Pose(
            position=Point(x=0.0, y=0.0, z=-0.1),
            orientation=Quaternion()
        )


        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, self.kettle_actual_place)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, self.kettle_actual_place)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, self.kettle_actual_place)


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

def kettle_entry(args=None):
    rclpy.init(args=args)
    kettle = Kettle()
    rclpy.spin(kettle)
    rclpy.shutdown()
