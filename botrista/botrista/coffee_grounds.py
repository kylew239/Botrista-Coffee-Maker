from rclpy.node import Node
import rclpy
import numpy as np
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
import tf2_geometry_msgs
from franka_msgs.msg import GraspEpsilon
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Vector3,
    TransformStamped,
    Transform
)
from franka_msgs.action import (
    Grasp,
)
from rclpy.action import ActionServer, ActionClient
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from botrista_interfaces.action import GroundsAction
from botrista_interfaces.action import GraspProcess
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
from image_geometry.cameramodels import PinholeCameraModel
import pyrealsense2


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

        # define observation poses (relative to april tag)
        self.scoop_offset_pose_observe = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.filter_center_offset_pose_observe = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.filter_dump_pose_observe = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )

        # define refinement, approach, grasp, and retreat poses (relative to handle)
        self.scoop_offset_pose_refine = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.filter_center_offset_pose_refine = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.filter_dump_pose_refine = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.scoop_offset_pose_approach = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.filter_center_offset_pose_approach = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.filter_dump_pose_approach = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.scoop_offset_pose_grasp = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.filter_center_offset_pose_grasp = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )
        self.filter_dump_pose_grasp = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0)
            )   

        # define scooping poses (relative to coffee ground april tag)
        self.scoop_pose_measure = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0))
        self.scoop_pose_1 = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0))
        self.scoop_pose_2 = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0))
        self.scoop_pose_3 = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0))
        
        # define dumping poses (relative to pot april tag)
        self.dump_pose_1 = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0))
        self.dump_pose_2 = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0))
        self.dump_pose_3 = Pose(
            position=Point(
                x=0.1, y=0.0, z=0.2),
            orientation=Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0))
 
        # Initialize actual pickup positions
        self.scoop_actual_pickup_pose = Pose()    

        # define grasp commands
        self.grasp_command_scoop = Grasp.Goal(
            width=0.02, force=50.0, speed=0.05)
        self.grasp_command_open = Grasp.Goal(
            width=0.02, force=50.0, speed=0.05)

        # start action server for scooping routine
        self.scoop_action_server = ActionServer(
            self,
            GroundsAction,
            'scoop',
            self.scoop_coffee_grounds)
        
        # start action client for grasp process action
        self.action_client = ActionClient(self, GraspProcess, 'grasp_process')

        # create subscription to camera topic
        self.image_subscription = self.create_subscription(
            Image, "/camera/d405/color/image_rect_raw", self.image_callback, qos_profile=10)

        # subscription to the depth image
        self.depth_image_subscription = self.create_subscription(
            Image, "/camera/d405/aligned_depth_to_color/image_raw", self.depth_image_callback, qos_profile=10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "/camera/d405/aligned_depth_to_color/camera_info", self.camera_info_callback, qos_profile=10)

        # published the masked depth image
        self.depth_publisher = self.create_publisher(
            Image, "/depth_mask", qos_profile=10
        )

        self.cv_bridge = cv_bridge.CvBridge()

        self.camera_model = PinholeCameraModel()

        # camera intrinsics
        self.depth_image = None
        self.intrinsics = None

        # Create tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_parent_frame = "panda_link0"

        # Moveit Wrapper Object
        self.moveit = MoveItApi(self,
                                "panda_link0",
                                "panda_hand_tcp",
                                "panda_manipulator",
                                "joint_states",
                                "panda")


    def scoop_coffee_grounds(self, goal_handle):
        """
        Description:
            Action callback for the scooping routine, scoops coffee and dumps it in the filter
        """
        result = GroundsAction()
        result.status = 0
        self.measure_coffee_height()
        result.status = 1
        self.grab_scoop()
        result.status = 2
        self.scoop_grounds()
        result.status = 3
        self.dump_grounds()
        result.status = 4
        self.return_scoop()
        result.status = 5
        result.complete = True
        return result


    async def grab_scoop(self):
        """
        Description:
            Function to pick up the coffee scoop
        """
        try:
            tf = self.buffer.lookup_transform(
                "panda_link0", "filtered_coffee_scoop", Time())

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
            width=0.03,
            force=50.0,
            speed=0.05,
            epsilon=GraspEpsilon(inner=0.01, outer=0.01),
            retreat_pose=self.retreat_pose
        )

        self.get_logger().warn("MADE GRASP PROCESS GOAL")

        goal = await self.grasp_process.send_goal_async(goal_msg)
        res = await goal.get_result_async()
        self.scoop_actual_pickup_pose = res.result.actual_grasp_pose


    async def scoop_grounds(self):
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_grounds_tag", Time())

        approach_pose = Pose(
            position=self.grounds_offset_pos_approach,
            orientation=self.grounds_offset_orient_approach
        )
        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, tf)

        grasp_pose = Pose(
            position=self.grounds_offset_pos,
            orientation=self.grounds_offset_orient
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, tf)

        retreat_pose = Pose(
            position=self.grounds_offset_pos_retreat,
            orientation=self.grounds_offset_orient_retreat
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=self.grasp_command_scoop,
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)


    async def dump_grounds(self):
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_XXXX_tag", Time())

        approach_pose = Pose(
            position=self.filter_center_offset_pos_approach,
            orientation=self.filter_center_offset_orient_upright
        )
        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, tf)

        grasp_pose = Pose(
            position=self.filter_center_offset_pos,
            orientation=self.filter_center_offset_orient_flipped
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, tf)

        retreat_pose = Pose(
            position=self.filter_center_offset_pos_retreat,
            orientation=self.filter_center_offset_orient_upright
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=self.grasp_command_scoop,
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)


    async def return_scoop(self):
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

        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, self.scoop_actual_pickup_pose)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, self.scoop_actual_pickup_pose)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, self.scoop_actual_pickup_pose)

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


    async def flip_shake_filter(self):
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_TRASH_tag", Time())

        approach_pose = Pose(
            position=self.filter_dump_pos_approach,
            orientation=self.dump_orientation_upright
        )
        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, tf)

        grasp_pose = Pose(
            position=self.filter_dump_pos,
            orientation=self.dump_orientation_flipped
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, tf)

        retreat_pose = Pose(
            position=self.filter_dump_pos_retreat,
            orientation=self.dump_orientation_upright
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=self.grasp_command_filter,
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)


    def camera_info_callback(self, camera_info):
        self.intrinsics = pyrealsense2.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.k[2]
        self.intrinsics.ppy = camera_info.k[5]
        self.intrinsics.fx = camera_info.k[0]
        self.intrinsics.fy = camera_info.k[4]
        self.intrinsics.model = pyrealsense2.distortion.none
        self.intrinsics.coeffs = [i for i in camera_info.d]


    def depth_image_callback(self, image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(
            image, desired_encoding="passthrough")


    def measure_coffee_height(self):
        self.move_to_measure_pose()

        
        # get the 3d point
        point = pyrealsense2.rs2_deproject_pixel_to_point(
            self.intrinsics, [cX, cY], depth)
        point = np.array(point)
        point /= 1000.0  # mm to m


    def move_to_measure_pose(self):
        """
        Description: Move robot to specifcied position and orientation to measure coffee ground height
        """
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_coffee_grounds", Time())
        
        measure_pose = tf2_geometry_msgs.do_transform_pose(self.scoop_pose_measure, tf)     

        fut = self.moveit_arm.plan(
                        point=measure_pose.position,
                        orientation=measure_pose.orientation,
                        execute=True,
                        use_jc=True
                        )
        return fut   


def coffee_grounds_entry(args=None):
    rclpy.init(args=args)
    coffee_grounds = CoffeeGrounds()
    rclpy.spin(coffee_grounds)
    rclpy.shutdown()
