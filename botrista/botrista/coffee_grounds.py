from rclpy.node import Node
import rclpy
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import tf2_geometry_msgs
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
from std_msgs.msg import Header
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from botrista_interfaces.action import GroundsAction
from rclpy.time import Time

class CoffeeGrounds(Node):
    """
    Measures coffee depth, scoops coffee, and dumps coffee in coffee maker.
    Also dumps used coffee grounds from filter.
    """
    def __init__(self):
        super().__init__('coffee_grounds')
        self.scoop_offset_pos = Point(x=0.1, y=0.0, z=0.2)
        self.grounds_offset_pos = Point(x=0.1, y=0.0, z=0.2)
        self.filter_handle_offset_pos = Point(x=0.1, y=0.0, z=0.2)
        self.filter_center_offset_pos = Point(x=0.1, y=0.0, z=0.2)
        self.filter_dump_pos = Point(x=0.5, y=0.5, z=0.2)

        self.scoop_offset_pos_approach = Point(x=0.1, y=0.0, z=0.2)
        self.grounds_offset_pos_approach = Point(x=0.1, y=0.0, z=0.2)
        self.filter_handle_offset_pos_approach = Point(x=0.1, y=0.0, z=0.2)
        self.filter_center_offset_pos_approach = Point(x=0.1, y=0.0, z=0.2)
        self.filter_dump_pos_approach = Point(x=0.5, y=0.5, z=0.2)

        self.scoop_offset_pos_retreat = Point(x=0.1, y=0.0, z=0.2)
        self.grounds_offset_pos_retreat = Point(x=0.1, y=0.0, z=0.2)
        self.filter_handle_offset_pos_retreat = Point(x=0.1, y=0.0, z=0.2)
        self.filter_center_offset_pos_retreat = Point(x=0.1, y=0.0, z=0.2)
        self.dump_position_retreat = Point(x=0.5, y=0.5, z=0.2)

        self.scoop_offset_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.grounds_offset_orient_approach = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.grounds_offset_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.grounds_offset_orient_retreat = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.filter_handle_offset_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.filter_center_offset_orient_upright = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.filter_center_offset_orient_flipped = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.dump_orientation_upright = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.dump_orientation_dump = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.grasp_command_scoop=Grasp.Goal(width=0.02, force=50.0, speed=0.05)
        self.grasp_command_filter=Grasp.Goal(width=0.02, force=50.0, speed=0.05)
        self.grasp_command_open=Grasp.Goal(width=0.02, force=50.0, speed=0.05)

        self.scoop_action_server = ActionServer(
            self,
            GroundsAction,
            'scoop',
            self.fill_coffee_maker)
        self.dump_action_server = ActionServer(
            self,
            GroundsAction,
            'dump',
            self.dump_coffee_filter)   
          
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


    def fill_coffee_maker(self, goal_handle):
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


    def dump_coffee_filter(self, goal_handle):
        result = GroundsAction()
        result.status = 0
        self.grab_filter()
        result.status = 1
        self.flip_shake_filter()
        result.status = 2
        self.place_filter()
        result.status = 3
        result.complete = True
        return result


    async def grab_scoop(self):
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_scoop_tag", Time())

        approach_pose = Pose(
            position=self.scoop_offset_pos_approach,
            orientation=self.scoop_offset_orient
        )
        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, tf)

        grasp_pose = Pose(
            position=self.scoop_offset_pos,
            orientation=self.scoop_offset_orient
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, tf)

        retreat_pose = Pose(
            position=self.scoop_offset_pos_retreat,
            orientation=self.scoop_offset_orient
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=self.grasp_command_scoop,
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)


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
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_scoop_tag", Time())

        approach_pose = Pose(
            position=self.scoop_offset_pos_retreat,
            orientation=self.scoop_offset_orient
        )
        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, tf)

        grasp_pose = Pose(
            position=self.scoop_offset_pos,
            orientation=self.scoop_offset_orient
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, tf)

        retreat_pose = Pose(
            position=self.scoop_offset_pos_approach,
            orientation=self.scoop_offset_orient
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=self.grasp_command_open,
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)
    

    async def grab_filter(self):
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_XXXX_tag", Time())

        approach_pose = Pose(
            position=self.filter_handle_offset_pos_approach,
            orientation=self.filter_handle_offset_orient
        )
        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, tf)

        grasp_pose = Pose(
            position=self.filter_handle_offset_pos,
            orientation=self.filter_handle_offset_orient
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, tf)

        retreat_pose = Pose(
            position=self.filter_handle_offset_pos_retreat,
            orientation=self.filter_handle_offset_orient
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=self.grasp_command_filter,
            retreat_pose=retreat_pose,
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


    async def place_filter(self):
        tf = self.buffer.lookup_transform(
            "panda_link0", "filtered_XXXX_tag", Time())

        approach_pose = Pose(
            position=self.filter_handle_offset_pos_approach,
            orientation=self.filter_handle_offset_orient
        )
        approach_pose = tf2_geometry_msgs.do_transform_pose(approach_pose, tf)

        grasp_pose = Pose(
            position=self.filter_handle_offset_pos,
            orientation=self.filter_handle_offset_orient
        )
        grasp_pose = tf2_geometry_msgs.do_transform_pose(grasp_pose, tf)

        retreat_pose = Pose(
            position=self.filter_handle_offset_pos_retreat,
            orientation=self.filter_handle_offset_orient
        )
        retreat_pose = tf2_geometry_msgs.do_transform_pose(retreat_pose, tf)

        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=self.grasp_command_open,
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)


    async def measure_coffee_height(self):
        pass


def coffee_grounds_entry(args=None):
    rclpy.init(args=args)
    coffee_grounds = CoffeeGrounds()
    rclpy.spin(coffee_grounds)
    rclpy.shutdown()
