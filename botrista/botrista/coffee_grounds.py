from rclpy.node import Node
import rclpy
from moveit_wrapper.moveitapi import MoveItApi
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
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
        self.dump_position = Point(x=0.5, y=0.5, z=0.2)
        self.scoop_offset_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.grounds_offset_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.filter_handle_offset_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.filter_center_offset_orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.dump_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
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

    def grab_scoop(self):
        pass

    def scoop_grounds(self):
        pass
    
    def dump_grounds(self):
        pass
    
    def return_scoop(self):
        pass
    
    def grab_filter(self):
        pass
    
    def flip_shake_filter(self):
        pass
    
    def place_filter(self):
        pass

    def measure_coffee_height(self):
        pass
    
    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

def coffee_grounds_entry(args=None):
    rclpy.init(args=args)
    coffee_grounds = CoffeeGrounds()
    rclpy.spin(coffee_grounds)
    rclpy.shutdown()
