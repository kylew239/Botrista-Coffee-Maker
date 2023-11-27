from rclpy.node import Node
import rclpy
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Vector3,
    TransformStamped,
    Transform
)
from std_msgs.msg import Header
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer, TransformBroadcaster

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
        

    def timer_callback(self):
        pass

    def fill_coffee_maker(self):
        self.measure_coffee_height()
        self.grab_scoop()
        self.scoop_grounds()
        self.dump_grounds()
        self.return_scoop()

    def dump_coffee_filter(self):
        self.grab_filter()
        self.flip_shake_filter()
        self.place_filter()

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
