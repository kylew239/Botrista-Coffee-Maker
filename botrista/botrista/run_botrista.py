import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from franka_msgs.action import Grasp
from rclpy.time import Time
from time import sleep
from franka_msgs.msg import GraspEpsilon
from botrista_interfaces.action import EmptyAction, GraspProcess
from rclpy.action import ActionServer, ActionClient


class Botrista(Node):

    def __init__(self):
        super().__init__("botrista")




def botrista_entry(args=None):
    rclpy.init(args=args)
    botrista = Botrista()
    rclpy.spin(botrista)
    rclpy.shutdown()
