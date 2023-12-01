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
from botrista_interfaces.action import EmptyAction
from botrista_interfaces.action import PourAction
from rclpy.action import ActionServer, ActionClient


class Botrista(Node):
    """
    Description:
        Waits to detect an empty cup placed in the detection area, then runs following routine:
            1. Turns on Kettle (action)
            2. Pick up Filter from filter stand (action)
            3. Place Filter on Pot (action)
            4. Scoop grounds (scoop action)
            5. Wait for boiling
            6. Pick up Kettle (pick_kettle action)
            7. Pour water from kettle (pour_action action)
            8. Place Kettle on kettle stand (place_kettle action)
            9. Wait for coffee grounds to soak
            10. Pick up Filter from Pot (action)
            11. Place Filter on filter stand (action)
            12. Pick up Pot from pot stand (pick_pot action)
            13. Pour Coffee (pour_action action)
            14. Put Pot on pot stand (place_pot action)
    """
    def __init__(self):
        super().__init__("botrista")
        
        # start action client for pick_kettle action
        self.action_client_pick_kettle = ActionClient(self, EmptyAction, 'pick_kettle')
        
        # start action client for place_kettle action
        self.action_client_place_kettle = ActionClient(self, EmptyAction, 'place_kettle')
        
        # start action client for pick_pot action
        self.action_client_pick_pot = ActionClient(self, EmptyAction, 'pick_pot')
        
        # start action client for place_pot action
        self.action_client_place_pot = ActionClient(self, EmptyAction, 'place_pot')

        # start action client for pour_action action
        self.action_client_pour_action = ActionClient(self, PourAction, 'pour_action')
        
        # start action client for scoop action
        self.action_client_scoop = ActionClient(self, EmptyAction, 'scoop')

        # start action client for pour_action action
        self.action_client_pour_action = ActionClient(self, PourAction, 'pour_action')




def botrista_entry(args=None):
    rclpy.init(args=args)
    botrista = Botrista()
    rclpy.spin(botrista)
    rclpy.shutdown()
