"""
Offers a service that allows the robot to wait for a specified amount of time.

Services Offered:
    + delay (DelayTime) - pauses for a specified amount of time.
"""
import rclpy
from rclpy.node import Node
from time import sleep
from botrista_interfaces.srv import DelayTime
from rclpy.callback_groups import ReentrantCallbackGroup


class Delay_Node(Node):
    def __init__(self):
        """Initialize the delay_node."""
        super().__init__("delay_node")
        self.delay_timer = self.create_service(
            DelayTime, "delay", self.delay_callback, callback_group=ReentrantCallbackGroup())

    def delay_callback(self, request, response):
        """
        Delays for the requested amount of time.

        Keyword Arguments:
            + request (botrista_interfaces_srv/DelayTime)-- Delay time.

        Returns
        -------
            + Empty return

        """
        sleep(request.time)
        return response


def delay_entry(args=None):
    rclpy.init(args=args)
    node = Delay_Node()
    rclpy.spin(node)
    rclpy.shutdown()
