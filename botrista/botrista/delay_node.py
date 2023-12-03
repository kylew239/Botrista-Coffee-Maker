import rclpy
from rclpy.node import Node
from time import sleep
from std_srvs.srv import Empty


class Delay_Node(Node):
    def __init__(self):
        super().__init__("delay_node")
        self.delay_timer = self.create_service(
            Empty, "delay", self.delay_callback)

    def delay_callback(self, request, response):
        sleep(3)
        return response


def delay_entry(args=None):
    rclpy.init(args=args)
    node = Delay_Node()
    rclpy.spin(node)
    rclpy.shutdown()
