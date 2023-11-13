from rclpy.node import Node
import rclpy


class CameraLocalizer(Node):
    def __init__(self):
        super().__init__('camera_localizer')

        pass


def main(args=None):
    rclpy.init(args=args)
    camera_localizer = CameraLocalizer()
    rclpy.spin(camera_localizer)
    rclpy.shutdown()
