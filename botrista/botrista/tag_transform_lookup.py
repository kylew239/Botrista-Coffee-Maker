import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time


class TagTransformLookup(Node):
    def __init__(self):
        super().__init__("tag_transform_lookup")

        self.target_frame = "filtered_kettle_tag"
        self.source_frame = "panda_hand_tcp"

        self.buffer = Buffer()
        self.timer = self.create_timer(1.0, self.lookup_transform)
        self.transform_listener = TransformListener(self.buffer, self, qos=10)

    def lookup_transform(self):

        tf = self.buffer.lookup_transform(self.target_frame,
                                          self.source_frame,
                                          Time())

        self.get_logger().warn(f"Transform: {tf}")


def tag_transform_entry(args=None):
    rclpy.init(args=args)
    tag_lookup = TagTransformLookup()
    rclpy.spin(tag_lookup)
    rclpy.shutdown()
