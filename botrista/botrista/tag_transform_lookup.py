import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
from rclpy import Future
from geometry_msgs.msg import TransformStamped


class TagTransformLookup(Node):
    def __init__(self):
        super().__init__("tag_transform_lookup")

        self.declare_parameter('target_frame')
        self.declare_parameter('source_frame')
        self.declare_parameter('out_file')

        self.target_frame = "filtered_kettle_tag"
        self.source_frame = "panda_hand_tcp"

        self.buffer = Buffer()
        self.timer = self.create_timer(0.1, self.lookup_transform)
        self.transform_listener = TransformListener(self.buffer, self, qos=10)

    async def lookup_transform(self):

        fut = Future()

        def transform_cb(self, transform):
            fut.set_result(transform)

        tf = await self.buffer.lookup_transform_async(self.target_frame,
                                                      self.source_frame,
                                                      Duration(seconds=0.0))

        self.get_logger().info(f"Transform: {tf}")


def tag_transform_entry(args=None):
    rclpy.init(args=args)
    tag_lookup = TagTransformLookup()
    rclpy.spin(tag_lookup)
    rclpy.shutdown()
