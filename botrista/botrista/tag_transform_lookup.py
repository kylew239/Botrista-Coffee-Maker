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

        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter(
            'source_frame').get_parameter_value().string_value
        self.out_file = self.get_parameter(
            'out_file').get_parameter_value().string_value

        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self, qos=10)

    def lookup_transform(self):

        fut = Future()

        def transform_cb(self, transform):
            fut.set_result(transform)

        cor = self.buffer.lookup_transform_async(self.target_frame,
                                                 self.source_frame,
                                                 Duration(seconds=0.0))
        rclpy.get_global_executor().create_task(cor).add_done_callback(transform_cb)

        return fut


def tag_transform_entry(args=None):
    rclpy.init(args=args)
    tag_lookup = TagTransformLookup()

    transform_future = tag_lookup.lookup_transform()

    rclpy.spin_until_future_complete(tag_lookup, transform_future)

    transform: TransformStamped = transform_future.result

    with open(tag_lookup.out_file, 'w') as out_file:
        out_file.writelines([
            f"x: {transform.transform.translation.x}",
            f"y: {transform.transform.translation.y}",
            f"z: {transform.transform.translation.z}",
            f"qx: {transform.transform.rotation.x}",
            f"qy: {transform.transform.rotation.y}",
            f"qz: {transform.transform.rotation.z}",
            f"qw: {transform.transform.rotation.w}",
        ])

    rclpy.shutdown()
