from rclpy.node import Node
import rclpy
from std_msgs.msg import Header
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform, Vector3


class CameraLocalizer(Node):
    """
    Localizes the ceiling mounted d435i camera to the robot
    """

    def __init__(self):
        super().__init__('camera_localizer')

        # create transform listener and buffer
        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self, qos=10)
        self.transform_broadcaster = TransformBroadcaster(self, qos=10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # find transform to localizer tag
            localizer_tag_to_franka_tf = TransformStamped()
            localizer_tag_to_franka_tf.header = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='camera_localizer_tag'
            )

            localizer_tag_to_franka_tf.transform = Transform(
                translation=Vector3(
                    x=0.555,
                    y=0.302
                )
            )

            localizer_tag_to_franka_tf.child_frame_id = "franka"
            self.transform_broadcaster.sendTransform(
                localizer_tag_to_franka_tf)

        except Exception as e:
            pass


def camera_localizer_entry(args=None):
    rclpy.init(args=args)
    camera_localizer = CameraLocalizer()
    rclpy.spin(camera_localizer)
    rclpy.shutdown()
