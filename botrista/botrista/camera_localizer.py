""" 
A node that handles camera localization and tag filtering.

Publishes Transforms:
    + d405_link (geometry_msgs/TransformStamped) - The tf from the d405 camera to the robot.
    + filtered_camera_localizer_tag (geometry_msgs/TransformStamped) - The filtered tf of the camera localizer tag.
    + filtered_kettle_tag (geometry_msgs/TransformStamped) - The filtered tf of the kettle tag.
    + filtered_filter_tag (geometry_msgs/TransformStamped) - The filtered tf of the filter tag.
    + filtered_pour_over_tag (geometry_msgs/TransformStamped) - The filtered tf of the pour over tag.
    + filtered_coffee_scoop (geometry_msgs/TransformStamped) - The filtered tf of the coffee scoop tag.
"""
from rclpy.node import Node
import rclpy
from rclpy.time import Time
from std_msgs.msg import Header
from rclpy.duration import Duration
from tf2_ros import (
    TransformListener,
    Buffer,
    TransformBroadcaster,
    StaticTransformBroadcaster,
)
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import numpy as np
from rclpy.clock import Clock
from filtering.kalman_filter import KalmanFilter


class CameraLocalizer(Node):
    """A node that handles camera localization and tag filtering."""

    def __init__(self):
        super().__init__("camera_localizer")

        # create transform listener and buffer
        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.static_transform_broadcaster = StaticTransformBroadcaster(self)

        # timer to publish transforms
        self.timer = self.create_timer(0.1, self.timer_callback)

        # the tags to filter
        self.tags = [
            "camera_localizer_tag",
            "kettle_tag",
            "filter_tag",
            "pour_over_tag",
            "coffee_scoop",
        ]

        # create the filter objects for each tag
        self.filters = [
            FilterTag(
                self.transform_broadcaster,
                self.buffer,
                self.get_clock(),
                "d435i_color_optical_frame",
                "camera_localizer_tag",
            )
        ]
        self.filters.extend(
            [
                FilterTag(
                    self.transform_broadcaster,
                    self.buffer,
                    self.get_clock(),
                    "filtered_camera_localizer_tag",
                    tag,
                    predict_up=True,
                )
                for tag in self.tags[1:]
            ]
        )

    async def timer_callback(self):
        """Publish the transform for the d405 and filter each tag."""
        # publish d405 transform to the franka hand
        self.transform_broadcaster.sendTransform(
            TransformStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(), frame_id="panda_hand"
                ),
                child_frame_id="d405_link",
                transform=Transform(
                    translation=Vector3(x=0.04, y=0.0, z=0.05),
                    rotation=Quaternion(
                        x=0.706825, y=-0.0005629, z=0.707388, w=0.0005633
                    ),
                ),
            )
        )

        try:
            # find transform to localizer tag
            localizer_tag_to_franka_tf = TransformStamped()
            localizer_tag_to_franka_tf.header = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="filtered_camera_localizer_tag",
            )

            # measured transform between the localizer tag and the robot base
            localizer_tag_to_franka_tf.transform = Transform(
                translation=Vector3(
                    x=0.555 - 0.200 / 2,
                    y=0.302 - 0.200 / 2,
                ),
                rotation=Quaternion(x=0.0, y=0.0, z=0.9999997, w=0.0007963),
            )

            localizer_tag_to_franka_tf.child_frame_id = "panda_link0"
            self.transform_broadcaster.sendTransform(
                localizer_tag_to_franka_tf)

            # filter the position of each tag.
            for filter in self.filters:
                filter.filter()

        except Exception as e:
            self.get_logger().warn(f"Exception: {e}")


class FilterTag:
    """Handles filtering the position of a tag using a Kalman filter."""

    def __init__(
        self,
        tf_broadcaster,
        tf_buffer: Buffer,
        clock,
        target_frame,
        source_frame,
        predict_up=False,
    ):
        """
        Initialize the filter.

        :param tf_broadcaster: The Tf broadcaster to use.
        :type tf_broadcaster: TransformBroadcaster
        :param tf_buffer: The Tf buffer to use.
        :type tf_buffer: Buffer
        :param clock: The clock to use.
        :type clock: Clock
        :param target_frame: The target frame to publish the filtered transform to.
        :type target_frame: str
        :param source_frame: The source frame to filter.
        :type source_frame: str
        :param predict_up: Whether or not to predict the tag facing directly up.
        :type predict_up: bool
        """
        self.mean = None
        self.sigma = np.identity(7) * 5.0
        self.tf_broadcaster = tf_broadcaster
        self.tf_buffer = tf_buffer
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.clock: Clock = clock
        self.predict_up = predict_up

        A = np.identity(7)
        R = np.identity(7) * 0.01
        Q = np.array([
            [0.05, 0, 0, 0, 0, 0, 0],
            [0, 0.05, 0, 0, 0, 0, 0],
            [0, 0, 2.0, 0, 0, 0, 0],
            [0, 0, 0, 10.0, 0, 0, 0],
            [0, 0, 0, 0, 10.0, 0, 0],
            [0, 0, 0, 0, 0, 10.0, 0],
            [0, 0, 0, 0, 0, 0, 10.0]
        ])

        c_vals = [1.0] * 7
        if self.predict_up:
            # always predict the tag to be facing up
            c_vals[3] = 0.0
            c_vals[4] = 0.0
        C = np.diagonal(c_vals)

        self.kalman_filter = KalmanFilter(A, R, Q, C)

    def filter(self):
        """Run the filter and publish the filtered transform."""
        now = self.clock.now()
        s, _ = now.seconds_nanoseconds()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time(seconds=s - 1),
                Duration(seconds=int(1.0 / 6.0)),
            )
        except Exception:
            tf = None

        if tf is not None:
            vec = self.tf_to_vec(tf)
            if self.mean is None:
                self.mean = vec
            else:
                self.mean, self.sigma = self.kalman_filter.filter(
                    self.mean, self.sigma, vec)

        if self.mean is not None:
            filtered_tag = TransformStamped()
            filtered_tag.header = Header(
                stamp=self.clock.now().to_msg(), frame_id=self.target_frame
            )
            filtered_tag.child_frame_id = "filtered_" + self.source_frame
            filtered_tag.transform = self.vec_to_tf(self.mean)
            self.tf_broadcaster.sendTransform(filtered_tag)

    def tf_to_vec(self, tf: TransformStamped):
        """
        Convert the TF to a measurement vector.

        :param tf: The transform to convert.
        :type tf: TransformStamped
        :return: A 7 length column vector of the transform.
        :rtype: np.array
        """
        return np.array(
            [
                [
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                ]
            ]
        ).T

    def vec_to_tf(self, vec):
        """
        Undo the vec_to_tf conversion.

        :param vec: 7 length column vector of the transform.
        :type vec: np.array
        :return: The Tf transform.
        :rtype: Transform
        """
        tf = Transform()
        vec = vec.flatten().ravel()
        tf.translation.x = vec[0]
        tf.translation.y = vec[1]
        tf.translation.z = vec[2]
        tf.rotation.x = vec[3]
        tf.rotation.y = vec[4]
        tf.rotation.z = vec[5]
        tf.rotation.w = vec[6]
        return tf

    def vec_to_tf(self, vec):
        """
        Undo the vec_to_tf conversion.

        :param vec: 7 length column vector of the transform.
        :type vec: np.array
        :return: The Tf transform.
        :rtype: Transform
        """
        tf = Transform()
        vec = vec.flatten().ravel()
        tf.translation.x = vec[0]
        tf.translation.y = vec[1]
        tf.translation.z = vec[2]
        tf.rotation.x = vec[3]
        tf.rotation.y = vec[4]
        tf.rotation.z = vec[5]
        tf.rotation.w = vec[6]
        return tf


def camera_localizer_entry(args=None):
    rclpy.init(args=args)
    camera_localizer = CameraLocalizer()
    rclpy.spin(camera_localizer)
    rclpy.shutdown()
