from rclpy.node import Node
import rclpy
from rclpy.time import Time
from std_msgs.msg import Header
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer, TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import numpy as np
from rclpy.clock import Clock
import asyncio
import tf2_geometry_msgs


class CameraLocalizer(Node):
    """
    Localizes the ceiling mounted d435i camera to the robot
    """

    def __init__(self):
        super().__init__('camera_localizer')

        # create transform listener and buffer
        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.static_transform_broadcaster = StaticTransformBroadcaster(
            self)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tags = ['camera_localizer_tag', 'kettle_tag',
                     'filter_tag', 'pour_over_tag', 'coffee_grounds', 'coffee_scoop']

        self.filters = [FilterTag(self.transform_broadcaster,
                                  self.buffer,
                                  self.get_clock(),
                                  "d435i_color_optical_frame",
                                  "camera_localizer_tag")]

        self.filters.extend([FilterTag(self.transform_broadcaster,
                                       self.buffer,
                                       self.get_clock(),
                                       "filtered_camera_localizer_tag",
                                       tag,
                                       predict_up=True) for tag in self.tags[1:]])

    async def timer_callback(self):
        self.transform_broadcaster.sendTransform(
            # publish d405 to franka
            TransformStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id="panda_hand"),
                child_frame_id="d405_link",
                transform=Transform(
                    translation=Vector3(
                        x=0.04,
                        y=0.0,
                        z=0.05
                    ),
                    rotation=Quaternion(
                        x=0.706825,
                        y=-0.0005629,
                        z=0.707388,
                        w=0.0005633
                    )
                )
            )
        )

        try:
            # find transform to localizer tag
            localizer_tag_to_franka_tf = TransformStamped()
            localizer_tag_to_franka_tf.header = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='filtered_camera_localizer_tag'
            )

            localizer_tag_to_franka_tf.transform = Transform(
                translation=Vector3(
                    x=0.555-0.200/2,
                    y=0.302-0.200/2,
                ),
                rotation=Quaternion(
                    x=0.0,
                    y=0.0,
                    z=0.9999997,
                    w=0.0007963
                )
            )

            localizer_tag_to_franka_tf.child_frame_id = "panda_link0"
            self.transform_broadcaster.sendTransform(
                localizer_tag_to_franka_tf)
            
            # # find transform from filter_handle to filter tag
            # localizer_filter_to_handle_tf = TransformStamped()
            # localizer_filter_to_handle_tf.header = Header(
            #     stamp=self.get_clock().now().to_msg(), frame_id="filter_tag"
            # )

            # localizer_filter_to_handle_tf.transform = Transform(
            #     translation=Vector3(
            #         x=0.15,
            #         y=0.03,
            #     ),
            #     rotation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0),
            # )

            # localizer_filter_to_handle_tf.child_frame_id = "filter_handle"
            # self.transform_broadcaster.sendTransform(localizer_filter_to_handle_tf)

            for filter in self.filters:
                filter.filter()

            # find transform from filter_handle to filter tag
            localizer_tag_to_pour_tf = TransformStamped()
            localizer_tag_to_pour_tf.header = Header(
                stamp=self.get_clock().now().to_msg(), frame_id="pour_over_tag"
            )

            localizer_tag_to_pour_tf.transform = Transform(
                translation=Vector3(
                    x= 0.15,
                    y= 0.03
                ),
                rotation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0),
            )

            localizer_tag_to_pour_tf.child_frame_id = "bottle_handle"
            self.transform_broadcaster.sendTransform(localizer_tag_to_pour_tf)

        except Exception as e:
            self.get_logger().warn(
                f"Exception: {e}")


class FilterTag:

    def __init__(self,
                 tf_broadcaster,
                 tf_buffer: Buffer,
                 clock,
                 target_frame,
                 source_frame,
                 predict_up=False):
        self.mean = None
        self.sigma = np.identity(7) * 5.0
        self.tf_broadcaster = tf_broadcaster
        self.tf_buffer = tf_buffer
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.clock: Clock = clock
        self.predict_up = predict_up

    def filter(self):
        now = self.clock.now()
        s, _ = now.seconds_nanoseconds()
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame,
                                                 self.source_frame,
                                                 Time(
                                                     seconds=s - 1),
                                                 Duration(seconds=int(1.0/6.0)))
        except Exception:
            tf = None

        if tf is not None:
            vec = self.tf_to_vec(tf)
            if self.mean is None:
                self.mean = vec
            else:
                self.mean, self.sigma = self.kalman_filter(
                    self.mean, self.sigma, vec)

        if self.mean is not None:
            filtered_tag = TransformStamped()
            filtered_tag.header = Header(
                stamp=self.clock.now().to_msg(),
                frame_id=self.target_frame
            )
            filtered_tag.child_frame_id = "filtered_" + self.source_frame
            filtered_tag.transform = self.vec_to_tf(self.mean)
            self.tf_broadcaster.sendTransform(filtered_tag)

    def kalman_filter(self, mean, sigma, zt):
        # for the state transition it shouldn't move
        A = np.identity(7)
        R = np.identity(7) * 0.01

        # measurement prediction is identity, the tag shouldn't be moving
        mean_prediction = A@mean

        if self.predict_up:
            mean_prediction[3] = 0.0
            mean_prediction[4] = 0.0

        # measurement noise
        Q = np.array([
            [0.05, 0, 0, 0, 0, 0, 0],
            [0, 0.05, 0, 0, 0, 0, 0],
            [0, 0, 2.0, 0, 0, 0, 0],
            [0, 0, 0, 10.0, 0, 0, 0],
            [0, 0, 0, 0, 10.0, 0, 0],
            [0, 0, 0, 0, 0, 10.0, 0],
            [0, 0, 0, 0, 0, 0, 10.0]
        ])
        C = np.identity(7)

        # kalman filter equations
        sigma_t = A @ sigma @ A.T + R
        K = sigma_t @ C.T @ np.linalg.inv(C @ sigma_t @ C.T + Q)
        u_t = mean_prediction + K @ (zt - C @ mean_prediction)
        sigma_t = (np.identity(7) - K @ C) @ sigma_t
        return u_t, sigma_t

    def tf_to_vec(self, tf: TransformStamped):
        return np.array([[
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        ]]).T

    def vec_to_tf(self, vec):
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
