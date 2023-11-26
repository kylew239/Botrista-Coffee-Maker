from rclpy.node import Node
import rclpy
from rclpy.time import Time
from std_msgs.msg import Header
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import numpy as np
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
        self.transform_listener = TransformListener(self.buffer, self, qos=10)
        self.transform_broadcaster = TransformBroadcaster(self, qos=10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tags = ['camera_localizer_tag', 'kettle_tag',
                     'filter_tag', 'pour_over_tag']
        self.filters = [FilterTag(self.transform_broadcaster,
                                  self.buffer,
                                  self.get_clock(),
                                  "d435i_color_optical_frame",
                                  tag)
                        for tag in self.tags]

    async def timer_callback(self):
        try:
            # find transform to localizer tag
            localizer_tag_to_franka_tf = TransformStamped()
            localizer_tag_to_franka_tf.header = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='filtered_camera_localizer_tag'
            )

            localizer_tag_to_franka_tf.transform = Transform(
                translation=Vector3(
                    x=0.555-0.2159/2,
                    y=0.302-0.2159/2,
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

            for filter in self.filters:
                await filter.filter()

        except Exception as e:
            self.get_logger().warn(
                f"Exception: {e}")


class FilterTag:

    def __init__(self,
                 tf_broadcaster,
                 tf_buffer: Buffer,
                 clock,
                 target_frame,
                 source_frame):
        self.mean = None
        self.sigma = np.identity(7) * 5.0
        self.tf_broadcaster = tf_broadcaster
        self.tf_buffer = tf_buffer
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.clock = clock

    async def filter(self):
        try:
            tf = await self.tf_buffer.lookup_transform_async(self.target_frame,
                                                             self.source_frame,
                                                             Time(seconds=0.0))
        except Exception as e:
            return

        vec = self.tf_to_vec(tf)
        if self.mean is None:
            self.mean = vec
        else:
            self.mean, self.sigma = self.kalman_filter(
                self.mean, self.sigma, vec)

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
        R = np.identity(7) * 0.05

        # measurement prediction is identity, the tag shouldn't be moving
        mean_prediction = A@mean

        # measurement noise
        Q = np.array([
            [0.05, 0, 0, 0, 0, 0, 0],
            [0, 0.05, 0, 0, 0, 0, 0],
            [0, 0, 2.0, 0, 0, 0, 0],
            [0, 0, 0, 5.0, 0, 0, 0],
            [0, 0, 0, 0, 5.0, 0, 0],
            [0, 0, 0, 0, 0, 5.0, 0],
            [0, 0, 0, 0, 0, 0, 5.0]
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
