"""
Node which detects if there is a cup and starts the entire the coffee making process.

Listens to the tf_listener and broadcasts a transform for the cups top.

Uses the wrapper class made in moveitapi. Moveit moves the robot to various points
and controls the gripper as well.

Parameters
----------
  + lower_mask (Integer List) - lower mask used to detect the cup.
  + upper_mask (Integer List) - higher mask used to detect the cup.

Subscriptions
-------------
  + image_rect_color (sensor_msgs/Image) - images published by the d435 camera.
  + camera_info (sensor_msgs/CameraInfo) - intrinsic info about the d435 camera.
  + restart_coffee (std_msgs/Empty) - resets the state machine to look for the cup to
    start coffee making process again

Publishes
---------
  + cup_image (sensor_msgs/Image) - image after processing showing the cup location.
  + coffee_start (std_msgs/Empty) - sends a message to the topic to start making coffee.

Client
------
  + delay (botrista_interfaces/DelayTime) - timer for delay in seconds.

"""

from rclpy.node import Node
import rclpy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Empty
from botrista_interfaces.srv import DelayTime
import time
from enum import Enum, auto
from image_geometry.cameramodels import PinholeCameraModel


class State(Enum):
    """
    State machine for the cup dectection.

    Once the cup is detected it should just publish a tf until it is reset.

    """

    START = auto()
    STOPPED = auto()
    CUP = auto()


class CupDetection(Node):
    """
    Detects the whether there is a cup and its location.

    Begins the coffee making process.

    """

    def __init__(self):
        super().__init__("cup_detection")
        # Parameters intializing
        self.declare_parameter(
            "lower_mask",
            [0, 0, 0],
            ParameterDescriptor(
                description="Lower limit for cv mask. \
                    Default:[100,25,25]"
            ),
        )
        self.declare_parameter(
            "upper_mask",
            [180, 255, 255],
            ParameterDescriptor(
                description="Upper limit for cv mask. \
                Default:[179,255,255]"
            ),
        )
        self.lower_m = (
            self.get_parameter("lower_mask").get_parameter_value().integer_array_value
        )
        self.upper_m = (
            self.get_parameter("upper_mask").get_parameter_value().integer_array_value
        )

        # Initializing variables
        self.cup_x = 0
        self.cup_y = 0
        self.cup_z = 0
        self.kernel = np.ones((5, 5), np.uint8)
        self.state = State.STOPPED

        # TF listener and broadcaster initializing
        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self, qos=10)
        self.transform_broadcaster = TransformBroadcaster(self, qos=10)
        self.cup_tf = TransformStamped()

        # CV Bridge initializing
        self.cv_bridge = CvBridge()
        self.cam = PinholeCameraModel()

        # Subscriber initializing
        self.img_sub = self.create_subscription(
            Image, "image_rect_color", self.img_callback, 10
        )
        self.start_sub = self.create_subscription(
            Empty, "restart_coffee", self.start_callback, 10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, "camera_info", self.cam_info_callback, 10
        )

        # Publishing the Image
        self.depth_publisher = self.create_publisher(
            Image, "/cup_image", qos_profile=10
        )
        self.coffee_publisher = self.create_publisher(
            Empty, "coffee_start", qos_profile=10
        )

        # Service Client intializing
        self.delay_client = self.create_client(
            DelayTime, "delay", callback_group=ReentrantCallbackGroup()
        )
        while not self.delay_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for delay service")

        # Timer intializing
        self.timer = self.create_timer(0.1, self.timer_callback)

    async def timer_callback(self):
        """
        Check the state of the node.

        Publishes the cup_location tf.

        """
        if self.state == State.STOPPED:
            await self.delay_client.call_async(DelayTime.Request(time=5.0))
            self.state = State.START
        if self.state == State.CUP:
            self.cup_tf.header.stamp = self.get_clock().now().to_msg()
            self.cup_tf.header.frame_id = "filtered_camera_localizer_tag"
            self.cup_tf.child_frame_id = "cup_location"
            # Converting the pixel of the center to a TF
            self.cup_tf.transform.translation = Vector3(
                x=(-self.cup_x / 2) + 0.015, y=(-self.cup_y) + 0.045, z=0.115
            )
            self.depth_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.cv_im))
            self.transform_broadcaster.sendTransform(self.cup_tf)
            self.coffee_publisher.publish(Empty())

    def start_callback(self, msg: Empty):
        """
        Start the coffee making process.

        Keyword Arguments:
            msg (geometry_msgs/Empty)-- Empty message to start the state.

        Returns
        -------
            Empty return

        """
        self.state = State.START

    def cam_info_callback(self, cam_info: CameraInfo):
        """
        Set the camera info.

        Keyword Arguments:
            + cam_info (sensor_msgs/CameraInfo)-- Intrinsic points of the camera.

        Returns
        -------
            + Empty return

        """
        self.cam.fromCameraInfo(cam_info)

    def img_callback(self, Image):
        """
        Process the image and gets the pixel to tf of the center of the cup.

        Once the tf is found, changes state.

        Keyword Arguments:
            + Image (sensor_msgs/Image)-- Image of the camera.

        Returns
        -------
            + Empty return

        """
        try:
            # Getting the tf from cam to filtered_camera_localizer on the table.
            table = self.buffer.lookup_transform(
                self.cam.tf_frame, "filtered_camera_localizer_tag", rclpy.time.Time()
            )
            # Converting that tf to a pixel point.
            point = self.cam.project3dToPixel(
                (
                    table.transform.translation.x,
                    table.transform.translation.y,
                    table.transform.translation.z,
                )
            )
            pixel_tf = (int(point[0]), int(point[1]))
            if self.state == State.START:
                # Converting the image to a cv image
                cv_im = self.cv_bridge.imgmsg_to_cv2(Image, "bgr8")
                roi_im = cv_im[
                    pixel_tf[1] - 350: pixel_tf[1] - 100,
                    pixel_tf[0] - 100: pixel_tf[0] + 100,
                    :,
                ]
                lower_bound = np.asarray(
                    [self.lower_m[0], self.lower_m[1], self.lower_m[2]]
                )
                upper_bound = np.asarray(
                    [self.upper_m[0], self.upper_m[1], self.upper_m[2]]
                )
                # Clearing the image and running a color mask through it.
                median = cv2.medianBlur(roi_im, 5)
                hsv_image = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
                result = cv2.bitwise_and(roi_im, roi_im, mask=mask)
                result_gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                # Extracting circles from the image.
                circles = cv2.HoughCircles(
                    result_gray,
                    cv2.HOUGH_GRADIENT,
                    1,
                    20,
                    param1=50,
                    param2=30,
                    minRadius=30,
                    maxRadius=50,
                )
                self.cv_im = cv_im.copy()
                if circles is not None:
                    # Looking for the largest circle which is the cup.
                    circles = np.uint16(np.around(circles))
                    circles2 = sorted(circles[0], key=lambda x: x[2], reverse=True)
                    self.get_logger().info("Cup!")
                    i = circles2[0]
                    # Saving the center of the circle and publishing an image with the center
                    (self.cup_x, self.cup_y, self.cup_z) = self.cam.projectPixelTo3dRay(
                        (i[0] + (pixel_tf[0] - 100), i[1] + (pixel_tf[1] - 350))
                    )
                    cv2.circle(
                        self.cv_im,
                        (i[0] + (pixel_tf[0] - 100), i[1] + (pixel_tf[1] - 350)),
                        7,
                        (0, 0, 255),
                        -1,
                    )
                    cv2.circle(
                        self.cv_im,
                        (i[0] + (pixel_tf[0] - 100), i[1] + (pixel_tf[1] - 350)),
                        i[2],
                        (0, 255, 0),
                        2,
                    )
                    self.depth_publisher.publish(
                        self.cv_bridge.cv2_to_imgmsg(self.cv_im)
                    )
                    self.state = State.CUP
                if circles is None:
                    pass
        except Exception as e:
            self.get_logger().warn(f"Exception: {e}")


def cup_detection_entry(args=None):
    rclpy.init(args=args)
    time.sleep(2)
    cup_detection = CupDetection()
    rclpy.spin(cup_detection)
    rclpy.shutdown()
