from rclpy.node import Node
import rclpy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from std_msgs.msg import Empty
import time
from enum import Enum, auto
from image_geometry.cameramodels import PinholeCameraModel
# cup height 115mm


class State(Enum):
    START = auto()
    STOPPED = auto()
    CUP = auto()


class CupDetection(Node):
    """
    Localizes the ceiling mounted d435i camera to the robot
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
            self.get_parameter(
                "lower_mask").get_parameter_value().integer_array_value
        )
        self.upper_m = (
            self.get_parameter(
                "upper_mask").get_parameter_value().integer_array_value
        )
        # Initialzing variables
        self.cup_x = 0
        self.cup_y = 0
        self.cup_z = 0
        self.kernel = np.ones((5, 5), np.uint8)
        self.state = State.START
        self.cam = PinholeCameraModel()
        # TF listener and broadcaster intializing
        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self, qos=10)
        self.transform_broadcaster = TransformBroadcaster(self, qos=10)
        self.cup_tf = TransformStamped()
        # CV Bridge intializing
        self.cv_bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image, "image_rect_color", self.img_callback, 10
        )
        # Subscriber intializing
        self.start_sub = self.create_subscription(
            Empty, "start_coffee", self.start_callback, 10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, "camera_info", self.cam_info_callback, 10
        )
        # Timer intializing
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.state == State.CUP:
            self.cup_tf.header.stamp = self.get_clock().now().to_msg()
            self.cup_tf.header.frame_id = "filtered_camera_localizer_tag"
            self.cup_tf.child_frame_id = "cup_location"
            self.cup_tf.transform.translation = Vector3(
                x=self.cup_x/2,
                y=-self.cup_y,
                z=0.115)
            self.get_logger().info(str(self.cup_tf.transform.translation))
            self.transform_broadcaster.sendTransform(self.cup_tf)

    def start_callback(self, msg: Empty):
        self.state = State.START

    def cam_info_callback(self, cam_info: CameraInfo):
        self.cam.fromCameraInfo(cam_info)

    def img_callback(self, Image):
        table = self.buffer.lookup_transform(
            self.cam.tf_frame, "filtered_camera_localizer_tag", rclpy.time.Time()
        )
        point = self.cam.project3dToPixel(
            (
                table.transform.translation.x,
                table.transform.translation.y,
                table.transform.translation.z
            )
        )
        pixel_tf = (int(point[0]), int(point[1]))
        if self.state == State.START:
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
            median = cv2.medianBlur(roi_im, 5)
            hsv_image = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            result = cv2.bitwise_and(roi_im, roi_im, mask=mask)
            result_gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
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
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    self.get_logger().info("Cup!")
                    (self.cup_x, self.cup_y, self.cup_z) = self.cam.projectPixelTo3dRay(
                        (i[0]+(pixel_tf[0]-100),
                         i[1]+(pixel_tf[1]-350)))
                    self.state = State.CUP
            if circles is None:
                self.get_logger().info("No Cup :(")


def cup_detection_entry(args=None):
    rclpy.init(args=args)
    time.sleep(5)
    cup_detection = CupDetection()
    rclpy.spin(cup_detection)
    rclpy.shutdown()
