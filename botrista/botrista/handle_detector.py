import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, TransformStamped, Transform
import tf2_geometry_msgs
from image_geometry.cameramodels import PinholeCameraModel
from scipy.ndimage import median_filter
import cv_bridge
import cv2
import numpy as np
import scipy.stats as ss
import pyrealsense2
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener, Buffer
from rclpy.time import Time
from std_msgs.msg import Header


class HandleDetector(Node):
    """
    Detects the largest handle and publishes the transform to the handle in tf.

    Subscriptions:
        + /camera/d405/color/image_raw (sensor_msgs/Image) - the image to use 
        for handle detection.

    """

    def __init__(self):
        super().__init__("handle_detector")

        # create subscription to camera topic
        self.image_subscription = self.create_subscription(
            Image, "/camera/d405/color/image_rect_raw", self.image_callback, qos_profile=10)

        # subscription to the depth image
        self.depth_image_subscription = self.create_subscription(
            Image, "/camera/d405/aligned_depth_to_color/image_raw", self.depth_image_callback, qos_profile=10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "/camera/d405/aligned_depth_to_color/camera_info", self.camera_info_callback, qos_profile=10)

        # published the masked depth image
        self.depth_publisher = self.create_publisher(
            Image, "/depth_mask", qos_profile=10
        )

        self.cv_bridge = cv_bridge.CvBridge()

        self.camera_model = PinholeCameraModel()

        # parameters for thresholding
        self.hue = [90, 114]
        self.saturation = [75, 255]
        self.value = [104, 255]

        # camera intrinsics
        self.depth_image = None
        self.intrinsics = None

        # tf2 broadcasters and listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=10)
        self.tf_broadcaster = TransformBroadcaster(self, qos=10)
        self.transform_broadcaster = TransformBroadcaster(self, qos=10)

    def depth_image_callback(self, image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(
            image, desired_encoding="passthrough")

    def camera_info_callback(self, camera_info):
        self.intrinsics = pyrealsense2.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.k[2]
        self.intrinsics.ppy = camera_info.k[5]
        self.intrinsics.fx = camera_info.k[0]
        self.intrinsics.fy = camera_info.k[4]
        self.intrinsics.model = pyrealsense2.distortion.none
        self.intrinsics.coeffs = [i for i in camera_info.d]

    def image_callback(self, image):

        if self.depth_image is None or self.intrinsics is None:
            return

        # convert ros image msg to opencv mat
        cv_img = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        thresholded = self.threshold_image(cv_img)
        handle_contour = self.find_handle_contour(thresholded)

        if handle_contour is not None:
            pose = self.contour_to_depth(handle_contour, thresholded)

            try:
                tf = self.tf_buffer.lookup_transform("panda_link0",
                                                     "d405_depth_optical_frame",
                                                     time=Time(seconds=0.0))
                pose_panda0 = tf2_geometry_msgs.do_transform_pose(pose, tf)
                self.transform_broadcaster.sendTransform(
                    TransformStamped(
                        header=Header(
                            stamp=self.get_clock().now().to_msg(),
                            frame_id="panda_link0"
                        ),
                        child_frame_id="handle",
                        transform=Transform(
                            translation=Vector3(
                                x=pose_panda0.position.x,
                                y=pose_panda0.position.y,
                                z=pose_panda0.position.z
                            ),
                            rotation=pose_panda0.orientation
                        )
                    )
                )
            except Exception as e:
                self.get_logger().warn(f"Exception: {e}")
                pass

    def contour_to_depth(self, contour, image):

        # Get center of the contour
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # create a mask for the contour
        mask = np.zeros_like(image)
        cv2.drawContours(mask, [contour], -1, color=255, thickness=-1)

        # erode mask slightly take ensure all points taken are
        # inside the contour
        mask = cv2.erode(mask, np.ones((3, 3)))

        # get the values under the mask
        depth_mask = cv2.bitwise_and(
            self.depth_image, self.depth_image, mask=mask)

        # draw the masked values
        depth_mask_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_mask, alpha=0.03), cv2.COLORMAP_JET)
        self.depth_publisher.publish(
            self.cv_bridge.cv2_to_imgmsg(depth_mask_colormap))

        # use all non-zero values as the depth, zero values are invalid
        depth_mask_filtered = depth_mask[depth_mask != 0]

        # filter outliers
        depth_mask_filtered = median_filter(depth_mask_filtered, 1)

        # get the average depth
        depth = ss.tmean(depth_mask_filtered)

        # get the 3d point
        point = pyrealsense2.rs2_deproject_pixel_to_point(
            self.intrinsics, [cX, cY], depth)
        point = np.array(point)
        # point[1] = -point[1]
        point /= 1000.0  # mm to m

        return Pose(
            position=Point(
                x=point[0],
                y=point[1],
                z=point[2]),
            orientation=Quaternion()
        )

    def find_handle_contour(self, image):
        """
        Find the handle in the thresholded image.

        Decides which contour is the handled by finding the largest contour.

        Arguments:
            + image (numpy.ndarray) - the thresholded image.

        Returns:
            The contour of the handle.
        """

        # erode and dilate to remove noise
        eroded = cv2.erode(image, np.ones((5, 5)))
        dilate = cv2.dilate(eroded, np.ones((10, 10)))
        clean_image = dilate

        # find contours
        contours, hierarchy = cv2.findContours(
            clean_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # identify largest contour
        contours = list(contours)

        # no contours found
        if len(contours) == 0:
            return None

        largest = max(contours, key=lambda cnt: cv2.contourArea(cnt))
        return largest

    def threshold_image(self, image):
        """
        Thresholds the image to isolate the handle.

        Arguments:
            + image (numpy.ndarray) - the image to threshold.

        Returns:
            The thresholded image.
        """

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv_image,
                           (self.hue[0], self.saturation[0], self.value[0]),
                           (self.hue[1], self.saturation[1], self.value[1]))


def handle_detector_entry(args=None):
    rclpy.init(args=args)
    handle_detector = HandleDetector()
    rclpy.spin(handle_detector)
    rclpy.shutdown()
