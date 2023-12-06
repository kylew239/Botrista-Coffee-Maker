import pyrealsense2 as rs
import numpy as np
import cv2


def nothing(x):
    pass


"""
This file is used for tuning the settings of the realsense camera. 
"""

pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == "RGB Camera":
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
if device_product_line == "L500":
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
pipeline.start(config)
cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar("H MIN", "RealSense", 0, 179, nothing)
cv2.createTrackbar("S MIN", "RealSense", 0, 255, nothing)
cv2.createTrackbar("V MIN", "RealSense", 0, 255, nothing)
cv2.createTrackbar("H MAX", "RealSense", 179, 179, nothing)
cv2.createTrackbar("S MAX", "RealSense", 255, 255, nothing)
cv2.createTrackbar("V MAX", "RealSense", 255, 255, nothing)
try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos("H MIN", "RealSense")
        s_min = cv2.getTrackbarPos("S MIN", "RealSense")
        v_min = cv2.getTrackbarPos("V MIN", "RealSense")
        h_max = cv2.getTrackbarPos("H MAX", "RealSense")
        s_max = cv2.getTrackbarPos("S MAX", "RealSense")
        v_max = cv2.getTrackbarPos("V MAX", "RealSense")
        hsv_min = "MIN H:{} S:{} V:{}".format(h_min, s_min, v_min)
        hsv_max = "MAX H:{} S:{} V:{}".format(h_max, s_max, v_max)
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        result = cv2.bitwise_and(color_image, color_image, mask=mask)
        cv2.imshow("RealSense", result)
        key = cv2.waitKey(1)
        if key == 27:
            f = open("../config/cam_cal.yaml", "w")
            f.write("cup_detection:\n")
            f.write("  ros__parameters:\n")
            f.write(
                "    lower_mask: ["
                + str(h_min)
                + ","
                + str(s_min)
                + ","
                + str(v_min)
                + "]\n"
            )
            f.write(
                "    upper_mask: ["
                + str(h_max)
                + ","
                + str(s_max)
                + ","
                + str(v_max)
                + "]"
            )
            f.close()
            break

finally:
    pipeline.stop()
