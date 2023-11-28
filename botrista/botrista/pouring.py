import rclpy
from rclpy.node import Node
import math
import numpy as np
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

# Object Importing
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer
from botrista_interfaces.action import PourAction
from std_srvs.srv import Empty


class Pouring(Node):
    def __init__(self):
        super().__init__(node_name="pouring")
        self.path = None
        self.cb = ReentrantCallbackGroup()
        self.get_logger().warn("Pouring started")

        # Creating Parameters
        self.declare_parameter("pour_begin_offset", 0.05,
                               ParameterDescriptor(
                                   description="Z distance to move to before pouring"))
        self.pour_height_offset = self.get_parameter("pour_begin_offset")\
            .get_parameter_value().double_value

        # Creating tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_parent_frame = "panda_link0"

        # Moveit Wrapper Object
        self.moveit = MoveItApi(self,
                                "panda_link0",
                                "panda_hand_tcp",
                                "panda_manipulator",
                                "joint_states",
                                "panda")

        # Creating action server
        self._action_server = ActionServer(self,
                                           PourAction,
                                           'pour_action',
                                           self.pour_callback,
                                           callback_group=self.cb)

    async def pour_callback(self, goal_handle):
        result = PourAction.Result()
        feedback = PourAction.Feedback()
        req = goal_handle.request

        offset = req.y_offset

        # Attempt to get point from frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_parent_frame,
                req.pour_frame,
                rclpy.time.Time())
            startVec = tf.transform.translation
            startPoint = Point(x=startVec.x, y=startVec.y -
                               offset, z=startVec.z)
        except TransformException:
            feedback.stage = f"Failed to get transform to {req.pour_frame}"
            goal_handle.publish_feedback(feedback)
            result.status = False
            return result

        currPose = await self.moveit.get_end_effector_pose()
        startOre = currPose.pose.orientation

        # Calculating path
        feedback.stage = "Calculating path"
        goal_handle.publish_feedback(feedback)
        waypoints = get_spiral_waypoints(startPoint,
                                         startOre,
                                         req.tilt_ang,
                                         req.num_points,
                                         req.spiral_radius,
                                         req.num_loops,
                                         offset,
                                         req.start_outside)
        standoff_point = startPoint
        standoff_point.z += self.pour_height_offset
        standoffPose = Pose(position=standoff_point,
                            orientation=startOre)
        waypoints.append(standoffPose)
        waypoints.insert(0, standoffPose)

        # Planning
        feedback.stage = "Planning path"
        goal_handle.publish_feedback(feedback)
        planned_traj = await self.moveit.create_cartesian_path(waypoints)

        # Executing path
        feedback.stage = "Executing path"
        goal_handle.publish_feedback(feedback)
        self.moveit.execute_trajectory(planned_traj.trajectory)

        goal_handle.succeed()
        result.status = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Pouring()
    rclpy.spin(node)
    rclpy.shutdown()


def get_spiral_waypoints(startPoint: Point,
                         startOre: Quaternion,
                         tiltOre: Quaternion,
                         numPoints: int,
                         maxRadius: float,
                         loops: float,
                         offset: float,
                         flipStart: bool = False) -> (list[Pose], float):
    """
    Create a spiral path given parameters

    Arguments:
        startPoint (geometry_msgs/Point) -- Starting point
        startOre (geometry_msgs/Quaternion) -- Starting Orientation
        tiltOre (geometry_msgs/Quaternion) -- Orientation to tilt at
        numPoints (int) -- number of points used to build the path
        maxRadius (float) -- distance from end of spiral to origin in cm
        loops (float) -- number of loops for the spiral to go through
        offset (float) -- offset distance to pour at

    Keyword Arguments:
        flipStart (bool) -- Start at the end of the spiral instead of the center (default: {False})

    Returns
    -------
        A list of waypoints

    """
    count = 0
    thTotal = loops*2*math.pi
    thStep = thTotal/numPoints
    b = maxRadius/2/math.pi/loops
    tilt_ang = angle_between_quaternions(startOre, tiltOre)

    # Create poses for each point along the spiral
    poseList = []
    while count < numPoints:
        # Calculating new point
        th = count*thStep
        r = th*b
        x = r*math.cos(th)
        y = r*math.sin(th)
        z = 0

        # Transform by offset
        x_n = x + startPoint.x
        y_n = y + startPoint.y
        z_n = z + startPoint.z + offset*math.sin(tilt_ang)

        poseList.append(Pose(position=Point(x=x_n,
                                            y=y_n,
                                            z=z_n),
                             orientation=tiltOre))

        count += 1

    if flipStart:
        poseList.reverse

    return poseList


def angle_between_quaternions(q0, q1):
    """
    Get the angle between 2 quaternions, assuming rotation about a single axis

    Arguments:
        q0 (geometry_msgs/Quaternion) -- The first quaternion
        q1 (geometry_msgs/Quaternion) -- The second quaternion

    Returns
    -------
        The angle in radians
    """
    # Extract the values from q0
    w0 = q0.w
    x0 = q0.x
    y0 = q0.y
    z0 = q0.z

    # Extract the values from q1
    w1 = -q1.w  # negative to get inverse
    x1 = q1.x
    y1 = q1.y
    z1 = q1.z

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    r, p, y = euler_from_quaternion(q0q1_x, q0q1_y, q0q1_z, q0q1_w)

    # we only care about roll because we rotate about the x-axis
    return abs(r)


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into roll, pitch, yaw

    Arguments:
        x (float) -- x value of quaternion
        y (float) -- y value of quaternion
        z (float) -- z value of quaternion
        w (float) -- w value of quaternion

    Returns
    -------
        Roll, pitch, yaw
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z
