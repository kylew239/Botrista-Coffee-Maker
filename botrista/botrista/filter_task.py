import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Transform
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum, auto
from franka_msgs.action import (
    Grasp,
)
from rclpy import Future
from rclpy.duration import Duration

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto(),
    GRASP = auto(),
    PLACING = auto(),
    OPENING = auto()

class place_filter(Node):
    def __init__(self):
        super().__init__("place_filter")
        self.declare_parameter(
            "frequency", 100.0, ParameterDescriptor(description="The frequency is 100")
        )
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.goal_pose = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.state = State.MOVING
        # create transform listener and buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        self.transform_broadcaster = TransformBroadcaster(self)

        base_frame = "panda_link0"
        end_effector_frame = "panda_hand_tcp"
        group_name = "panda_manipulator"
        self.go_position = MoveItApi(
            self,
            base_frame=base_frame,
            end_effector_frame=end_effector_frame,
            group_name=group_name,
            joint_state_topic="joint_states",
        )
        my_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=my_callback_group)
        self.point = Point()
        self.final_point = Point()
        self.q = Quaternion()
        # Create Grasp.action client
        self.grasp_action_client = ActionClient(self, Grasp, "panda_gripper/grasp", callback_group=my_callback_group)

    async def timer_callback(self):
        if self.state == State.MOVING:
            try:
                # create transform listener and buffer
                # get the position related to the robot
                to_frame_rel = 'panda_link0'
                from_frame_rel = 'filter_handle'
                if self.tf_buffer.can_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time()) == 1:
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())

                    self.point.x = t.transform.translation.x
                    self.point.y = t.transform.translation.y
                    self.point.z = t.transform.translation.z + 0.12

                # create transform listener and buffer
                # get the rotation related to the e-e
                to_frame_rel2 = 'panda_hand_tcp'
                from_frame_rel2 = 'filter_handle'
                if self.tf_buffer.can_transform(
                    to_frame_rel2,
                    from_frame_rel2,
                    rclpy.time.Time()) == 1:
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel2,
                        from_frame_rel2,
                        rclpy.time.Time())
                    self.q = t.transform.rotation

                    self.fut = await self.move_to_pick()
                    self.fut.add_done_callback(self.move_callback)

            except Exception as e:
                pass

        if self.state == State.PLACING:
            try:
                # create transform listener and buffer
                # get the position related to the robot
                to_frame_rel = 'panda_link0'
                from_frame_rel = 'bottle_handle'
                if self.tf_buffer.can_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time()) == 1:
                    t2 = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())

                    self.final_point.x = t2.transform.translation.x
                    self.final_point.y = t2.transform.translation.y
                    self.final_point.z = t2.transform.translation.z + 0.2

                # create transform listener and buffer
                # get the rotation related to the e-e
                to_frame_rel2 = 'panda_hand_tcp'
                from_frame_rel2 = 'bottle_handle'
                if self.tf_buffer.can_transform(
                    to_frame_rel2,
                    from_frame_rel2,
                    rclpy.time.Time()) == 1:
                    t2 = self.tf_buffer.lookup_transform(
                        to_frame_rel2,
                        from_frame_rel2,
                        rclpy.time.Time())
                    self.q = t2.transform.rotation
                    print(self.q)
                    print(self.final_point)

                    self.fut = await self.move_to_drop()
                    self.fut.add_done_callback(self.open_callback)

            except Exception as e:
                pass

        if self.state == State.GRASP:
            try:
                self.fut = await self.control_gripper()
                self.fut.add_done_callback(self.drop_callback)

            except Exception as e:
                pass

        if self.state == State.OPENING:
            try:
                self.fut = await self.control_gripper()
                self.fut.add_done_callback(self.open_callback)

            except Exception as e:
                pass

    def done(self, plan_result):
        # self.get_logger().warning('work')

        result = plan_result.result()
        result_future = result.get_result_async()
        result_future.add_done_callback(self.move_callback)

    def move_callback(self, fut):
        # self.get_logger().warning('work')
        self.state = State.GRASP

    def grasp_callback(self, fut):
        self.get_logger().warning('work')
        self.state = State.PLACING

    def drop_callback(self, fut):
        self.state = State.OPENING

    def open_callback(self,fut):
        # self.get_logger().warning('work')
        self.state = State.STOPPED

    def move_to_pick(self):
        """
        The franka robot arm will move to the target position
        """
        return self.go_position.plan(point=self.point, orientation=self.q, execute=True)

    def move_to_drop(self):
        """
        The franka robot arm will rotate 90 degree to the drop position
        """
        return self.go_position.plan(
            point=self.final_point, orientation=self.q, execute=True
        )
    
    async def control_gripper(self):
        """
        Controling the gripper open and close
        """
        goal = Grasp.Goal()
        if self.state == State.GRASP:
            goal.width = 0.02
        else:
            goal.width = 0.04
        goal.force = 50.0
        goal.speed = 0.01

        # gripper_move_future = self.grasp_action_client.send_goal_async(goal)

        # while gripper_move_future.done():
        #     goal_handle = gripper_move_future.result()
        #     result_future = goal_handle.get_result_async()
        #     return result_future.result()

        gripper_move_future = self.grasp_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gripper_move_future)
        goal_handle = gripper_move_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

def main(args=None):
    rclpy.init(args=args)
    res = place_filter()
    rclpy.spin(res) 
    rclpy.shutdown()


if __name__ == "__main__":
    main()
