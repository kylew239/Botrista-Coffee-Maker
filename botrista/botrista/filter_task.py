import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Transform
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy import Future
from rclpy.duration import Duration

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
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.point = Pose()
        self.q = Quaternion()

    def timer_callback(self):
        try:
            # create transform listener and buffer
            to_frame_rel = 'panda_hand_tcp'
            from_frame_rel = 'filter_handle'
            if self.tf_buffer.can_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time()) == 1:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())

                self.point = t.transform.translation
                print(self.point.y)
                self.q = t.transform.rotation
        except Exception as e:
            pass


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


def main(args=None):
    rclpy.init(args=args)
    res = place_filter()
    rclpy.spin(res)
    # fut = res.move_to_pick()
    # rclpy.spin_until_future_complete(res, fut)    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
