import rclpy
from rclpy.node import Node
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from moveit_wrapper.moveitapi import MoveItApi
from moveit_wrapper.grasp_planner import GraspPlan, GraspPlanner
from geometry_msgs.msg import Pose, Point, Quaternion
from control_msgs.msg import GripperCommand
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from franka_msgs.action import Grasp
from rclpy.time import Time

class filter_grasp(Node):
    def __init__(self):
        super().__init__("filter_grasp")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        base_frame = "panda_link0"
        end_effector_frame = "panda_hand_tcp"
        group_name = "panda_manipulator"
        self.go_position = MoveItApi(
            self,
            base_frame=base_frame,
            end_effector_frame=end_effector_frame,
            group_name=group_name,
            joint_state_topic="/franka/joint_states",
        )
        self.grasp_planner = GraspPlanner(
            self.go_position, "panda_gripper/grasp")

        self.srv = self.create_service(
            Empty, "grab_filter", self.grab_filter, callback_group=ReentrantCallbackGroup())
        
        # measured poses
        self.approach_pose = Pose(
            position=Point(x=0.006, y=0.05, z=0.536387),
            orientation=Quaternion(x=0.99683, y=-0.070782, z=0.003827, w=0.0360623)
        )
        self.grasp_pose = Pose(
            position=Point(x=-0.0522152, y=0.214896, z=0.436387),
            orientation=Quaternion(x=0.99683, y=-0.070782, z=0.003827, w=0.0360623)
        )
        self.retreat_pose = Pose(
            position=Point(x=-0.0522152, y=0.214896, z=0.586387),
            orientation=Quaternion(x=0.99683, y=-0.070782, z=0.003827, w=0.0360623)
        )

    async def grab_filter(self, request, response):
        tf = self.tf_buffer.lookup_transform(
            "panda_link0", "filtered_filter_tag", Time())

        approach_pose = tf2_geometry_msgs.do_transform_pose(
            self.approach_pose, tf)
        grasp_pose = tf2_geometry_msgs.do_transform_pose(self.grasp_pose, tf)
        retreat_pose = tf2_geometry_msgs.do_transform_pose(
            self.retreat_pose, tf)
        
        grasp_plan = GraspPlan(
            approach_pose=approach_pose,
            grasp_pose=grasp_pose,
            grasp_command=Grasp.Goal(
                width=0.02,
                force=50.0,
                speed=0.05,
            ),
            retreat_pose=retreat_pose,
        )

        await self.grasp_planner.execute_grasp_plan(grasp_plan)

        return response
    
def main(args=None):
    rclpy.init(args=args)
    res = filter_grasp()
    rclpy.spin(res) 
    rclpy.shutdown()


if __name__ == "__main__":
    main()