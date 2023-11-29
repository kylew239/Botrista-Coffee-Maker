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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup

class grasp_filter(Node):
    def __init__(self):
        super().__init__("grasp_filter")
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
            joint_state_topic="joint_states",
        )
        self.grasp_planner = GraspPlanner(
            self.go_position, "panda_gripper/grasp")

        self.srv = self.create_service(
            Empty, "grab", self.grab_filter, callback_group=ReentrantCallbackGroup())
        
    async def grab_filter(self, request, response):
        approach_point = Point()
        approach_q = Quaternion()
        approach_pose = Pose(approach_point, approach_q)
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

            approach_point.x = t.transform.translation.x
            approach_point.y = t.transform.translation.y
            approach_point.z = t.transform.translation.z + 0.22

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
            approach_q = t.transform.rotation

        grasp_point = approach_point
        grasp_point.z = approach_point - 0.1
        grasp_q = approach_q
        grasp_pose = Pose(grasp_point, grasp_q)

        retreat_point = approach_point
        retreat_point.z = approach_point + 0.2
        retreat_q = approach_q
        retreat_pose = Pose(retreat_point, retreat_q)

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
    res = grasp_filter()
    rclpy.spin(res) 
    rclpy.shutdown()


if __name__ == "__main__":
    main()