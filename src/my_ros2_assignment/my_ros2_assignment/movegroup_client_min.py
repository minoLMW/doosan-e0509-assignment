import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


class MoveGroupClientMin(Node):
    def __init__(self):
        super().__init__("movegroup_client_min")
        self._ac = ActionClient(self, MoveGroup, "/move_action")

    def send_xyz_goal(self, x: float, y: float, z: float):
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server(/move_action) not available.")
            return

        goal = MoveGroup.Goal()

        # MotionPlanRequest 구성
        req = MotionPlanRequest()
        req.group_name = "manipulator"        # ✅ SRDF에서 확인됨
        req.num_planning_attempts = 5
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        # 목표 Pose (orientation은 일단 단순화: identity)
        # 과제에서는 "현재 orientation 유지"가 목표라서
        # 다음 단계에서 TF로 현재 EE orientation을 읽어서 그대로 넣을 거야.
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.w = 1.0

        # PositionConstraint (박스 형태로 목표 지점에 구속)
        pc = PositionConstraint()
        pc.header = target.header
        pc.link_name = "link_6"  # ✅ tip link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]  # 매우 작은 허용오차

        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(target.pose)
        pc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        req.goal_constraints.append(c)

        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        self.get_logger().info(f"Sending goal: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        send_future = self._ac.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return

        self.get_logger().info("Goal accepted. Waiting result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        res = future.result().result
        code = res.error_code.val
        self.get_logger().info(f"Result received. error_code={code}")
        # 1 == SUCCESS in MoveIt error codes (대개)
        # 실패하면 code 값으로 판단 가능
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MoveGroupClientMin()

    # 테스트용 좌표 (너 환경에서 reachable로 조정 가능)
    node.send_xyz_goal(0.3, 0.0, 0.4)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
