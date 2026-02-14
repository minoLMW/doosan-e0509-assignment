#!/usr/bin/env python3
import math
import argparse
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)
from shape_msgs.msg import SolidPrimitive


@dataclass
class TargetCmd:
    mode: str  # "abs" or "rel"
    x: float
    y: float
    z: float
    vel: float  # 0~1 scaling
    acc: float  # 0~1 scaling


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


class MoveGroupClientXYZ(Node):
    """
    - /move_action (moveit_msgs/action/MoveGroup)로 goal을 보냄
    - TF로 현재 EE(link_6) pose를 읽고, orientation은 유지
    - abs/rel 좌표 처리
    - vel/acc scaling 반영
    - STOP: 현재 goal cancel 지원
    """

    def __init__(self):
        super().__init__("movegroup_client_xyz")
        self._ac = ActionClient(self, MoveGroup, "/move_action")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.group_name = "manipulator"
        self.base_frame = "base_link"
        self.ee_link = "link_6"

        # 현재 goal handle 저장(Stop에 필요)
        self._goal_handle = None

    # ---------------- TF ----------------
    def wait_tf_ready(self, total_wait_sec: float = 5.0, step_sec: float = 0.2) -> bool:
        """
        TF가 준비될 때까지 짧게 여러 번 확인
        """
        deadline = self.get_clock().now() + Duration(seconds=total_wait_sec)
        while self.get_clock().now() < deadline:
            if self.tf_buffer.can_transform(
                self.base_frame, self.ee_link, Time(), timeout=Duration(seconds=0.1)
            ):
                return True
            self.get_logger().warn(
                f"TF not ready yet ({self.base_frame} <- {self.ee_link}). Waiting..."
            )
            rclpy.spin_once(self, timeout_sec=step_sec)
        return False

    def get_current_ee_pose(self, timeout_sec: float = 2.0) -> Optional[PoseStamped]:
        """base_frame 기준 ee_link의 현재 Pose를 TF로 조회"""
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.base_frame,
                source_frame=self.ee_link,
                time=Time(),  # latest
                timeout=Duration(seconds=timeout_sec),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup failed ({self.base_frame} <- {self.ee_link}): {e}"
            )
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    # ---------------- Constraints ----------------
    def build_goal_constraints_keep_orientation(
        self,
        target_pose: PoseStamped,
        pos_tol: float = 0.001,
        orient_tol_rad: float = math.radians(3.0),
    ) -> Constraints:
        """
        PositionConstraint + OrientationConstraint로 목표 pose 구성
        - orientation은 target_pose에 넣어준 값(=현재 orientation 유지)을 그대로 강제
        """
        c = Constraints()

        # PositionConstraint (작은 박스)
        pc = PositionConstraint()
        pc.header = target_pose.header
        pc.link_name = self.ee_link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [pos_tol, pos_tol, pos_tol]
        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0

        # OrientationConstraint
        oc = OrientationConstraint()
        oc.header = target_pose.header
        oc.link_name = self.ee_link
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = orient_tol_rad
        oc.absolute_y_axis_tolerance = orient_tol_rad
        oc.absolute_z_axis_tolerance = orient_tol_rad
        oc.weight = 1.0

        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        return c

    # ---------------- STOP (Cancel) ----------------
    def stop(self):
        """
        현재 goal을 cancel 요청
        """
        if self._goal_handle is None:
            self.get_logger().error("No active goal to cancel.")
            rclpy.shutdown()
            return

        self.get_logger().warn("Sending CANCEL request to /move_action ...")
        cf = self._goal_handle.cancel_goal_async()
        cf.add_done_callback(self._cancel_done_cb)

    def _cancel_done_cb(self, future):
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().warn("Cancel accepted. Goal is canceling.")
            else:
                self.get_logger().error("Cancel rejected or nothing to cancel.")
        except Exception as e:
            self.get_logger().error(f"Cancel failed: {e}")
        finally:
            rclpy.shutdown()

    # ---------------- SEND ----------------
    def send_target(self, cmd: TargetCmd):
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server(/move_action) not available.")
            rclpy.shutdown()
            return

        # TF 준비 대기(중요)
        if not self.wait_tf_ready(total_wait_sec=5.0):
            self.get_logger().error(f"TF lookup failed after retries: {self.base_frame} <- {self.ee_link}")
            self.get_logger().error("Cannot read current EE pose.")
            rclpy.shutdown()
            return

        cur = self.get_current_ee_pose()
        if cur is None:
            self.get_logger().error("Cannot read current EE pose. (TF missing) Stop.")
            rclpy.shutdown()
            return

        # 목표 pose 계산
        target = PoseStamped()
        target.header.frame_id = self.base_frame
        target.header.stamp = self.get_clock().now().to_msg()

        if cmd.mode == "abs":
            target.pose.position.x = cmd.x
            target.pose.position.y = cmd.y
            target.pose.position.z = cmd.z
        else:
            # rel: base_frame 기준으로 현재 EE 위치에 delta를 더함
            target.pose.position.x = cur.pose.position.x + cmd.x
            target.pose.position.y = cur.pose.position.y + cmd.y
            target.pose.position.z = cur.pose.position.z + cmd.z

        # orientation 유지
        target.pose.orientation = cur.pose.orientation

        # MotionPlanRequest
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.num_planning_attempts = 5
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = clamp01(cmd.vel)
        req.max_acceleration_scaling_factor = clamp01(cmd.acc)

        # goal constraints
        req.goal_constraints.append(
            self.build_goal_constraints_keep_orientation(target_pose=target)
        )

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        self.get_logger().info(
            f"[SEND] mode={cmd.mode} "
            f"xyz=({target.pose.position.x:.3f},{target.pose.position.y:.3f},{target.pose.position.z:.3f}) "
            f"vel={req.max_velocity_scaling_factor:.2f} acc={req.max_acceleration_scaling_factor:.2f}"
        )

        f = self._ac.send_goal_async(goal)
        f.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("Goal rejected.")
            rclpy.shutdown()
            return

        self._goal_handle = gh  # <- STOP을 위해 저장
        self.get_logger().info("Goal accepted. Waiting result...")
        rf = gh.get_result_async()
        rf.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        res = future.result().result
        code = res.error_code.val
        self.get_logger().info(f"[DONE] moveit error_code={code}")
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stop", action="store_true", help="Cancel current goal (STOP)")
    parser.add_argument("--mode", choices=["abs", "rel"], default="abs")
    parser.add_argument("--x", type=float, default=0.30)
    parser.add_argument("--y", type=float, default=0.00)
    parser.add_argument("--z", type=float, default=0.40)
    parser.add_argument("--vel", type=float, default=0.20)  # 0~1
    parser.add_argument("--acc", type=float, default=0.20)  # 0~1
    args = parser.parse_args()

    rclpy.init()
    node = MoveGroupClientXYZ()

    # STOP 모드면 cancel만 수행
    if args.stop:
        node.stop()
        rclpy.spin(node)
        return

    node.send_target(TargetCmd(args.mode, args.x, args.y, args.z, args.vel, args.acc))
    rclpy.spin(node)


if __name__ == "__main__":
    main()
