#!/usr/bin/env python3
import math
import argparse
import uuid
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String
from std_srvs.srv import Trigger

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def error_code_to_text(val: int) -> str:
    mapping = {
        MoveItErrorCodes.SUCCESS: "SUCCESS",
        MoveItErrorCodes.FAILURE: "FAILURE",
        MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
        MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
        MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
        MoveItErrorCodes.PREEMPTED: "PREEMPTED",
        MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
        MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
        MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
        MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
        MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
        MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
    }
    return mapping.get(val, f"UNKNOWN({val})")


@dataclass
class Target:
    x: float
    y: float
    z: float


class SequenceMove(Node):
    """
    - targets를 순서대로 MoveGroup action으로 실행
    - mode=abs/rel 지원
    - 기본: position-only로 플래닝/실행 (성공률↑)
    - 옵션: keep_orientation 켜면 orientation 제약으로 1차 시도 후 실패 시 position-only로 자동 fallback
    - vel/acc scaling 반영
    - /assignment/stop (Trigger)로 언제든 취소
    - /assignment/status (String) : 사람이 보기 편한 텍스트(기존 유지)
    - /assignment/state_event (String) : status_node가 파싱하는 k=v;... 이벤트(추가)
    """

    def __init__(
        self,
        mode: str,
        targets: List[Target],
        vel: float,
        acc: float,
        keep_orientation: bool,
        plan_only: bool,
    ):
        super().__init__("assignment_sequence_move")

        self.mode = mode
        self.targets = targets
        self.vel = clamp01(vel)
        self.acc = clamp01(acc)

        self.keep_orientation = keep_orientation
        self.plan_only = plan_only

        self.group_name = "manipulator"
        self.base_frame = "base_link"
        self.ee_link = "link_6"

        self._ac = ActionClient(self, MoveGroup, "/move_action")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 기존 텍스트 상태(호환 유지)
        self.status_pub = self.create_publisher(String, "/assignment/status", 10)
        # 새 이벤트(AssignmentStatusNode가 파싱)
        self.state_event_pub = self.create_publisher(String, "/assignment/state_event", 10)

        self.stop_srv = self.create_service(Trigger, "/assignment/stop", self._on_stop)

        self._goal_handle = None
        self._send_future = None
        self._result_future = None

        self._idx = 0
        self._stopped = False
        self._busy = False

        self._last_code: Optional[int] = None
        self._last_text: str = ""
        self._phase: str = "IDLE"

        # 현재 타겟에서 orientation 시도 여부(keep_orientation가 True면 처음 1번만 True)
        self._try_orientation_this_target = False

        # 타임아웃
        self._accept_deadline = None
        self._result_deadline = None

        # goal_id 추적
        self._active_goal_id: str = ""

        self.create_timer(0.2, self._publish_status)
        self.create_timer(0.2, self._watch_timeouts)

        # 시작 약간 대기
        self.create_timer(0.5, self._kickoff_once)
        self._kicked = False

        # 시작 이벤트
        self._emit_state(
            state="IDLE",
            idx=self._idx,
            total=len(self.targets),
            mode=self.mode,
            vel=self.vel,
            acc=self.acc,
            keep_orientation=str(self.keep_orientation).lower(),
            log="Node created",
        )

    # ---------------- state event helper ----------------
    def _emit_state(self, **kwargs):
        # k=v;... 프로토콜 (status_node 파서와 호환)
        parts = [f"{k}={v}" for k, v in kwargs.items()]
        m = String()
        m.data = ";".join(parts)
        self.state_event_pub.publish(m)

    # ---------------- status (legacy text) ----------------
    def _publish_status(self):
        if self._stopped:
            state = "STOPPED"
        else:
            state = f"{self._phase} ({self._idx}/{len(self.targets)})"

        extra = ""
        if self._last_code is not None:
            extra = f" last={self._last_text}"

        msg = String()
        msg.data = state + extra
        self.status_pub.publish(msg)

    def _watch_timeouts(self):
        now = self.get_clock().now()

        if self._accept_deadline is not None and now > self._accept_deadline:
            self.get_logger().error("Timeout waiting for goal ACCEPT response (5s).")
            self._phase = "ERROR"
            self._busy = False
            self._accept_deadline = None

            self._emit_state(
                state="ERROR",
                idx=self._idx,
                total=len(self.targets),
                err=-1,
                msg="ACCEPT_TIMEOUT",
                log="Timeout waiting for goal ACCEPT (5s)",
            )
            rclpy.shutdown()
            return

        if self._result_deadline is not None and now > self._result_deadline:
            self.get_logger().error("Timeout waiting for RESULT (30s). Canceling goal.")
            self._phase = "ERROR"
            self._busy = False
            self._result_deadline = None
            try:
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()
            except Exception:
                pass

            self._emit_state(
                state="ERROR",
                idx=self._idx,
                total=len(self.targets),
                err=-2,
                msg="RESULT_TIMEOUT",
                goal_id=self._active_goal_id,
                log="Timeout waiting for RESULT (30s), cancel requested",
            )
            rclpy.shutdown()
            return

    # ---------------- stop service ----------------
    def _on_stop(self, request, response):
        self._stopped = True
        self.get_logger().warn("STOP requested: cancel active goal + stop sequence.")

        self._emit_state(state="STOPPING", idx=self._idx, total=len(self.targets), goal_id=self._active_goal_id, log="Cancel requested")

        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
                response.success = True
                response.message = "Cancel requested."
            except Exception as e:
                response.success = False
                response.message = f"Cancel failed: {e}"
        else:
            response.success = True
            response.message = "No active goal."
        return response

    # ---------------- TF helpers ----------------
    def _wait_tf_ready(self, total_wait_sec: float = 5.0) -> bool:
        deadline = self.get_clock().now() + Duration(seconds=total_wait_sec)
        while self.get_clock().now() < deadline:
            if self.tf_buffer.can_transform(
                self.base_frame, self.ee_link, Time(), timeout=Duration(seconds=0.2)
            ):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _get_current_ee_pose(self, timeout_sec: float = 1.0) -> Optional[PoseStamped]:
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.base_frame,
                source_frame=self.ee_link,
                time=Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except Exception:
            return None

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        return ps

    # ---------------- constraints ----------------
    def _constraints_position_only(self, target_pose: PoseStamped, pos_tol: float = 0.02) -> Constraints:
        c = Constraints()

        pc = PositionConstraint()
        pc.header = target_pose.header
        pc.link_name = self.ee_link

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [pos_tol, pos_tol, pos_tol]
        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0

        c.position_constraints.append(pc)
        return c

    def _constraints_keep_orientation(
        self,
        target_pose: PoseStamped,
        pos_tol: float = 0.02,
        orient_tol_rad: float = math.radians(20.0),
    ) -> Constraints:
        c = Constraints()

        pc = PositionConstraint()
        pc.header = target_pose.header
        pc.link_name = self.ee_link

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [pos_tol, pos_tol, pos_tol]
        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0

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

    # ---------------- kickoff / sequencing ----------------
    def _kickoff_once(self):
        if self._kicked:
            return
        self._kicked = True

        self._phase = "WAIT_SERVER"
        self._emit_state(state="PLANNING", idx=self._idx, total=len(self.targets), mode=self.mode, vel=self.vel, acc=self.acc,
                         keep_orientation=str(self.keep_orientation).lower(), log="Wait /move_action server")

        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server(/move_action) not available.")
            self._phase = "ERROR"
            self._emit_state(state="ERROR", idx=self._idx, total=len(self.targets), err=-3, msg="MOVE_ACTION_NOT_READY", log="/move_action not available")
            rclpy.shutdown()
            return

        self._phase = "WAIT_TF"
        self._emit_state(state="PLANNING", idx=self._idx, total=len(self.targets), log=f"Wait TF {self.base_frame}<-{self.ee_link}")

        if not self._wait_tf_ready():
            self.get_logger().error(f"TF not ready: {self.base_frame} <- {self.ee_link}")
            self._phase = "ERROR"
            self._emit_state(state="ERROR", idx=self._idx, total=len(self.targets), err=-4, msg="TF_NOT_READY", log=f"TF not ready: {self.base_frame}<-{self.ee_link}")
            rclpy.shutdown()
            return

        if len(self.targets) == 0:
            self.get_logger().error("No targets provided.")
            self._phase = "ERROR"
            self._emit_state(state="ERROR", idx=0, total=0, err=-5, msg="NO_TARGETS", log="No targets provided")
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"Start sequence: mode={self.mode}, targets={len(self.targets)}, vel={self.vel}, acc={self.acc}, "
            f"keep_orientation={self.keep_orientation}, plan_only={self.plan_only}"
        )

        self._emit_state(
            state="PLANNING",
            idx=self._idx,
            total=len(self.targets),
            mode=self.mode,
            vel=self.vel,
            acc=self.acc,
            keep_orientation=str(self.keep_orientation).lower(),
            log="Start sequence",
        )

        self._send_next()

    def _build_target_pose(self) -> Optional[PoseStamped]:
        cur = self._get_current_ee_pose()
        if cur is None:
            return None

        t = self.targets[self._idx]
        target = PoseStamped()
        target.header.frame_id = self.base_frame
        target.header.stamp = self.get_clock().now().to_msg()

        if self.mode == "abs":
            target.pose.position.x = t.x
            target.pose.position.y = t.y
            target.pose.position.z = t.z
        else:
            target.pose.position.x = cur.pose.position.x + t.x
            target.pose.position.y = cur.pose.position.y + t.y
            target.pose.position.z = cur.pose.position.z + t.z

        # “자세 유지” 옵션을 켜면 현재 자세를 목표 orientation으로 사용
        target.pose.orientation = cur.pose.orientation
        return target

    def _send_next(self):
        if self._stopped:
            self.get_logger().warn("Sequence stopped. Exiting.")
            self._phase = "STOPPED"
            self._busy = False
            self._emit_state(state="IDLE", idx=self._idx, total=len(self.targets), log="Stopped")
            rclpy.shutdown()
            return

        if self._idx >= len(self.targets):
            self.get_logger().info("Sequence completed.")
            self._phase = "DONE"
            self._busy = False
            self._emit_state(state="IDLE", idx=self._idx, total=len(self.targets), err=MoveItErrorCodes.SUCCESS, msg="DONE", log="Sequence completed")
            rclpy.shutdown()
            return

        target = self._build_target_pose()
        if target is None:
            self.get_logger().error("Cannot read current EE pose (TF).")
            self._phase = "ERROR"
            self._emit_state(state="ERROR", idx=self._idx, total=len(self.targets), err=-6, msg="EE_POSE_READ_FAIL", log="Cannot read current EE pose (TF)")
            rclpy.shutdown()
            return

        # 현재 타겟에서 orientation을 1차로 시도할지 결정
        self._try_orientation_this_target = bool(self.keep_orientation)

        self._send_goal_for_current_target(target)

    def _send_goal_for_current_target(self, target: PoseStamped):
        # 각 목표마다 goal_id 생성(트래킹용)
        self._active_goal_id = str(uuid.uuid4())

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.vel
        req.max_acceleration_scaling_factor = self.acc

        req.goal_constraints.clear()

        # 1차: orientation 포함 (옵션 on일 때)
        if self._try_orientation_this_target:
            req.goal_constraints.append(
                self._constraints_keep_orientation(
                    target,
                    pos_tol=0.02,
                    orient_tol_rad=math.radians(20.0),
                )
            )
            mode_text = "pos+ori"
        else:
            req.goal_constraints.append(self._constraints_position_only(target, pos_tol=0.02))
            mode_text = "pos-only"

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = bool(self.plan_only)
        goal.planning_options.replan = False

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self._busy = True
        self._phase = "WAIT_ACCEPT"

        self.get_logger().info(
            f"[{self._idx+1}/{len(self.targets)}] SEND xyz=({target.pose.position.x:.3f},{target.pose.position.y:.3f},{target.pose.position.z:.3f}) "
            f"vel={self.vel:.2f} acc={self.acc:.2f} ({mode_text}, plan_only={self.plan_only})"
        )

        # 이벤트: EXECUTING(실제론 goal 송신 단계지만 UI에서 “동작 시작”으로 보는 게 자연스러움)
        self._emit_state(
            state="EXECUTING",
            goal_id=self._active_goal_id,
            idx=self._idx,
            total=len(self.targets),
            mode=self.mode,
            vel=self.vel,
            acc=self.acc,
            keep_orientation=str(self.keep_orientation).lower(),
            log=f"SEND goal ({mode_text}) xyz=({target.pose.position.x:.3f},{target.pose.position.y:.3f},{target.pose.position.z:.3f})",
        )

        self._accept_deadline = self.get_clock().now() + Duration(seconds=5.0)
        self._send_future = self._ac.send_goal_async(goal)
        self._send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self._accept_deadline = None

        try:
            gh = future.result()
        except Exception as e:
            self.get_logger().error(f"send_goal_async failed: {e}")
            self._phase = "ERROR"
            self._busy = False
            self._emit_state(state="ERROR", idx=self._idx, total=len(self.targets), goal_id=self._active_goal_id, err=-7, msg="SEND_GOAL_FAIL", log=f"send_goal_async failed: {e}")
            rclpy.shutdown()
            return

        self.get_logger().warn(f"Goal accepted? {gh.accepted}")

        if not gh.accepted:
            self._phase = "ERROR"
            self._busy = False
            self._emit_state(state="ERROR", idx=self._idx, total=len(self.targets), goal_id=self._active_goal_id, err=-8, msg="GOAL_REJECTED", log="Goal rejected by server")
            rclpy.shutdown()
            return

        self._goal_handle = gh
        self._phase = "WAIT_RESULT"

        self._result_deadline = self.get_clock().now() + Duration(seconds=30.0)

        self._result_future = gh.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        self._result_deadline = None

        try:
            wrapped = future.result()
            res = wrapped.result
        except Exception as e:
            self.get_logger().error(f"get_result_async failed: {e}")
            self._phase = "ERROR"
            self._busy = False
            self._emit_state(state="ERROR", idx=self._idx, total=len(self.targets), goal_id=self._active_goal_id, err=-9, msg="GET_RESULT_FAIL", log=f"get_result_async failed: {e}")
            rclpy.shutdown()
            return

        code = res.error_code.val
        text = error_code_to_text(code)
        self._last_code = code
        self._last_text = text

        self.get_logger().info(f"[{self._idx+1}/{len(self.targets)}] DONE moveit_error_code={code} ({text})")

        # --------- fallback 로직 ----------
        # orientation 포함으로 실패했다면, 같은 타겟을 pos-only로 1회 재시도
        if self._try_orientation_this_target and code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn("Fallback: retry same target with position-only constraints.")
            self._emit_state(
                state="EXECUTING",
                idx=self._idx,
                total=len(self.targets),
                goal_id=self._active_goal_id,
                err=int(code),
                msg=text,
                log="Fallback: retry same target with position-only",
            )

            self._try_orientation_this_target = False  # 다음 시도는 pos-only

            target = self._build_target_pose()
            if target is None:
                self.get_logger().error("Cannot read current EE pose (TF) for fallback.")
                self._phase = "ERROR"
                self._busy = False
                self._emit_state(state="ERROR", idx=self._idx, total=len(self.targets), goal_id=self._active_goal_id, err=-10, msg="FALLBACK_TF_FAIL", log="Cannot read EE pose for fallback")
                rclpy.shutdown()
                return

            self._goal_handle = None
            self._busy = False
            self._send_goal_for_current_target(target)
            return

        # 결과 이벤트 (성공/실패)
        if code == MoveItErrorCodes.SUCCESS:
            self._emit_state(
                state="IDLE",
                idx=self._idx,
                total=len(self.targets),
                goal_id=self._active_goal_id,
                err=int(code),
                msg=text,
                log=f"DONE idx={self._idx} ({text})",
            )
        else:
            self._emit_state(
                state="ERROR",
                idx=self._idx,
                total=len(self.targets),
                goal_id=self._active_goal_id,
                err=int(code),
                msg=text,
                log=f"FAIL idx={self._idx} ({text})",
            )

        # 성공했거나(pos-only), fallback까지 끝났으면 다음 타겟으로
        self._idx += 1
        self._goal_handle = None
        self._busy = False
        self._send_next()


def parse_targets(s: str) -> List[Target]:
    out: List[Target] = []
    for part in s.split(";"):
        part = part.strip()
        if not part:
            continue
        xyz = [p.strip() for p in part.split(",")]
        if len(xyz) != 3:
            raise ValueError(f"Invalid target format: '{part}' (need x,y,z)")
        out.append(Target(float(xyz[0]), float(xyz[1]), float(xyz[2])))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["abs", "rel"], default="abs")
    parser.add_argument("--targets", type=str, required=True, help='Example: "0.30,0,0.40; 0.25,0.10,0.35"')
    parser.add_argument("--vel", type=float, default=0.2)
    parser.add_argument("--acc", type=float, default=0.2)

    parser.add_argument(
        "--keep-orientation",
        action="store_true",
        help="Try keeping current EE orientation (fallback to pos-only on failure).",
    )
    parser.add_argument("--plan-only", action="store_true", help="Plan only (no execution).")

    args = parser.parse_args()
    targets = parse_targets(args.targets)

    rclpy.init()
    node = SequenceMove(
        args.mode,
        targets,
        args.vel,
        args.acc,
        keep_orientation=bool(args.keep_orientation),
        plan_only=bool(args.plan_only),
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
