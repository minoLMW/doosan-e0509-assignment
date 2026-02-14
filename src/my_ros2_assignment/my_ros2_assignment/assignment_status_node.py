#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from my_ros2_assignment_msgs.msg import AssignmentStatus

# motion_state enum
MOTION_IDLE = 0
MOTION_PLANNING = 1
MOTION_EXECUTING = 2
MOTION_STOPPING = 3
MOTION_ERROR = 4


class AssignmentStatusNode(Node):
    def __init__(self):
        super().__init__("assignment_status_node")

        # ---- params ----
        self.declare_parameter("base_frame", "base_link")
        # 너 환경에서 EE가 link_6는 확실하므로 기본값을 link_6로 둠
        self.declare_parameter("ee_frame", "link_6")
        self.declare_parameter("controller_name", "dsr_moveit_controller")
        self.declare_parameter("status_rate_hz", 10.0)
        self.declare_parameter("log_keep_n", 50)

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.controller_name = self.get_parameter("controller_name").value
        self.status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self.log_keep_n = int(self.get_parameter("log_keep_n").value)

        # ---- cache ----
        self.last_joint_state = JointState()
        self.recent_logs = []

        self.motion_state = MOTION_IDLE
        self.motion_state_text = "IDLE"
        self.is_moving = False
        self.active_goal_id = ""
        self.last_moveit_error_code = 0
        self.last_error_msg = ""

        self.mode = "abs"
        self.vel = 0.0
        self.acc = 0.0
        self.keep_orientation = False
        self.target_total = 0
        self.target_index = 0
        self.position_error_m = 0.0

        # TF 로그/상태
        self._tf_ready_once = False
        self._tf_not_ready_count = 0

        # ---- TF ----
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Action clients (서버 체크용) ----
        self.move_group_ac = ActionClient(self, MoveGroup, "/move_action")
        self.traj_ac = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self.controller_name}/follow_joint_trajectory",
        )

        # ---- pub/sub ----
        self.pub_status = self.create_publisher(AssignmentStatus, "/assignment/status", 10)
        self.create_subscription(JointState, "/joint_states", self.cb_joint, qos_profile_sensor_data)
        self.create_subscription(String, "/assignment/state_event", self.cb_state_event, 10)

        # ---- timer ----
        period = 1.0 / max(self.status_rate_hz, 1.0)
        self.create_timer(period, self.on_timer)

        self.push_log(f"StatusNode start base_frame={self.base_frame}, ee_frame={self.ee_frame}")

    # ---------------- callbacks ----------------
    def cb_joint(self, msg: JointState):
        self.last_joint_state = msg

    def cb_state_event(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        parts = [p.strip() for p in text.split(";") if p.strip()]
        kv = {}
        for p in parts:
            if "=" in p:
                k, v = p.split("=", 1)
                kv[k.strip()] = v.strip()

        if "state" in kv:
            st = kv["state"].upper()
            if st == "IDLE":
                self.set_motion(MOTION_IDLE, "IDLE", False)
            elif st == "PLANNING":
                self.set_motion(MOTION_PLANNING, "PLANNING", True)
            elif st == "EXECUTING":
                self.set_motion(MOTION_EXECUTING, "EXECUTING", True)
            elif st == "STOPPING":
                self.set_motion(MOTION_STOPPING, "STOPPING", True)
            elif st == "ERROR":
                self.set_motion(MOTION_ERROR, "ERROR", False)

        if "goal_id" in kv:
            self.active_goal_id = kv["goal_id"]
        if "idx" in kv:
            try:
                self.target_index = int(kv["idx"])
            except Exception:
                pass
        if "total" in kv:
            try:
                self.target_total = int(kv["total"])
            except Exception:
                pass
        if "mode" in kv:
            self.mode = kv["mode"]
        if "vel" in kv:
            try:
                self.vel = float(kv["vel"])
            except Exception:
                pass
        if "acc" in kv:
            try:
                self.acc = float(kv["acc"])
            except Exception:
                pass
        if "keep_orientation" in kv:
            self.keep_orientation = kv["keep_orientation"].lower() in ["1", "true", "yes", "y"]
        if "err" in kv:
            try:
                self.last_moveit_error_code = int(kv["err"])
            except Exception:
                pass
        if "msg" in kv:
            self.last_error_msg = kv["msg"]
        if "log" in kv:
            self.push_log(kv["log"])

    # ---------------- utils ----------------
    def set_motion(self, state: int, text: str, moving: bool):
        self.motion_state = state
        self.motion_state_text = text
        self.is_moving = moving

    def push_log(self, line: str):
        self.recent_logs.append(line)
        if len(self.recent_logs) > self.log_keep_n:
            self.recent_logs = self.recent_logs[-self.log_keep_n :]

    def get_ee_tf(self):
        """base_frame 기준 ee_frame TF transform. 없으면 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            return tf
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def check_action_servers(self):
        """
        실제 액션 서버 준비 여부 체크:
        - /move_action (MoveGroup)
        - /<controller>/follow_joint_trajectory (FollowJointTrajectory)
        """
        move_ok = self.move_group_ac.server_is_ready()
        traj_ok = self.traj_ac.server_is_ready()
        return move_ok, traj_ok

    # ---------------- main timer ----------------
    def on_timer(self):
        st = AssignmentStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = "assignment_status"

        # Connection
        move_ok, traj_ok = self.check_action_servers()
        st.move_group_available = bool(move_ok)
        st.trajectory_controller_ready = bool(traj_ok)
        st.controller_name = self.controller_name

        # Motion
        st.motion_state = self.motion_state
        st.motion_state_text = self.motion_state_text
        st.is_moving = self.is_moving
        st.active_goal_id = self.active_goal_id
        st.last_moveit_error_code = self.last_moveit_error_code
        st.last_error_msg = self.last_error_msg

        # Target
        st.target_total = int(self.target_total)
        st.target_index = int(self.target_index)
        st.mode = self.mode
        st.vel = float(self.vel)
        st.acc = float(self.acc)
        st.keep_orientation = bool(self.keep_orientation)

        # Robot state
        st.joint_state = self.last_joint_state

        # EE pose from TF
        tf = self.get_ee_tf()
        st.ee_pose_base.header.stamp = st.header.stamp
        st.ee_pose_base.header.frame_id = self.base_frame

        if tf is not None:
            st.ee_pose_base.pose.position.x = tf.transform.translation.x
            st.ee_pose_base.pose.position.y = tf.transform.translation.y
            st.ee_pose_base.pose.position.z = tf.transform.translation.z
            st.ee_pose_base.pose.orientation = tf.transform.rotation

            if not self._tf_ready_once:
                self._tf_ready_once = True
                self.push_log(f"TF ready: {self.base_frame} -> {self.ee_frame}")
        else:
            # TF 준비 전까지만 not-ready 로그를 스로틀로 남김
            if not self._tf_ready_once:
                self._tf_not_ready_count += 1
                if self._tf_not_ready_count % 10 == 0:
                    self.push_log(
                        f"TF not ready: {self.base_frame} -> {self.ee_frame} (x{self._tf_not_ready_count})"
                    )

        st.position_error_m = float(self.position_error_m)

        # Logs
        st.recent_logs = list(self.recent_logs)

        self.pub_status.publish(st)


def main():
    rclpy.init()
    node = AssignmentStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
