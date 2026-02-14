#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class StopClient(Node):
    def __init__(self):
        super().__init__("assignment_stop_client")
        self.cli = self.create_client(Trigger, "/assignment/stop")
        self.deadline_sec = 10.0
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.2, self.tick)
        self.called = False

    def tick(self):
        if self.called:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.deadline_sec:
            self.get_logger().error("STOP service not found within 10s. Exiting.")
            rclpy.shutdown()
            return

        if not self.cli.service_is_ready():
            # 서비스가 아직 없으면 조용히 기다림(스팸 로그 줄이기)
            return

        self.called = True
        req = Trigger.Request()
        fut = self.cli.call_async(req)
        fut.add_done_callback(self.done)

    def done(self, fut):
        try:
            res = fut.result()
            self.get_logger().info(f"STOP response: success={res.success}, msg='{res.message}'")
        except Exception as e:
            self.get_logger().error(f"STOP call failed: {e}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = StopClient()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
