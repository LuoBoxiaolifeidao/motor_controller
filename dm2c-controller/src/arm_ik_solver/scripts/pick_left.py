#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        self.tcp_pub = self.create_publisher(Float64MultiArray, "/left_tcp", 10)
        self.finger_pub = self.create_publisher(Float64MultiArray, "/left_tool", 10)

        self.steps = [
            # (description, tcp_target, finger_target, attach, detach, delay_sec)
            ("move to cup",        [-0.4, 0.3, 0.9, 0, 0, -1.57], None, False, False, 4.0),
            ("approach cup",       [-0.292, 0.5, 0.75, 0, 0, -1.57], None, False, False, 3.5),
            ("attach grip",        None, None, True, False, 0.5),
            ("close fingers",      None, [-0.33, 0.33], False, False, 3.0),
            ("lift cup",           [-0.292, 0.7, 0.95, 0, 0, -1.57], None, False, False, 3.5),
            ("move to dest",       [-0.292, 0.5, 0.75, 0, 0, -1.57], None, False, False, 3.5),
            ("open fingers",       None, [0.0, 0.0], False, False, 3.0),
            ("detach grip",        None, None, False, True, 0.5),
            ("retract arm",        [-0.4, 0.3, 0.9, 0, 0, -1.57], None, False, False, 3.5),
            ("go home",            [-0.128, 0.349, 0.296, 0, 0, 3.142], None, False, False, 1.0),
        ]
        self.idx = 0
        self.start_next_step()

    def start_next_step(self):
        if self.idx >= len(self.steps):
            self.get_logger().info("all steps done")
            rclpy.shutdown()
            return
        desc, tcp, finger, attach, detach, delay = self.steps[self.idx]
        self.get_logger().info(f"step {self.idx + 1}/{len(self.steps)}: {desc}")

        if tcp is not None:
            self.tcp_pub.publish(Float64MultiArray(data=tcp))
        if finger is not None:
            self.finger_pub.publish(Float64MultiArray(data=finger))
        if attach:
            self.run_ign("attach")
        if detach:
            self.run_ign("detach")

        self.idx += 1
        self._timer = self.create_timer(delay, self._on_timer)

    def _on_timer(self):
        self._timer.cancel()
        self.start_next_step()

    def run_ign(self, action):
        topic = "/left_gripper/" + action
        cmd = ["ign", "topic", "-t", topic, "-m", "ignition.msgs.Empty", "-p", ""]
        self.get_logger().info(f"ign: {' '.join(cmd)}")
        try:
            subprocess.run(cmd, timeout=5)
        except Exception as e:
            self.get_logger().warn(f"ign failed: {e}")


def main():
    rclpy.init()
    node = PickAndPlace()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
