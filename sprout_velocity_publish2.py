#!/usr/bin/env python3
"""
High-level policy -> Sprout velocity command bridge node (ROS2 / rclpy)

Simulates a sequence of (vx, vy, vyaw) velocity commands from a high-level
navigation policy and publishes them at a fixed rate to Sprout's motor-control
velocity topic so the robot walks.

  Topic:   /motor_control/velocity/command
  Message: fauna_msgs/msg/VelocityCommand
  Fields:  vx    (m/s,   +forward / -backward)
           vy    (m/s,   +left / -right, lateral strafe)
           vyaw  (rad/s, +turn left / -turn right)
           header.stamp / header.id
           command_priority.sender_id      (required; command is ignored without it)
           command_priority.priority        (default 0 = SDK level)
           command_priority.release_control (set True on the final message to hand control back)

Before running, confirm the field names match this script (adjust if the actual
definitions differ):
    ros2 interface show fauna_msgs/msg/VelocityCommand
    ros2 interface show fauna_msgs/msg/CommandPriority
"""

import uuid
import rclpy
from rclpy.node import Node
from fauna_msgs.msg import VelocityCommand

# ---------- Simulated velocity command sequence from the high-level policy ----------
# Each segment: (vx[m/s], vy[m/s], vyaw[rad/s], duration[s])
POLICY_SEQUENCE = [
    (0.30, 0.0, 0.0, 2.0),   # walk forward 2 seconds
    (0.00, 0.0, 0.5, 2.0),   # turn left 2 seconds
    (0.30, 0.0, 0.2, 2.0),   # walk forward while turning left 2 seconds
    (0.00, 0.0, 0.0, 1.0),   # stop 1 second
]

CONTROL_HZ = 5.0            # Publishing frequency; a command expires after ~2s without an update
TOPIC = '/motor_control/velocity/command'


class PolicyVelocityBridge(Node):
    def __init__(self):
        super().__init__('policy_velocity_bridge')
        self.pub = self.create_publisher(VelocityCommand, TOPIC, 10)
        self.sender_id = f'high-level-policy-{uuid.uuid4()}'
        self.msg_count = 0
        self.done = False
        self.start_t = self.get_clock().now()
        self.create_timer(1.0 / CONTROL_HZ, self.on_tick)
        self.get_logger().info(f'publishing to {TOPIC}, sender_id={self.sender_id}')

    def current_cmd(self, elapsed_s):  # elapsed_s = time since start
        """Return the current (vx, vy, vyaw) based on elapsed time; None when the sequence ends."""
        t = 0.0
        for vx, vy, vyaw, dur in POLICY_SEQUENCE:
            if elapsed_s < t + dur:
                return vx, vy, vyaw
            t += dur
        return None

    def make_msg(self, vx, vy, vyaw, release=False):
        msg = VelocityCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.id = str(self.msg_count)
        self.msg_count += 1
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vyaw = float(vyaw)
        # ---- Command arbitration: sender_id is required, or the whole command is dropped ----
        msg.command_priority.sender_id = self.sender_id
        msg.command_priority.priority = 0            # raise this to outrank other sources
        msg.command_priority.release_control = release
        return msg

    def on_tick(self):
        elapsed = (self.get_clock().now() - self.start_t).nanoseconds * 1e-9
        cmd = self.current_cmd(elapsed)
        if cmd is None:
            self.done = True
            return
        # Keep re-publishing the current velocity to sustain walking (don't send just once and stop)
        self.pub.publish(self.make_msg(*cmd))


def main():
    rclpy.init()
    node = PolicyVelocityBridge()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # On finish/interrupt: send a few zero-velocity commands and release control so the robot stops safely
        for _ in range(5):
            node.pub.publish(node.make_msg(0.0, 0.0, 0.0, release=True))
        node.get_logger().info('stopped, control released.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()