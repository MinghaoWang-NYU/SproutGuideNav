#!/usr/bin/env python3

import time
import uuid

import rclpy
from rclpy import wait_for_message
from rclpy.node import Node

from fauna_msgs.msg import MotorControlInputMode, MotorControlSystemState, VelocityCommand

# 指令仲裁用的优先级 (0 = SDK 级), 完整优先级表见官方 DOCS-SITE
FAUNA_SDK_COMMAND_PRIORITY = 0

POLICY_SEQUENCE = [
    (0.30, 0.0, 0.0, 2.0),   # walk forward 2 seconds
    (0.00, 0.0, 0.5, 2.0),   # turn left 2 seconds
    (0.30, 0.0, 0.2, 2.0),   # walk forward while turning left 2 seconds
    (0.00, 0.0, 0.0, 1.0),   # stop 1 second
]

CONTROL_HZ = 10.0
TOPIC = '/motor_control/velocity/command'
STATE_TOPIC = '/motor_control/system_state'

class PolicyVelocityBridge(Node):
    def __init__(self) -> None:
        super().__init__("policy_velocity_bridge")
        self.publisher = self.create_publisher(VelocityCommand, TOPIC, 10)
        self._sender_id = f"sdk-policy-velocity-bridge-{uuid.uuid4()}"
        self.get_logger().info("Velocity Publisher Initialized")
        self.start_t = self.get_clock().now()
        self.done = False

    def check_ready(self) -> bool:
        """发布前确认机器人处于 walk 模式且为 API 输入模式。"""
        _, system_state = wait_for_message.wait_for_message(MotorControlSystemState, self, STATE_TOPIC)
        if system_state.mode != "walk":
            self.get_logger().error(f"Robot is not in walk mode. Aborting. Current mode: '{system_state.mode}'")
            return False
        if system_state.input_mode != MotorControlInputMode.API:
            current_mode = "JOYSTICK" if system_state.input_mode == MotorControlInputMode.JOYSTICK else "UNKNOWN"
            self.get_logger().error(
                f"Robot is not in API input mode (currently in {current_mode} mode). "
                "To enable API control, use the Fauna app or joystick to switch to API mode."
            )
            return False
        return True

    def current_cmd(self, elapsed_s):
        t = 0.0
        for vx, vy, vyaw, dur in POLICY_SEQUENCE:
            if elapsed_s < t + dur:
                return vx, vy, vyaw
            t += dur
        return None

    def make_msg(self, vx, vy, vyaw):
        msg = VelocityCommand()
        # ---- 指令仲裁: sender_id 标识指令来源, priority 决定谁能压过谁 ----
        msg.command_priority.sender_id = self._sender_id
        msg.command_priority.priority = FAUNA_SDK_COMMAND_PRIORITY
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vyaw = float(vyaw)
        return msg

    def run_sequence(self):
        if not self.check_ready():
            return

        # 时间轴从真正开始发布时起算, 不含建节点和状态检查的耗时
        self.start_t = self.get_clock().now()
        while self.done is False:
            elapsed = (self.get_clock().now() - self.start_t).nanoseconds * 1e-9
            cmd = self.current_cmd(elapsed)
            if cmd is None:
                self.done = True
                break

            self.publisher.publish(self.make_msg(*cmd))
            time.sleep(1.0 / CONTROL_HZ)

        self.stop()

    def stop(self):
        """发零速度停车, 正常结束和异常中断共用。"""
        self.get_logger().info("Stopping...")
        self.publisher.publish(self.make_msg(0.0, 0.0, 0.0))

def main(args=None):
    rclpy.init(args=args)
    node = PolicyVelocityBridge()
    try:
        node.run_sequence()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")

    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()         