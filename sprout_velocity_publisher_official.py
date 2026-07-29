#!/usr/bin/env python3
"""
上层策略 -> Sprout 速度指令桥接节点 (ROS2 / rclpy)

模拟上层导航策略输出的 (vx, vy, vyaw) 速度指令序列,
以固定频率持续发布到 Sprout 的运动控制速度话题, 让机器人行走。

本文件按官方例程 walk_back_forth.py 的写法改写:
用阻塞循环 10 Hz 重发速度指令, 不使用 timer / spin。

  话题:  /motor_control/velocity/command
  消息:  fauna_msgs/msg/VelocityCommand
  字段:  vx    (m/s,  +前 / -后)
         vy    (m/s,  +左 / -右, 侧向平移)
         vyaw  (rad/s,+左转 / -右转)
         command_priority.sender_id  (指令仲裁用, 本节点唯一 id)
         command_priority.priority   (0 = SDK 级, 见官方 DOCS-SITE 优先级表)

运行前提 (与官方例程一致, 不满足会直接放弃):
  - 机器人处于 walk 模式        (system_state.mode == "walk")
  - 输入模式为 API             (system_state.input_mode == MotorControlInputMode.API)

字段名可用下面命令核对, 不一致就按实际定义改:
    ros2 interface show fauna_msgs/msg/VelocityCommand
    ros2 interface show fauna_msgs/msg/MotorControlSystemState
    ros2 interface show fauna_msgs/msg/CommandPriority
"""

import time
import uuid

import rclpy
from rclpy import wait_for_message
from rclpy.node import Node

from fauna_msgs.msg import MotorControlInputMode, MotorControlSystemState, VelocityCommand

# 指令仲裁用的优先级 (0 = SDK 级), 完整优先级表见官方 DOCS-SITE
FAUNA_SDK_COMMAND_PRIORITY = 0

# ---------- 模拟上层策略输出的速度指令序列 ----------
# 每段: (vx[m/s], vy[m/s], vyaw[rad/s], 持续时间[s])
POLICY_SEQUENCE = [
    (0.30, 0.0, 0.0, 2.0),   # walk forward 2 seconds
    (0.00, 0.0, 0.5, 2.0),   # turn left 2 seconds
    (0.30, 0.0, 0.2, 2.0),   # walk forward while turning left 2 seconds
    (0.00, 0.0, 0.0, 1.0),   # stop 1 second
]

PUBLISH_PERIOD = 0.1        # 发布周期 10 Hz; 指令 ~2s 无更新会过期
TOPIC = '/motor_control/velocity/command'
STATE_TOPIC = '/motor_control/system_state'


class PolicyVelocityBridge(Node):
    """把上层策略的速度序列以官方例程的方式发布给 Sprout。"""

    def __init__(self) -> None:
        super().__init__('policy_velocity_bridge')
        self.publisher = self.create_publisher(VelocityCommand, TOPIC, 10)
        self._sender_id = f'sdk-policy-velocity-bridge-{uuid.uuid4()}'
        self.get_logger().info(f'Policy velocity bridge initialized, publishing to {TOPIC}')

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

    def make_msg(self, vx: float, vy: float, vyaw: float) -> VelocityCommand:
        msg = VelocityCommand()
        # ---- 指令仲裁: sender_id 标识指令来源, priority 决定谁能压过谁 ----
        msg.command_priority.sender_id = self._sender_id
        msg.command_priority.priority = FAUNA_SDK_COMMAND_PRIORITY
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vyaw = float(vyaw)
        return msg

    def publish_for(self, vx: float, vy: float, vyaw: float, duration: float) -> None:
        """在 duration 秒内以 10 Hz 持续重发同一速度 (不要发一条就停)。"""
        msg = self.make_msg(vx, vy, vyaw)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(PUBLISH_PERIOD)

    def run_sequence(self) -> None:
        """按顺序执行 POLICY_SEQUENCE 中的每一段速度指令。"""
        if not self.check_ready():
            return

        for vx, vy, vyaw, duration in POLICY_SEQUENCE:
            self.get_logger().info(f'vx={vx} vy={vy} vyaw={vyaw} for {duration}s')
            self.publish_for(vx, vy, vyaw, duration)

        self.stop()

    def stop(self, duration: float | None = None) -> None:
        """停止机器人。"""
        self.get_logger().info("Stopping...")
        msg = self.make_msg(0.0, 0.0, 0.0)
        if duration:
            start_time = time.time()
            while time.time() - start_time < duration:
                self.publisher.publish(msg)
                time.sleep(PUBLISH_PERIOD)
        else:
            self.publisher.publish(msg)


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = PolicyVelocityBridge()

    try:
        node.run_sequence()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
