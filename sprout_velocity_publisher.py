#!/usr/bin/env python3
"""
上层策略 -> Sprout 速度指令桥接节点 (ROS2 / rclpy)

模拟上层导航策略输出的 (vx, vy, vyaw) 速度指令序列,
以固定频率持续发布到 Sprout 的运动控制速度话题, 让机器人行走。

  话题:  /motor_control/velocity/command
  消息:  fauna_msgs/msg/VelocityCommand
  字段:  vx    (m/s,  +前 / -后)
         vy    (m/s,  +左 / -右, 侧向平移)
         vyaw  (rad/s,+左转 / -右转)
         header.stamp / header.id
         command_priority.sender_id      (必填, 否则指令被忽略)
         command_priority.priority        (默认 0 = SDK 级)
         command_priority.release_control (最后一条设 True 交还控制权)

运行前请先用下面命令确认字段名与本脚本一致, 不一致就按实际定义改:
    ros2 interface show fauna_msgs/msg/VelocityCommand
    ros2 interface show fauna_msgs/msg/CommandPriority
"""

import uuid
import rclpy
from rclpy.node import Node
from fauna_msgs.msg import VelocityCommand

# ---------- 模拟上层策略输出的速度指令序列 ----------
# 每段: (vx[m/s], vy[m/s], vyaw[rad/s], 持续时间[s])
POLICY_SEQUENCE = [
    (0.30, 0.0, 0.0, 2.0),   # 直行前进 2 秒
    (0.00, 0.0, 0.5, 2.0),   # 原地左转 2 秒
    (0.30, 0.0, 0.2, 2.0),   # 边走边微左转 2 秒
    (0.00, 0.0, 0.0, 1.0),   # 停住 1 秒
]

CONTROL_HZ = 5.0            # 发布频率; 指令 ~2s 无更新会过期
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

    def current_cmd(self, elapsed_s): # elapsed_s为当前时间
        """根据已运行时间取出当前 (vx, vy, vyaw); 序列结束返回 None。"""
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
        # ---- 命令仲裁: sender_id 必填, 否则整条指令被丢弃 ----
        msg.command_priority.sender_id = self.sender_id
        msg.command_priority.priority = 0            # 需要压过其它来源时调高
        msg.command_priority.release_control = release
        return msg

    def on_tick(self):
        elapsed = (self.get_clock().now() - self.start_t).nanoseconds * 1e-9
        cmd = self.current_cmd(elapsed)
        if cmd is None:
            self.done = True
            return
        # 持续重发当前速度以维持行走 (不要发一条就停)
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
        # 结束/中断: 多发几条零速度并交还控制权, 保证机器人停稳
        for _ in range(5):
            node.pub.publish(node.make_msg(0.0, 0.0, 0.0, release=True))
        node.get_logger().info('stopped, control released.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()