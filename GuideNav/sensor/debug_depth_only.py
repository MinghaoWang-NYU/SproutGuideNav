#!/usr/bin/env python3
"""
最小化诊断脚本: 只订阅 /zed/depth/depth_registered, 不做任何其他事情。
用来判断 depth 收不到消息是不是跟 extract_data_two.py 里同时存在
RGB/Odom 订阅、cv2.imwrite 等其他逻辑有关, 还是这个话题本身的问题。

Usage:
    python3 sensor/debug_depth_only.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthOnlyListener(Node):
    def __init__(self):
        super().__init__('debug_depth_only_listener')
        self.count = 0
        self.create_subscription(Image, '/zed/depth/depth_registered', self.cb, 10)
        self.create_timer(2.0, self.status)
        self.get_logger().info("Subscribed to /zed/depth/depth_registered, waiting...")

    def cb(self, msg):
        self.count += 1
        self.get_logger().info(
            f"[HIT #{self.count}] encoding={msg.encoding} size={msg.width}x{msg.height} "
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )

    def status(self):
        self.get_logger().info(f"status: received {self.count} depth messages so far")


def main():
    rclpy.init()
    node = DepthOnlyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
