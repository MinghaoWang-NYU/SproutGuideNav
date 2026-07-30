"""
ROS2 node for extracting RGB-D images with odometry (Sprout / ZED2i).

Usage:
    python sensor/extract_data_two.py --output-dir /path/to/save
    python sensor/extract_data_two.py --output-dir /path/to/save --odom-topic /其他里程计话题

输出结构 (base_dir 会再套一层启动时间戳):
    <output-dir>/<YYYYmmdd_HHMMSS>/color/   ← ZED2i RGB 图像 (时间戳为文件名)
    <output-dir>/<YYYYmmdd_HHMMSS>/depth/   ← ZED2i 深度图 (16-bit PNG, 单位毫米, 时间戳为文件名)
    <output-dir>/<YYYYmmdd_HHMMSS>/odom.csv ← 里程计数据

建图走方式 A (topogen/gen_dinov3.py) 时只用 color/, depth/ 和 odom.csv 用不到但仍照常录制。
建图走 naive 方式 (sensor/build_topomap.py) 时 color/、depth/、odom.csv 三者都需要,
depth/ 只用于和 build_topomap.py 的时间戳对齐检查, 关键帧筛选逻辑本身不读取深度值。

注意: /zed/depth/depth_registered 发布的是 32FC1 (米制 float) 深度图, 这里转换成
16-bit PNG (毫米整数) 存储。如果后续要给 se2_estimate.py 的 PnP 分支使用这份深度数据,
需要在读取时把像素值除以 1000 换算回米, 目前 se2_estimate.py 里没有做这个换算。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2
import numpy as np
from datetime import datetime
import csv
import argparse
from nav_msgs.msg import Odometry


def parse_args():
    parser = argparse.ArgumentParser(description="Extract RGB-D data from ROS2 topics")
    parser.add_argument("--output-dir", "-o", type=str, default="./data_output",
                       help="Base output directory for extracted data")
    parser.add_argument("--odom-topic", type=str, default="/fused_odom",
                       help="Odometry topic name (default: /fused_odom). "
                            "用 `ros2 topic list | grep -i odom` 核对实际话题名")
    return parser.parse_args()


class SimpleImageSaver(Node):
    def __init__(self, output_base_dir, odom_topic='/fused_odom'):
        super().__init__('simple_image_saver')
        self.bridge = CvBridge()
        self.odom_topic = odom_topic

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_dir = os.path.join(output_base_dir, timestamp)

        # Create directories (目录名用 color/、depth/, 与 build_topomap.py / gen_dinov3.py 对齐)
        self.color_dir = os.path.join(self.base_dir, 'color')
        self.depth_dir = os.path.join(self.base_dir, 'depth')
        self.odom_csv_path = os.path.join(self.base_dir, 'odom.csv')

        os.makedirs(self.color_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        # Simple individual subscribers - no synchronization
        # Sprout ZED2i RGB + depth 话题 (SDK 文档 04 - Perception)
        # depth/ 是给 naive 建图 (build_topomap.py) 用的时间戳对齐输入; reloc3r repeat phase 本身不需要深度图
        self.create_subscription(Image, '/zed/rgb/image_rect_color', self.zed_rgb_callback, 10)
        # 实测确认: fauna_zed_node 发布 /zed/depth/depth_registered 用的是 RELIABLE
        # (`ros2 topic info /zed/depth/depth_registered -v` 可核实), 跟默认 QoS (10, RELIABLE) 一致即可,
        # 之前用 BEST_EFFORT (qos_profile_sensor_data) 订阅在这台机器的 rmw_zenoh_cpp 上收不到任何消息
        self.create_subscription(Image, '/zed/depth/depth_registered', self.zed_depth_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Setup CSV for odometry
        self.odom_file = open(self.odom_csv_path, 'w', newline='')
        self.odom_writer = csv.writer(self.odom_file)
        self.odom_writer.writerow([
            'timestamp',
            'pos_x', 'pos_y', 'pos_z',
            'ori_x', 'ori_y', 'ori_z', 'ori_w',
            'lin_vel_x', 'lin_vel_y', 'lin_vel_z',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z'
        ])

        # Counters
        self.color_count = 0
        self.depth_count = 0
        self.odom_count = 0

        # Status timer
        self.create_timer(5.0, self.status_callback)

        self.get_logger().info(f"Simple extractor ready - saving to {self.base_dir}")

    def zed_rgb_callback(self, msg):
        try:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ts_str = f"{timestamp:.9f}"

            # Convert and save image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_path = os.path.join(self.color_dir, f"{ts_str}.png")
            cv2.imwrite(image_path, cv_image)

            self.color_count += 1

        except Exception as e:
            self.get_logger().error(f"Error in zed_rgb_callback: {e}")

    def zed_depth_callback(self, msg):
        # 诊断用: 不经过任何转换代码, 确认回调本身有没有被中间件触发
        self.depth_cb_hits = getattr(self, 'depth_cb_hits', 0) + 1
        if self.depth_cb_hits <= 3 or self.depth_cb_hits % 50 == 0:
            self.get_logger().info(
                f"[diag] zed_depth_callback fired (#{self.depth_cb_hits}), "
                f"encoding={msg.encoding}, size={msg.width}x{msg.height}, step={msg.step}, "
                f"data_len={len(msg.data)}")
        try:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ts_str = f"{timestamp:.9f}"

            # /zed/depth/depth_registered 是 32FC1 (米), passthrough 拿原始浮点值
            depth_m = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)

            # 转成 16-bit PNG (毫米), 方便无损存储; 超出 uint16 范围的值截断
            depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
            depth_path = os.path.join(self.depth_dir, f"{ts_str}.png")
            cv2.imwrite(depth_path, depth_mm)

            self.depth_count += 1

        except Exception as e:
            import traceback
            self.get_logger().error(f"Error in zed_depth_callback: {e}\n{traceback.format_exc()}")

    def odom_callback(self, msg):
        try:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ts_str = f"{timestamp:.9f}"

            # Save odometry data
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            lv = msg.twist.twist.linear
            av = msg.twist.twist.angular

            self.odom_writer.writerow([
                ts_str,
                p.x, p.y, p.z,
                o.x, o.y, o.z, o.w,
                lv.x, lv.y, lv.z,
                av.x, av.y, av.z
            ])
            self.odom_file.flush()

            self.odom_count += 1

        except Exception as e:
            self.get_logger().error(f"Error in odom_callback: {e}")

    def status_callback(self):
        """Print status every 5 seconds"""
        total = self.color_count + self.depth_count + self.odom_count
        self.get_logger().info(
            f"Saved - Color: {self.color_count}, Depth: {self.depth_count}, "
            f"Odom: {self.odom_count} (Total: {total})"
        )
        if self.color_count == 0:
            self.get_logger().warning(
                "未收到任何 RGB 图像, 请用 `ros2 topic hz /zed/rgb/image_rect_color` 核对相机话题")
        if self.depth_count == 0:
            self.get_logger().warning(
                "未收到任何深度图, 请用 `ros2 topic hz /zed/depth/depth_registered` 核对相机话题。"
                "建图方式 A (gen_dinov3.py) 不需要 depth, 此警告可忽略; naive 建图 (build_topomap.py) 需要 depth")
        if self.odom_count == 0:
            self.get_logger().warning(
                f"未收到任何里程计消息 (订阅的是 '{self.odom_topic}'), "
                "请用 `ros2 topic list | grep -i odom` 核对话题名, 并用 --odom-topic 指定。"
                "建图方式 A 不需要 odom, 此警告可忽略")

    def destroy_node(self):
        total = self.color_count + self.depth_count + self.odom_count
        self.get_logger().info(f"Final count: {total} messages saved")
        if hasattr(self, 'odom_file'):
            self.odom_file.close()
        super().destroy_node()


def main():
    args = parse_args()
    rclpy.init()
    node = None
    try:
        node = SimpleImageSaver(args.output_dir, odom_topic=args.odom_topic)
        node.get_logger().info("Simple ImageSaver node started.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    except Exception as e:
        print(f"Exception in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
