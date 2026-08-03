"""
保存地图 - Python 直接订阅 /map 话题（替代 map_saver）

解决 map_saver 在 WSL 的 DDS 订阅问题。
订阅 /map → 保存为 PGM + YAML。

Usage:
    python3 save_map.py
"""
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import numpy as np


class MapSaver(Node):
    def __init__(self):
        super().__init__("map_saver_py")
        self.sub = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10)
        self.received = False
        self.get_logger().info("等待 /map 数据...")

    def map_callback(self, msg):
        if self.received:
            return
        self.received = True
        self.get_logger().info(f"收到地图: {msg.info.width}x{msg.info.height}, 分辨率 {msg.info.resolution}")

        # 栅格转图像
        width, height = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape(height, width)
        # -1未知(205灰), 0空闲(255白), 100占用(0黑)
        img = np.zeros((height, width), dtype=np.uint8)
        img[data < 0] = 205
        img[data == 0] = 255
        img[data > 50] = 0
        img[data > 0] = 254  # 其他占用

        # 保存 PGM
        pgm_path = "/root/tb3_map/indoor_map.pgm"
        Image.fromarray(img).save(pgm_path)

        # 保存 YAML
        yaml_path = "/root/tb3_map/indoor_map.yaml"
        yaml_content = f"""image: indoor_map.pgm
resolution: {msg.info.resolution}
origin: [{msg.info.origin.position.x}, {msg.info.origin.position.y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
        with open(yaml_path, "w") as f:
            f.write(yaml_content)

        self.get_logger().info(f"地图已保存: {pgm_path} + {yaml_path}")
        self.get_logger().info("下一步: 启动 AMCL 定位 + Nav2 导航")


def main():
    rclpy.init()
    saver = MapSaver()
    try:
        # 等待地图回调（最多30秒）
        for _ in range(60):
            rclpy.spin_once(saver, timeout_sec=0.5)
            if saver.received:
                break
    finally:
        saver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
