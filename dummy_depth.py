#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class DummyDepthFromColor(Node):
    """
    Subscribes to an RGB Image and publishes a same-size zero depth image with matching timestamps.
    """
    def __init__(self):
        super().__init__("dummy_depth")

        self.declare_parameter("in_rgb", "/robot/d455/color/image_raw")
        self.declare_parameter("out_depth", "/robot/d455/aligned_depth_to_color/image_raw")
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("encoding", "16UC1")  # try 32FC1 only if something complains
        self.declare_parameter("value_mm", 2000)  # 2m default
        self.value_mm = int(self.get_parameter("value_mm").value)


        self.in_rgb = self.get_parameter("in_rgb").value
        self.out_depth = self.get_parameter("out_depth").value
        self.frame_id = self.get_parameter("frame_id").value
        self.encoding = self.get_parameter("encoding").value

        self.pub = self.create_publisher(Image, self.out_depth, 10)
        self.sub = self.create_subscription(Image, self.in_rgb, self.cb, 10)

        self.get_logger().info(f"Dummy depth: {self.in_rgb} -> {self.out_depth} ({self.encoding})")

    def cb(self, rgb: Image):
        d = Image()
        d.header.stamp = rgb.header.stamp
        d.header.frame_id = self.frame_id
        d.height = rgb.height
        d.width = rgb.width
        d.is_bigendian = 0

        if self.encoding == "32FC1":
            d.encoding = "32FC1"
            d.step = d.width * 4
            # d.data = b"\x00" * (d.height * d.step)
            val = self.value_mm
            px = val.to_bytes(2, byteorder="little", signed=False)
            d.data = px * (d.height * d.width)

        else:
            d.encoding = "16UC1"
            d.step = d.width * 2
            # d.data = b"\x00" * (d.height * d.step)
            val = self.value_mm
            px = val.to_bytes(2, byteorder="little", signed=False)
            d.data = px * (d.height * d.width)


        self.pub.publish(d)

def main():
    rclpy.init()
    node = DummyDepthFromColor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
