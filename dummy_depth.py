#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def reliable_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

class DummyDepthFromRGB(Node):
    def __init__(self):
        super().__init__("dummy_depth")

        # Inputs (already relayed + frame-fixed in your setup)
        self.declare_parameter("rgb_image", "/robot/d455/color/image_raw")
        self.declare_parameter("rgb_info",  "/robot/d455/color/camera_info")

        # Outputs (what ROMAN expects if it wants depth)
        self.declare_parameter("depth_image_out", "/robot/d455/aligned_depth_to_color/image_raw")
        self.declare_parameter("depth_info_out",  "/robot/d455/aligned_depth_to_color/camera_info")

        # Frame + encoding
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("encoding", "16UC1")  # common for RealSense depth; alt: 32FC1

        self.rgb_image = self.get_parameter("rgb_image").value
        self.rgb_info  = self.get_parameter("rgb_info").value
        self.depth_image_out = self.get_parameter("depth_image_out").value
        self.depth_info_out  = self.get_parameter("depth_info_out").value
        self.frame_id = self.get_parameter("frame_id").value
        self.encoding = self.get_parameter("encoding").value

        # Pub QoS: RELIABLE so ROMAN subscribers (often RELIABLE) won't reject it
        self.pub_depth = self.create_publisher(Image, self.depth_image_out, reliable_qos(depth=5))
        self.pub_info  = self.create_publisher(CameraInfo, self.depth_info_out, reliable_qos(depth=10))

        # Sub QoS: RELIABLE (your bag overrides set RELIABLE for your color topics)
        self.last_info = None
        self.create_subscription(CameraInfo, self.rgb_info, self.on_info, reliable_qos(depth=10))
        self.create_subscription(Image, self.rgb_image, self.on_rgb, reliable_qos(depth=5))

        self.get_logger().info(
            f"Dummy depth ON. RGB: {self.rgb_image}, {self.rgb_info} -> "
            f"DEPTH: {self.depth_image_out}, {self.depth_info_out} (encoding={self.encoding})"
        )

    def on_info(self, msg: CameraInfo):
        # Keep intrinsics; only enforce frame
        msg.header.frame_id = self.frame_id
        self.last_info = msg
        # publish immediately too
        self.pub_info.publish(msg)

    def on_rgb(self, rgb: Image):
        # Publish depth camera_info aligned to this rgb timestamp
        if self.last_info is not None:
            info = self.last_info
            info.header.stamp = rgb.header.stamp
            info.header.frame_id = self.frame_id
            self.pub_info.publish(info)

        depth = Image()
        depth.header.stamp = rgb.header.stamp
        depth.header.frame_id = self.frame_id
        depth.height = rgb.height
        depth.width = rgb.width
        depth.is_bigendian = 0

        if self.encoding == "32FC1":
            depth.encoding = "32FC1"
            depth.step = depth.width * 4
            depth.data = b"\x00" * (depth.height * depth.step)
        else:
            # default to 16UC1
            depth.encoding = "16UC1"
            depth.step = depth.width * 2
            depth.data = b"\x00" * (depth.height * depth.step)

        self.pub_depth.publish(depth)

def main():
    rclpy.init()
    node = DummyDepthFromRGB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
