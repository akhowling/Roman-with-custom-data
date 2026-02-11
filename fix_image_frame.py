#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageFrameFix(Node):
    def __init__(self):
        super().__init__('image_frame_fix')
        self.declare_parameter('in_topic', '/uav/color/image')
        self.declare_parameter('out_topic', '/robot/d455/color/image_raw')
        self.declare_parameter('frame_id', 'camera_frame') 

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Image, out_topic, 10)
        self.sub = self.create_subscription(Image, in_topic, self.cb, 10)
        self.get_logger().info(f"Relaying {in_topic} -> {out_topic} with frame_id='{self.frame_id}'")

    def cb(self, msg: Image):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImageFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
