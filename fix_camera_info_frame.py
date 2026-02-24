#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoFrameFix(Node):
    def __init__(self):
        super().__init__('camera_info_frame_fix')
        self.declare_parameter('in_topic', '/uav/color/info')
        self.declare_parameter('out_topic', '/robot/d455/color/camera_info')
        self.declare_parameter('frame_id', 'camera_frame')  

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(CameraInfo, out_topic, 10)
        self.sub = self.create_subscription(CameraInfo, in_topic, self.cb, 10)
        self.get_logger().info(f"Relaying {in_topic} -> {out_topic} with frame_id='{self.frame_id}'")

    # def cb(self, msg: CameraInfo):
    #     msg.header.frame_id = self.frame_id
    #     self.pub.publish(msg)
    def cb(self, msg: CameraInfo):
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.stamp = msg.header.stamp
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CameraInfoFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()