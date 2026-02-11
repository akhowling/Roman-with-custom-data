#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTF(Node):
    def __init__(self):
        super().__init__('static_tf_uav_map_to_camera_frame')
        self.declare_parameter('parent', 'uav_map')
        self.declare_parameter('child', 'camera_frame')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)

        parent = self.get_parameter('parent').value
        child  = self.get_parameter('child').value
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)

        br = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = rclpy.time.Time(seconds=0).to_msg()  # time-invariant
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0
        br.sendTransform(t)
        self.get_logger().info(f"Published static TF {parent} -> {child} at (x,y,z)=({x},{y},{z})")

def main():
    rclpy.init()
    node = StaticTF()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
