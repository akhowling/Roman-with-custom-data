#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/robot/odom')
        self.declare_parameter('parent_frame', 'robot/odom')
        self.declare_parameter('child_frame', 'robot/base_link')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb, 50)

    # def cb(self, msg: Odometry):
    #     t = TransformStamped()
    #     t.header = msg.header
    #     t.header.frame_id = self.parent_frame
    #     t.child_frame_id = self.child_frame
    #     t.transform.translation.x = msg.pose.pose.position.x
    #     t.transform.translation.y = msg.pose.pose.position.y
    #     t.transform.translation.z = msg.pose.pose.position.z
    #     t.transform.rotation = msg.pose.pose.orientation
    #     self.br.sendTransform(t)
    def cb(self, msg: Odometry):
        t = TransformStamped()

        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Copy stamp
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec

        # SUBTRACT 0.2 SECONDS
        nsec -= 200_000_000
        if nsec < 0:
            sec -= 1
            nsec += 1_000_000_000

        t.header.stamp.sec = sec
        t.header.stamp.nanosec = nsec

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()