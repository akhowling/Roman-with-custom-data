#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToCameraTF(Node):
    def __init__(self):
        super().__init__("pose_to_camera_tf")
        self.declare_parameter("child_frame", "/camera_frame")
        self.child_frame = self.get_parameter("child_frame").value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseStamped, "/pose", self.cb, 50)

    def cb(self, msg: PoseStamped):
        # parent frame comes from PoseStamped header.frame_id
        parent = msg.header.frame_id if msg.header.frame_id else "map"

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = parent
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(PoseToCameraTF())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

