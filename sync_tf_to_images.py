#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from collections import deque

def tfloat(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

class SyncTFToImages(Node):
    def __init__(self):
        super().__init__('sync_tf_to_images')
        self.declare_parameter('pose_topic', '/uav/pose')
        self.declare_parameter('image_topic', '/uav/color/image_raw')
        self.declare_parameter('parent_frame', 'uav_map')
        self.declare_parameter('child_frame', 'camera_frame')
        self.declare_parameter('max_pose_buffer', 5000)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.max_buf = int(self.get_parameter('max_pose_buffer').value)

        self.br = TransformBroadcaster(self)
        self.buf = deque()  # (t_float, msg)

        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 50)
        self.create_subscription(Image, self.image_topic, self.on_img, 50)

        self.get_logger().info(f"pose={self.pose_topic} image={self.image_topic} parent={self.parent} child={self.child}")

    def on_pose(self, msg: PoseStamped):
        self.buf.append((tfloat(msg.header.stamp), msg))
        while len(self.buf) > self.max_buf:
            self.buf.popleft()

    def _send(self, stamp, pose_msg: PoseStamped):
        tfm = TransformStamped()
        tfm.header.stamp = stamp
        tfm.header.frame_id = self.parent
        tfm.child_frame_id = self.child
        tfm.transform.translation.x = pose_msg.pose.position.x
        tfm.transform.translation.y = pose_msg.pose.position.y
        tfm.transform.translation.z = pose_msg.pose.position.z
        tfm.transform.rotation = pose_msg.pose.orientation
        self.br.sendTransform(tfm)

    def on_img(self, img: Image):
        if not self.buf:
            return
        ti = tfloat(img.header.stamp)

        # closest pose
        best = None
        best_dt = None
        for tp, pose in reversed(self.buf):
            dt = abs(tp - ti)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = pose
            if best_dt is not None and dt > best_dt and dt > 0.5:
                break
        if best is None:
            return

        # 1) publish at the pose stamp (timeline coverage)
        self._send(best.header.stamp, best)
        # 2) publish at the image stamp (exact match for FastSAM)
        self._send(img.header.stamp, best)

def main():
    rclpy.init()
    node = SyncTFToImages()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
