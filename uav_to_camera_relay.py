#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data


def normalize_frame_id(frame_id: str, default_frame: str) -> str:
    if not frame_id:
        return default_frame
    fid = frame_id.lstrip('/')
    return fid if fid else default_frame


class UAVToCameraRelay(Node):
    def __init__(self):
        super().__init__("uav_to_camera_relay")

        # Use sim time so timestamps line up with /clock from rosbag
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.parameter.Parameter.Type.BOOL,
            True
        )])

        # QoS: images often are best-effort sensor data
        img_qos = qos_profile_sensor_data

        # CameraInfo often works fine with sensor_data QoS too, but keep it explicit:
        info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # Publishers (your "generic camera" topics)
        self.pub_rgb = self.create_publisher(Image, "/uav/color/image_raw", 10)
        self.pub_rgb_info = self.create_publisher(CameraInfo, "/uav/color/camera_info", 10)
        self.pub_depth = self.create_publisher(Image, "/uav/depth/image_raw", 10)
        self.pub_depth_info = self.create_publisher(CameraInfo, "/uav/depth/camera_info", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/pose", 50)

        
        # self.pub_uav_color_info = self.create_publisher(CameraInfo, "/uav/color/camera_info", 10)
        # self.pub_uav_depth_info = self.create_publisher(CameraInfo, "/uav/depth/camera_info", 10)

        
        # self.pub_uav_color_image_raw = self.create_publisher(Image, "/uav/color/image_raw", 10)
        # self.pub_uav_depth_image_raw = self.create_publisher(Image, "/uav/depth/image_raw", 10)

        # Subscriptions (from your bag)
        self.sub_rgb = self.create_subscription(Image, "/uav/color/image", self.cb_rgb, img_qos)
        self.sub_rgb_info = self.create_subscription(CameraInfo, "/uav/color/info", self.cb_rgb_info, info_qos)
        self.sub_depth = self.create_subscription(Image, "/uav/depth/image", self.cb_depth, img_qos)
        self.sub_depth_info = self.create_subscription(CameraInfo, "/uav/depth/info", self.cb_depth_info, info_qos)
        self.sub_pose = self.create_subscription(PoseStamped, "/uav/pose", self.cb_pose, pose_qos)

    def cb_rgb(self, msg: Image):
        msg.header.frame_id = normalize_frame_id(msg.header.frame_id, "camera_frame")
        self.pub_rgb.publish(msg)
        # self.pub_uav_color_image_raw.publish(msg)

    def cb_rgb_info(self, msg: CameraInfo):
        msg.header.frame_id = normalize_frame_id(msg.header.frame_id, "camera_frame")
        
        self.pub_rgb_info.publish(msg)
       
        # self.pub_uav_color_info.publish(msg)

    def cb_depth(self, msg: Image):
        msg.header.frame_id = normalize_frame_id(msg.header.frame_id, "camera_frame")
        self.pub_depth.publish(msg)
        # self.pub_uav_depth_image_raw.publish(msg)

    def cb_depth_info(self, msg: CameraInfo):
        msg.header.frame_id = normalize_frame_id(msg.header.frame_id, "camera_frame")
        
        self.pub_depth_info.publish(msg)
        
        # self.pub_uav_depth_info.publish(msg)

    def cb_pose(self, msg: PoseStamped):
        msg.header.frame_id = normalize_frame_id(msg.header.frame_id, "camera_frame")
        self.pub_pose.publish(msg)
    
    
    



def main():
    rclpy.init()
    node = UAVToCameraRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

