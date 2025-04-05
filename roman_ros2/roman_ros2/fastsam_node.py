#!/usr/bin/env python3
import numpy as np
import os
from scipy.spatial.transform import Rotation as Rot
import struct
import open3d as o3d

# ROS imports
import rclpy
from rclpy.node import Node
import cv_bridge
import message_filters
import ros2_numpy as rnp
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile
import tf2_ros
from rclpy.executors import MultiThreadedExecutor

# ROS msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import roman_msgs.msg as roman_msgs

# robot_utils
from robotdatapy.camera import CameraParams

# ROMAN
from roman.map.fastsam_wrapper import FastSAMWrapper
from roman.params.fastsam_params import FastSAMParams

# relative
from roman_ros2.utils import observation_to_msg

class FastSAMNode(Node):

    def __init__(self):
        super().__init__('fastsam_node')
        
        # internal variables
        self.bridge = cv_bridge.CvBridge()

        # ros params
        self.declare_parameters(
            namespace='',
            parameters=[
                ("map_frame_id", "map"),
                ("odom_base_frame_id", "base"),
                ("config_path", ""),
                ("min_dt", 0.1),
                # ("fastsam_viz", False),
            ]
        )

        self.cam_frame_id = None
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.odom_base_frame_id = self.get_parameter("odom_base_frame_id").value
        self.min_dt = self.get_parameter("min_dt").value
        config_path = self.get_parameter("config_path").value

        # self.visualize = self.get_parameter("fastsam_viz").value
        self.visualize = False # TODO: is supporting this helpful?
        self.last_t = -np.inf

        # FastSAM set up after camera info can be retrieved
        self.get_logger().info("Waiting for depth camera info messages...")
        depth_info_msg = self._wait_for_message("depth/camera_info", sensor_msgs.CameraInfo)
        self.get_logger().info("Received for depth camera info messages...")
        self.get_logger().info("Waiting for color camera info messages...")
        color_info_msg = self._wait_for_message("color/camera_info", sensor_msgs.CameraInfo)
        self.get_logger().info("Received for color camera info messages...")
        self.depth_params = CameraParams.from_msg(depth_info_msg)
        color_params = CameraParams.from_msg(color_info_msg)

        # fastsam wrapper
        fastsam_params = FastSAMParams.from_yaml(config_path)
        self.fastsam = FastSAMWrapper.from_params(fastsam_params, self.depth_params)

        self.setup_ros()

    def _wait_for_message(self, topic, msg_type):
        """
        Wait for a message on topic of type msg_type
        """
        subscription = self.create_subscription(msg_type, topic, self._wait_for_message_cb, 1)
        
        self._wait_for_message_msg = None
        while self._wait_for_message_msg is None:
            rclpy.spin_once(self)
        msg = self._wait_for_message_msg
        # subscription.destroy()

        return msg
    
    def _wait_for_message_cb(self, msg):
        self._wait_for_message_msg = msg
        return

    def setup_ros(self):

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ros subscribers
        subs = [
            message_filters.Subscriber(self, sensor_msgs.Image, "color/image_raw"),
            message_filters.Subscriber(self, sensor_msgs.Image, "depth/image_raw"),
        ]
        self.ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.1)
        self.ts.registerCallback(self.cb) # registers incoming messages to callback

        # ros publishers
        self.obs_pub = self.create_publisher(roman_msgs.ObservationArray, "roman/observations", qos_profile=QoSProfile(depth=10))

        if self.visualize:
            self.ptcld_pub = self.create_publisher(sensor_msgs.PointCloud, "roman/observations/ptcld")

        self.get_logger().info("FastSAM node setup complete.")

    def cb(self, *msgs):
        """
        This function gets called every time synchronized odometry, image message, and 
        depth image message are received.
        """
        
        self.get_logger().info("Received messages")
        img_msg, depth_msg = msgs
        if self.cam_frame_id is None:
            self.cam_frame_id = img_msg.header.frame_id

        # check that enough time has passed since last observation (to not overwhelm GPU)
        t = rclpy.time.Time.from_msg(img_msg.header.stamp).nanoseconds * 1e-9
        if t - self.last_t < self.min_dt:
            return
        else:
            self.last_t = t

        try:
            # self.tf_buffer.waitForTransform(self.map_frame_id, self.cam_frame_id, img_msg.header.stamp, rospy.Duration(0.5))
            transform_stamped_msg = self.tf_buffer.lookup_transform(self.map_frame_id, self.cam_frame_id, img_msg.header.stamp, rclpy.duration.Duration(seconds=2.0))
            flu_transformed_stamped_msg = self.tf_buffer.lookup_transform(self.map_frame_id, self.odom_base_frame_id, img_msg.header.stamp, rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warning("tf lookup failed")
            self.get_logger().warning(str(ex))
            return       
         
        pose = rnp.numpify(transform_stamped_msg.transform).astype(np.float64)
        pose_flu = rnp.numpify(flu_transformed_stamped_msg.transform).astype(np.float64)

        

        # conversion from ros msg to cv img
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg)

        observations = self.fastsam.run(t, pose, img, img_depth=depth)

        observation_msgs = [observation_to_msg(obs) for obs in observations]
        
        observation_array = roman_msgs.ObservationArray(
            header=img_msg.header,
            pose=rnp.msgify(geometry_msgs.Pose, pose),
            pose_flu=rnp.msgify(geometry_msgs.Pose, pose_flu),
            observations=observation_msgs
        )
        self.obs_pub.publish(observation_array)

        # if self.visualize:
        #     self.pub_ptclds(observations, img_msg.header, depth)

        return
    
    # def pub_ptclds(self, observations, header, depth):
    #     points_msg = sensor_msgs.PointCloud()
    #     points_msg.header = header
    #     points_msg.header.frame_id = self.cam_frame_id
    #     points_msg.points = []
    #     points_msg.channels = [sensor_msgs.ChannelFloat32(name='rgb', values=[])]

    #     for i, obs in enumerate(observations):
    #         # color
    #         color_unpacked = np.random.rand(3)*256
    #         color_raw = int(color_unpacked[0]*256**2 + color_unpacked[1]*256 + color_unpacked[2])
    #         color_packed = struct.unpack('f', struct.pack('i', color_raw))[0]
            
    #         points = obs.point_cloud
    #         sampled_points = np.random.choice(len(points), min(len(points), 100), replace=True)
    #         points = [points[i] for i in sampled_points]
    #         points_msg.points += [geometry_msgs.Point32(x=p[0], y=p[1], z=p[2]) for p in points]
    #         points_msg.channels[0].values += [color_packed for _ in points]
        
    #     self.ptcld_pub.publish(points_msg)

def main():

    rclpy.init()
    node = FastSAMNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == "__main__":
    main()

