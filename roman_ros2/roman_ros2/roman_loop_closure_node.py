#!/usr/bin/env python3

import numpy as np
from dataclasses import dataclass
from typing import List
from copy import deepcopy

# ROS imports
import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from rclpy.executors import MultiThreadedExecutor
import tf2_ros

# ROS msgs
import roman_msgs.msg as roman_msgs
import geometry_msgs.msg as geometry_msgs
import visualization_msgs.msg as visualization_msgs

# Custom modules
from robotdatapy.transform import transform_to_xyzrpy

from roman.object.segment import Segment, SegmentMinimalData
from roman.align.roman_registration import ROMANRegistration, ROMANParams
from roman.align.object_registration import InsufficientAssociationsException

from roman.params.submap_align_params import SubmapAlignParams
from roman.utils import transform_rm_roll_pitch
from roman.map.map import Submap

# Local imports
from roman_ros2.utils import msg_to_segment, MapColors, default_marker

@dataclass
class SegmentUpdateResult():
    """
    Class to store the result of a segment update.
    """
    submap_created: bool
    submap_segments: List[SegmentMinimalData] = None

class SegmentQueue():

    def __init__(self, submap_num_segments: int, submap_overlapping_segments: int):
        self.segments = dict()
        self.submap_num_segments = submap_num_segments
        self.submap_overlapping_segments = submap_overlapping_segments

    def update(self, segment: Segment):
        # update existing segment
        if segment.id in self.segments:
            segment.first_seen = self.segments[segment.id].first_seen
            self.segments[segment.id] = segment
            return SegmentUpdateResult(False)

        # create new segment
        segment.first_seen = segment.last_seen
        self.segments[segment.id] = segment

        # check if we have enough segments to create a submap
        if len(self.segments) < self.submap_num_segments:
            return SegmentUpdateResult(False)
        
        assert len(self.segments) == self.submap_num_segments, \
            "ERROR: Segment queue is incorrect size."

        # create submap
        submap = []
        for seg_id, seg in self.segments.items():
            submap.append(seg)

        segments_sorted_by_time = sorted(submap, key=lambda seg: np.mean([seg.first_seen, seg.last_seen]))
        rm_segment_ids = [seg.id for seg in segments_sorted_by_time[:-self.submap_overlapping_segments]]

        # remove segments that will not be include in subsequent submaps
        for seg_id in rm_segment_ids:
            del self.segments[seg_id]

        return SegmentUpdateResult(True, submap)
            

class ROMANLoopClosureNode(Node):

    def __init__(self):
        
        super().__init__('roman_loop_closure_node')

        # ros parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ("config_path", ""),
                ("submap_num_segments", 40),
                ("submap_overlapping_segments", 20),
                ("ego_robot_name", ""),
                ("robot_names", []),
                ("robot_flu_ref_frames", []),
                ("robot_odom_frames", []),
            ]
        )
        
        config_path = self.get_parameter("config_path").value
        self.submap_num_segments = self.get_parameter("submap_num_segments").value
        self.submap_overlapping_segments = self.get_parameter("submap_overlapping_segments").value
        self.ego_robot = self.get_parameter("ego_robot_name").value
        self.robots = self.get_parameter("robot_names").value
        self.robot_flu_ref_frames = self.get_parameter("robot_flu_ref_frames").value
        self.robot_odom_frames = self.get_parameter("robot_odom_frames").value

        # check params
        assert self.ego_robot != "", "ERROR: ego_robot_name param must be set."
        assert len(self.robots) > 0, "ERROR: robot_names param must be set."
        assert len(self.robot_flu_ref_frames) == len(self.robots), "ERROR: robot_flu_ref_frames param must be set."
        assert len(self.robot_odom_frames) == len(self.robots), "ERROR: robot_odom_frames param must be set."
        assert self.ego_robot in self.robots, "ERROR: ego_robot_name must be in robot_names."

        # internal variables
        self.segment_queues = {robot: SegmentQueue(self.submap_num_segments, self.submap_overlapping_segments) for robot in self.other_robots}
        self.submaps = {robot: [] for robot in self.other_robots}

        if config_path == "":
            submap_align_params = SubmapAlignParams()
        else:
            submap_align_params = SubmapAlignParams.from_yaml(config_path)

        self.roman_reg = submap_align_params.get_object_registration()
        
        self.setup_ros()
        
    def setup_ros(self):
        
        # ros subscribers
        self.segment_subs = [
            self.create_subscription(roman_msgs.Segment, f"/{robot}/roman/segment_updates", 
                                     lambda msg: self.seg_cb(msg, robot), 20)
        for robot in self.other_robots]

        # tf buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ros publishers
        # TODO: setup publisher: https://github.com/MIT-SPARK/pose_graph_tools/tree/ros2/pose_graph_tools_msgs/msg

    def seg_cb(self, seg_msg: roman_msgs.Segment, robot: str):
        """
        Callback function for segment messages.
        """
        segment = msg_to_segment(seg_msg)
        seg_update_res = self.segment_queues[robot].update(segment)
        if not seg_update_res.submap_created:
            return
        
        # create submap
        submap_ref_time = np.mean([np.mean([seg.first_seen, seg.last_seen]) for seg in seg_update_res.submap_segments])
        # try:
        T_odom_flu_msg = self.tf_buffer.lookup_transform(self.robot_odom_frames[robot], self.robot_flu_ref_frames[robot], submap_ref_time, rclpy.duration.Duration(seconds=2.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        #     self.get_logger().warning("tf lookup failed")
        #     print(ex)
        #     return
        T_odom_flu = rnp.numpify(T_odom_flu_msg.transform).astype(np.float64)
        submap_id = len(self.submaps[robot])
        
        submap = Submap(
            id=submap_id,
            time=submap_ref_time,
            pose_flu=T_odom_flu,
            segments=[],
            segment_frame='submap_gravity_aligned'
        )
        for seg_i in seg_update_res.submap_segments:
            segment = deepcopy(seg_i)
            segment.transform(submap.pose_gravity_aligned)
            submap.segments.append(segment)

        self.submaps[robot].append(submap)

        # run submap registration
        if robot == self.ego_robot:
            for r2 in self.robots:
                self.run_submap_registration(submap, robot, r2)
        else:
            # run registration with ego robot
            self.run_submap_registration(submap, robot, self.ego_robot)
            

    def run_submap_registration(self, submap: Submap, robot: str, other_robot: str):
        for submap2 in self.submaps[other_robot]:
            # check if submap2 is already registered
            if submap2.id == submap.id and robot == other_robot:
                continue
            
            # run registration
            try:
                associations = self.roman_reg.register(submap.segments, submap2.segments)
            except InsufficientAssociationsException:
                continue

            T_submap1_submap2 = self.roman_reg.T_align(
                submap.segments, submap2.segments, associations)

            print(f"Found {len(associations)} associations between submaps {submap.id} and {submap2.id}.")
            # TODO: publish loop closure


def main():

    rclpy.init()
    node = ROMANLoopClosureNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

