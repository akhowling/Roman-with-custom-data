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
from roman.map.map import Submap, ROMANMap, submaps_from_roman_map, SubmapParams

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
                ("ego_id", 0),
                ("ego_name", ""),
                ("ego_flu_ref_frame", ""),
                ("ego_odom_frame", ""),
                ("team_ids", []),
                ("team_names", []),
                ("team_flu_ref_frames", []),
                ("team_odom_frames", []),
                ("prior_session_maps", []),
                ("prior_session_ids", [])
            ]
        )
        
        config_path = self.get_parameter("config_path").value
        self.submap_num_segments = self.get_parameter("submap_num_segments").value
        self.submap_overlapping_segments = self.get_parameter("submap_overlapping_segments").value
        self.ego_id = self.get_parameter("ego_id").value
        self.ego_name = self.get_parameter("ego_name").value
        self.ego_flu_ref_frame = self.get_parameter("ego_flu_ref_frame").value
        self.ego_odom_frame = self.get_parameter("ego_odom_frame").value
        self.team_ids = self.get_parameter("team_ids").value
        self.team_names = self.get_parameter("team_names").value
        self.team_flu_ref_frames = self.get_parameter("team_flu_ref_frames").value
        self.team_odom_frames = self.get_parameter("team_odom_frames").value
        self.prior_session_maps = self.get_parameter("prior_session_maps").value
        self.prior_session_ids = self.get_parameter("prior_session_ids").value

        # organize multi-robot metadata
        self.live_names = [self.ego_name] + self.team_names
        self.live_ids = [self.ego_id] + self.team_ids
        self.all_ids = self.live_ids + self.prior_session_ids

        self.flu_ref_frames = {robot_id: robot_flu_ref_frame 
            for robot_id, robot_flu_ref_frame in 
            zip(self.live_ids, [self.ego_flu_ref_frame] + self.team_flu_ref_frames)}
        self.odom_frames = {robot_id: robot_odom_frame
            for robot_id, robot_odom_frame in 
            zip(self.live_ids, [self.ego_odom_frame] + self.team_odom_frames)}

        # check params
        assert self.ego_name != "", "ERROR: ego_robot_name param must be set."
        assert self.ego_flu_ref_frame != "", "ERROR: ego_robot_flu_ref_frame param must be set."
        assert self.ego_odom_frame != "", "ERROR: ego_robot_odom_frame param must be set."
        assert len(self.team_ids) == len(self.team_names), "ERROR: team_ids and team_names must be the same length."
        assert len(self.team_flu_ref_frames) == len(self.team_ids), "ERROR: team_flu_ref_frames param must be set."
        assert len(self.team_odom_frames) == len(self.team_ids), "ERROR: team_odom_frames param must be set."
        assert len(self.prior_session_maps) == len(self.prior_session_ids), \
            "ERROR: prior_session_maps and prior_session_ids must be the same length."
        assert len(set(self.all_ids)) == len(self.all_ids), \
            "ERROR: all robot ids must be unique. Check team_ids and prior_session_ids."

        # internal variables

        self.segment_queues = {
            live_id: SegmentQueue(self.submap_num_segments, self.submap_overlapping_segments)
        for live_id in self.live_ids}
        self.submaps = {live_id: [] for live_id in self.live_ids}

        if config_path == "":
            submap_align_params = SubmapAlignParams()
        else:
            submap_align_params = SubmapAlignParams.from_yaml(config_path)

        self.roman_reg = submap_align_params.get_object_registration()

        # load prior session maps
        self.get_logger().info("Loading prior maps.")
        for prior_id, map_path in zip(self.prior_session_ids, self.prior_session_maps):
            prior_map = ROMANMap.from_pickle(map_path)
            submap_params = SubmapParams.from_submap_align_params(submap_align_params)
            submaps = submaps_from_roman_map(prior_map, submap_params)
            self.submaps[prior_id] = submaps
            self.get_logger().info(f"Loaded prior session map for robot {prior_id} from {map_path}.")
        
        self.setup_ros()
        
    def setup_ros(self):
        
        # ros subscribers
        self.segment_subs = [
            self.create_subscription(roman_msgs.Segment, f"/{robot_name}/roman/segment_updates", 
                                     lambda msg: self.seg_cb(msg, robot_id), 20)
        for robot_name, robot_id in zip(self.live_names, self.live_ids)]

        # tf buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ros publishers
        # TODO: setup publisher: https://github.com/MIT-SPARK/pose_graph_tools/tree/ros2/pose_graph_tools_msgs/msg

    def seg_cb(self, seg_msg: roman_msgs.Segment, robot_id: int):
        """
        Callback function for segment messages.
        """
        segment = msg_to_segment(seg_msg)
        seg_update_res = self.segment_queues[robot_id].update(segment)
        if not seg_update_res.submap_created:
            return
        
        # create submap
        submap_ref_time = np.mean([np.mean([seg.first_seen, seg.last_seen]) for seg in seg_update_res.submap_segments])
        # try:
        T_odom_flu_msg = self.tf_buffer.lookup_transform(self.odom_frames[robot_id], 
            self.flu_ref_frames[robot_id], submap_ref_time, rclpy.duration.Duration(seconds=2.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        #     self.get_logger().warning("tf lookup failed")
        #     print(ex)
        #     return
        T_odom_flu = rnp.numpify(T_odom_flu_msg.transform).astype(np.float64)
        submap_id = len(self.submaps[robot_id])
        
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

        self.submaps[robot_id].append(submap)

        # run submap registration
        if robot_id == self.ego_id:
            for r2 in self.all_ids:
                self.run_submap_registration(submap, robot_id, r2)
        else:
            # run registration with ego robot
            self.run_submap_registration(submap, robot_id, self.ego_robot)
            

    def run_submap_registration(self, submap: Submap, robot_id: int, other_id: int):
        for submap2 in self.submaps[other_id]:
            # check if submap2 is already registered
            if submap2.id == submap.id and robot_id == other_id:
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

