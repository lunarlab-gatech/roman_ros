#!/usr/bin/env python3

import numpy as np
from numpy import random
import os
import cv2 as cv
import struct
import pickle
import time
import signal

# ROS imports
import rclpy
from rclpy.node import Node
import cv_bridge
import message_filters
import ros2_numpy as rnp
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
from rclpy.executors import MultiThreadedExecutor

# ROS msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import roman_msgs.msg as roman_msgs
from ros_system_monitor_msgs.msg import NodeInfoMsg
from sklearn.decomposition import PCA

# robot_utils
from robotdatapy.camera import CameraParams

# ROMAN
from roman.map.fastsam_wrapper import FastSAMWrapper
from roman.map.mapper import Mapper, MapperParams
from roman.map.map import ROMANMap
from roman.object.segment import Segment
from roman.viz import visualize_map_on_img

# relative
from roman_ros2.utils import observation_from_msg, segment_to_msg, time_stamp_to_float

class RomanMapNode(Node):

    def __init__(self):
        super().__init__('roman_map_node')
        self.up = True

        # ros params
        self.declare_parameters(
            namespace='',
            parameters=[
                ("robot_id", 0),
                ("config_path", ""),
                ("visualize", False),
                ("output_roman_map", ""),
                ("map_frame_id", "map"),
                ("object_ref", "bottom_middle"),
                ("T_camera_flu", np.eye(4).reshape(-1).tolist()),
                ("publish_active_segments", False),
                ("nickname", "roman_map"),
                ("viz_num_objs", 20),
                ("viz_pts_per_obj", 250),
                ("viz_min_viz_dt", 2.0),
                ("viz_rotate_img", ""),
                ("viz_pointcloud", False),
                ("wait_for_tf_time", 1.0)
            ]
        )

        self.robot_id = self.get_parameter("robot_id").value
        self.visualize = self.get_parameter("visualize").value
        self.output_file = self.get_parameter("output_roman_map").value
        self.object_ref = self.get_parameter("object_ref").value
        T_camera_flu = self.get_parameter("T_camera_flu").value
        T_camera_flu = np.array(T_camera_flu).reshape(4, 4)
        self.publish_active_segments = self.get_parameter("publish_active_segments").value
        self.nickname = self.get_parameter("nickname").value
        config_path = self.get_parameter("config_path").value
        self.wait_for_tf_time = self.get_parameter("wait_for_tf_time").value

        if self.visualize:
            self.map_frame_id = self.get_parameter("map_frame_id").value
            self.viz_num_objs = self.get_parameter("viz_num_objs").value
            self.viz_pts_per_obj = self.get_parameter("viz_pts_per_obj").value
            self.min_viz_dt = self.get_parameter("viz_min_viz_dt").value
            self.viz_rotate_img = self.get_parameter("viz_rotate_img").value
            self.viz_pointcloud = self.get_parameter("viz_pointcloud").value
            if self.viz_rotate_img == "":
                self.viz_rotate_img = None

            self.pc_fields = [
                sensor_msgs.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='rgb', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ]

            self.pca = PCA(n_components=3)

        if self.output_file != "":
            self.output_file = os.path.expanduser(self.output_file)
            self.pose_history = []
            self.time_history = []
            self.get_logger().info(f"Output file: {self.output_file}")
        else:
            self.output_file = None

        # mapper
        self.status_pub = self.create_publisher(NodeInfoMsg, "roman/roman_map/status", qos_profile=QoSProfile(depth=10))
        self.log_and_send_status("RomanMapNode setting up mapping...", status=NodeInfoMsg.STARTUP)
        self.log_and_send_status("RomanMapNode waiting for color camera info messages...", status=NodeInfoMsg.STARTUP)
        color_info_msg = self._wait_for_message("color/camera_info", sensor_msgs.CameraInfo)
        color_params = CameraParams.from_msg(color_info_msg)
        self.log_and_send_status("RomanMapNode received for color camera info messages...", status=NodeInfoMsg.STARTUP)

        if config_path != "":
            mapper_params = MapperParams.from_yaml(config_path)
        else:
            mapper_params = MapperParams()
        self.mapper = Mapper(
           mapper_params,
           camera_params=color_params
        )
        self.mapper.set_T_camera_flu(T_camera_flu)

        self.setup_ros()

    def setup_ros(self):
        
        # ros publishers
        self.segments_pub = self.create_publisher(roman_msgs.Segment, "roman/segment_updates", qos_profile=10)
        self.pulse_pub = self.create_publisher(std_msgs.Empty, "roman/pulse", qos_profile=10)

        # ros subscribers
        self.create_subscription(roman_msgs.ObservationArray, "roman/observations", self.obs_cb, 10)

        # visualization
        if self.visualize:
            self.last_viz_t = -np.inf
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            
            self.create_subscription(sensor_msgs.Image, "color/image_raw", self.viz_cb, 10)
            self.bridge = cv_bridge.CvBridge()
            self.annotated_img_pub = self.create_publisher(sensor_msgs.Image, "roman/annotated_img", qos_profile=10)
            self.object_points_pub = self.create_publisher(sensor_msgs.PointCloud2, "roman/object_points", qos_profile=10)

        self.log_and_send_status("ROMAN Map Node setup complete.", status=NodeInfoMsg.STARTUP)
        self.log_and_send_status("Waiting for observation.", status=NodeInfoMsg.STARTUP)

    def obs_cb(self, obs_array_msg):
        """
        Triggered by incoming observation messages
        """
        if not self.up:
            return

        # publish pulse
        map_size = len(self.mapper.segments) + \
                len(self.mapper.inactive_segments) + \
                len(self.mapper.segment_graveyard)
        self.log_and_send_status(f"Map size: {map_size}")
        self.pulse_pub.publish(std_msgs.Empty())
        
        if len(obs_array_msg.observations) == 0:
            return
        
        observations = []
        for obs_msg in obs_array_msg.observations:
            observations.append(observation_from_msg(obs_msg))

        t = observations[0].time
        assert all([obs.time == t for obs in observations])

        inactive_ids = [segment.id for segment in self.mapper.inactive_segments]
        self.mapper.update(time_stamp_to_float(obs_array_msg.header.stamp), rnp.numpify(obs_array_msg.pose), observations)
        updated_inactive_ids = [segment.id for segment in self.mapper.inactive_segments]
        new_inactive_ids = [seg_id for seg_id in updated_inactive_ids if seg_id not in inactive_ids]

        # publish segments
        segment: Segment
        for segment in self.mapper.inactive_segments:
            # TODO: this does not include a way to notify of a deleted segment
            if segment.last_seen == t or segment.id in new_inactive_ids:
                self.publish_segment(segment)
        if self.publish_active_segments:
            for segment in self.mapper.segments:
                self.publish_segment(segment)
        
        if self.output_file is not None:
            self.pose_history.append(rnp.numpify(obs_array_msg.pose_flu))
            self.time_history.append(t)

        # Publish Point Clouds
        if self.visualize:
            # Get the self.viz_num_objs most recently seen segments
            most_recently_seen_segments = sorted(
                self.mapper.segments + self.mapper.inactive_segments + self.mapper.segment_graveyard, 
                key=lambda x: x.last_seen if len(x.points) > 10 else 0, reverse=True)#[:self.viz_num_objs]
            
            total_points = np.zeros([0, 4], dtype=np.float32)

            if len(most_recently_seen_segments) >= 3:
                # Extract semantic descriptors for each segment and fit_transform with pca
                semantic_descriptors = np.array([segment.semantic_descriptor for segment in most_recently_seen_segments])
                pca_array = self.pca.fit_transform(semantic_descriptors)
                min_val = pca_array.min(axis=0)
                max_val = pca_array.max(axis=0)
                scaled = (pca_array - min_val) / (max_val - min_val + 1e-8)  # avoid div0
                colors_unpacked = (scaled * 255).astype(np.uint8)
                
                for i, segment in enumerate(most_recently_seen_segments):
                    color_unpacked = colors_unpacked[i]
                    color_raw = int(color_unpacked[0]*256**2 + color_unpacked[1]*256 + color_unpacked[2])
                    color_packed = struct.unpack('f', struct.pack('I', color_raw))[0]
                    rgb_uint32 = struct.unpack('I', struct.pack('f', color_packed))[0]
                    r = (rgb_uint32 >> 16) & 0xFF
                    g = (rgb_uint32 >> 8) & 0xFF
                    b = rgb_uint32 & 0xFF
                    
                    # Sample 50% points from the segment randomly
                    points = segment.points
                    sampled_points = np.random.choice(len(points), int(len(points) / 2), replace=False)
                    points = [points[i] for i in sampled_points]
                    points = np.concatenate((points, np.full((len(points), 1), color_packed)), axis=1)
                    total_points = np.concatenate((total_points, np.array(points)), axis=0)

                # If there aren't any points, return
                if len(total_points.shape) == 1:
                    return
                
                # Convert nddarray to iterable list of tuples
                points = []
                for row in total_points:
                    points.append(tuple(row))

                # Create a PointCloud2 message
                header = std_msgs.Header()
                header.stamp = obs_array_msg.header.stamp
                header.frame_id = self.map_frame_id
                cloud_msg = pc2.create_cloud(header, self.pc_fields, points)
                self.object_points_pub.publish(cloud_msg)

    def publish_segment(self, segment: Segment):
        if self.object_ref == 'bottom_middle':
            segment.set_center_ref('bottom_middle')
        self.segments_pub.publish(segment_to_msg(self.robot_id, segment))

    def viz_cb(self, img_msg):
        """
        Triggered by incoming odometry and image messages
        """
        if not self.up:
            return

        t = time_stamp_to_float(img_msg.header.stamp)
        if t - self.last_viz_t < self.min_viz_dt:
            return
        else:
            self.last_viz_t = t
            
        cam_frame_id = img_msg.header.frame_id
        
        try:
            transform_stamped_msg = self.tf_buffer.lookup_transform(self.map_frame_id, cam_frame_id, img_msg.header.stamp, rclpy.duration.Duration(seconds=self.wait_for_tf_time))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warning("tf lookup failed; failed to publish annotated image")
            self.get_logger().warning(str(ex))
            return

        pose = rnp.numpify(transform_stamped_msg.transform).astype(np.float64)

        # conversion from ros msg to cv img
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        img = visualize_map_on_img(t, pose, img, self.mapper)
            
        if self.viz_rotate_img is not None:
            if self.viz_rotate_img == "CW":
                img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
            elif self.viz_rotate_img == "CCW":
                img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
            elif self.viz_rotate_img == "180":
                img = cv.rotate(img, cv.ROTATE_180)
        
        annotated_img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        annotated_img_msg.header = img_msg.header
        self.annotated_img_pub.publish(annotated_img_msg)
        
    def log_and_send_status(self, note, status=NodeInfoMsg.NOMINAL):
        """
        Log a message and send it to the status topic.
        """
        self.get_logger().info(note)
        status_msg = NodeInfoMsg()
        status_msg.nickname = self.nickname
        status_msg.node_name = self.get_fully_qualified_name()
        status_msg.status = status
        status_msg.notes = note
        self.status_pub.publish(status_msg)
        return
    
    def shutdown(self):
        if self.output_file is None:
            print(f"No file to save to.")
        if self.output_file is not None:
            self.up = False
            print(f"Saving map to {self.output_file}...")
            time.sleep(1.0)
            self.mapper.make_pickle_compatible()
            pkl_file = open(self.output_file, 'wb')
            pickle.dump(self.mapper.get_roman_map(), pkl_file, -1)
            pkl_file.close()
        self.destroy_node()

    def _wait_for_message(self, topic, msg_type):
        """
        Wait for a message on topic of type msg_type
        """
        subscription = self.create_subscription(msg_type, topic, self._wait_for_message_cb, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        
        self._wait_for_message_msg = None
        while self._wait_for_message_msg is None:
            rclpy.spin_once(self)
        msg = self._wait_for_message_msg
        # subscription.destroy()

        return msg
    
    def _wait_for_message_cb(self, msg):
        self._wait_for_message_msg = msg
        return

def main():

    rclpy.init()
    node = RomanMapNode()

    # signal.signal(signal.SIGINT, node.shutdown)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()