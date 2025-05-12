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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_updater import Updater, DiagnosticStatusWrapper
import tf2_ros
from rclpy.executors import MultiThreadedExecutor

# ROS msgs
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import roman_msgs.msg as roman_msgs
from ros_system_monitor_msgs.msg import NodeInfoMsg

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
                ("nickname", "fastsam"),
                ("wait_for_tf_time", 2.0),
                # ("fastsam_viz", False),
            ]
        )

        self.cam_frame_id = None
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.odom_base_frame_id = self.get_parameter("odom_base_frame_id").value
        self.min_dt = self.get_parameter("min_dt").value
        self.nickname = self.get_parameter("nickname").value
        config_path = self.get_parameter("config_path").value
        self.wait_for_tf_time = self.get_parameter("wait_for_tf_time").value

        # self.visualize = self.get_parameter("fastsam_viz").value
        self.visualize = False # TODO: is supporting this helpful?
        self.last_t = -np.inf
        self.last_diff = None

        # FastSAM set up after camera info can be retrieved
        self.status_pub = self.create_publisher(NodeInfoMsg, "roman/fastsam/status", qos_profile=QoSProfile(depth=10))
        self.log_and_send_status("Waiting for camera info messages...", status=NodeInfoMsg.STARTUP)
        depth_info_msg = self._wait_for_message("depth/camera_info", sensor_msgs.CameraInfo)
        self.log_and_send_status("Received for depth camera info messages...", status=NodeInfoMsg.STARTUP)
        self.log_and_send_status("Waiting for color camera info messages...", status=NodeInfoMsg.STARTUP)
        color_info_msg = self._wait_for_message("color/camera_info", sensor_msgs.CameraInfo)
        self.log_and_send_status("Received for color camera info messages...", status=NodeInfoMsg.STARTUP)
        self.depth_params = CameraParams.from_msg(depth_info_msg)
        color_params = CameraParams.from_msg(color_info_msg)

        # fastsam wrapper
        if config_path != "":
            fastsam_params = FastSAMParams.from_yaml(config_path)
        else:
            fastsam_params = FastSAMParams()
            
        self.fastsam = FastSAMWrapper.from_params(fastsam_params, self.depth_params)

        self.setup_ros()

    def _wait_for_message(self, topic, msg_type):
        """
        Wait for a message on topic of type msg_type
        """
        subscription = self.create_subscription(msg_type, topic, self._wait_for_message_cb, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        
        self._wait_for_message_msg = None
        while self._wait_for_message_msg is None:
            rclpy.spin_once(self)
        msg = self._wait_for_message_msg

        return msg
    
    def _wait_for_message_cb(self, msg):
        self._wait_for_message_msg = msg
        return

    def setup_ros(self):
        
        # ros publishers
        self.obs_pub = self.create_publisher(roman_msgs.ObservationArray, "roman/observations", qos_profile=QoSProfile(depth=10))

        if self.visualize:
            self.ptcld_pub = self.create_publisher(sensor_msgs.PointCloud, "roman/observations/ptcld")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ros subscribers
        subs = [
            message_filters.Subscriber(self, sensor_msgs.Image, "color/image_raw"),
            message_filters.Subscriber(self, sensor_msgs.Image, "depth/image_raw"),
        ]
        self.ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.1)
        self.ts.registerCallback(self.cb) # registers incoming messages to callback

        # Setup Diagnostics Publisher
        self.updater = Updater(self)
        self.updater.setHardwareID(self.get_fully_qualified_name())
        self.updater.add("Segmentation", self.diagnostic_callback)
        self.updater_timer = self.create_timer(1.0, self.updater.update)

        self.log_and_send_status("FastSAM node setup complete.")

    def diagnostic_callback(self, stat: DiagnosticStatusWrapper):
        if self.last_diff is None:
            stat.summary(DiagnosticStatusWrapper.WARN, "No messages received")
        elif self.last_diff >= 4 * self.min_dt:
            stat.summary(DiagnosticStatusWrapper.ERROR, f"Difference between processed messages is {self.last_diff:.2f} seconds, which is at least four times min processing time of {self.min_dt:.2f} seconds")
        elif self.last_diff  >= 2 * self.min_dt:
            stat.summary(DiagnosticStatusWrapper.WARN, f"Difference between processed messages is {self.last_diff:.2f} seconds, which is at least twice min processing time of {self.min_dt:.2f} seconds")
        else:
            stat.summary(DiagnosticStatusWrapper.OK, "Images segmented and observations published successfully")
        stat.add("Last message time", str(self.last_t))
        stat.add("Last difference", str(self.last_diff))
        stat.add("Min processing time", str(self.min_dt))
        return stat

    def cb(self, *msgs):
        """
        This function gets called every time synchronized odometry, image message, and 
        depth image message are received.
        """
        
        img_msg, depth_msg = msgs
        if self.cam_frame_id is None:
            self.cam_frame_id = img_msg.header.frame_id

        # check that enough time has passed since last observation (to not overwhelm GPU)
        t = rclpy.time.Time.from_msg(img_msg.header.stamp).nanoseconds * 1e-9
        if t - self.last_t < self.min_dt:
            return
        else:
            self.last_diff = t - self.last_t
            self.last_t = t

        try:
            # self.tf_buffer.waitForTransform(self.map_frame_id, self.cam_frame_id, img_msg.header.stamp, rospy.Duration(0.5))
            transform_stamped_msg = self.tf_buffer.lookup_transform(self.map_frame_id, self.cam_frame_id, img_msg.header.stamp, rclpy.duration.Duration(seconds=self.wait_for_tf_time))
            flu_transformed_stamped_msg = self.tf_buffer.lookup_transform(self.map_frame_id, self.odom_base_frame_id, img_msg.header.stamp, rclpy.duration.Duration(seconds=self.wait_for_tf_time))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.log_and_send_status("tf lookup failed", status=NodeInfoMsg.WARNING)
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
        self.log_and_send_status("Processed callback", status=NodeInfoMsg.NOMINAL)

        # if self.visualize:
        #     self.pub_ptclds(observations, img_msg.header, depth)

        return
    
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

