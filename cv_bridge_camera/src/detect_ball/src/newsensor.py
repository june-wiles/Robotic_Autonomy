#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from visualization_msgs.msg import Marker
from std_msgs.msg import String


class BallDetectionNode:
    def __init__(self):
        rospy.init_node('ball_detection', anonymous=True)

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to both Camera and LiDAR Topics
        self.sensor_sub = rospy.Subscriber("/d400/aligned_depth_to_color/image_raw", LaserScan, self.sensor_callback)

        # Publisher for Image and Marker (Visualization)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.center_pub = rospy.Publisher("centers", String, queue_size=10)
        self.marker_pub = rospy.Publisher("detected_region", Marker, queue_size=10)

        # TF2 Buffer for Transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Sensor Model Parameters
        self.dc_min = 1.5  # Minimum detection distance
        self.dc_max = 4.5  # Maximum detection distance
        self.w = 7  # Sigmoid slope

    
    def sensor_callback(self, data):
        print("Running Sensor Callback")
        detected_objects = self.process_sensor_data(data)

        if detected_objects:
            for obj in detected_objects:
                world_coords = self.transform_to_world(obj)
                probability = self.compute_probability(world_coords[0])
                self.publish_marker(world_coords, probability)

    

    def process_sensor_data(self, scan_data):
        detected_objects = []
        print(scan_data.ranges)
        for i, distance in enumerate(scan_data.ranges):
            if scan_data.range_min < distance < scan_data.range_max:
                angle = scan_data.angle_min + i * scan_data.angle_increment
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                detected_objects.append((x, y, 0.1))
        return detected_objects

    def compute_probability(self, distance):
        pod = 1 / (1 + np.exp(-self.w * (distance - self.dc_min))) - 1 / (1 + np.exp(-self.w * (distance - self.dc_max)))
        return max(0, min(pod, 1))

    def transform_to_world(self, detection):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            world_x = detection[0] + trans.transform.translation.x
            world_y = detection[1] + trans.transform.translation.y
            return world_x, world_y, detection[2]
        except Exception as e:
            rospy.logerr("TF2 transform error: %s", str(e))
            return detection

    def publish_marker(self, position, probability):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.1
        marker.scale.x = marker.scale.y = marker.scale.z = position[2] * 2
        marker.color.a = 1.0
        marker.color.r = 1.0 - probability
        marker.color.g = probability
        marker.color.b = 0.0
        self.marker_pub.publish(marker)


if __name__ == '__main__':
    try:
        node = BallDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
