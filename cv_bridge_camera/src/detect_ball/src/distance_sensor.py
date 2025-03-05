#!/usr/bin/env python3
from __future__ import print_function
import rospy
from std_msgs.msg import Float32
import numpy as np


import roslib
roslib.load_manifest('detect_ball')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
class MySubscriber(object):

    def __init__(self):
        
        self.encoder = 0.0
        self.desiredcounts = 0.0


        rospy.Subscriber('/centers', String, self.centers_callback)
        rospy.Subscriber('/d400/aligned_depth_to_color/image_raw', Image, self.distance_callback)


    def centers_callback(self,msg):
        self.centers = msg.data

    def distance_callback(self,msg):
        cv_bridge = CvBridge()
        try:
            depth_img = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        depth_array = np.array(depth_img, dtype = np.float32)
        detected_objects = self.centers
        if detected_objects:
            
            world_coords = self.transform_to_world(obj)
            probability = self.compute_probability(world_coords[0])
            self.publish_marker(world_coords, probability)
    def transform_to_world(self, detection):
        """ Transforms detected coordinates to world frame using TF2. """
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            world_x = detection[0] + trans.transform.translation.x
            world_y = detection[1] + trans.transform.translation.y
            return world_x, world_y, detection[2]
        except Exception as e:
            rospy.logerr("TF2 transform error: %s", str(e))
            return detection

    def loop(self):
        rospy.logwarn("Starting Loop...")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('subscriber_node', anonymous=True, log_level=rospy.WARN)
    my_subs = MySubscriber()
    my_subs.loop()
