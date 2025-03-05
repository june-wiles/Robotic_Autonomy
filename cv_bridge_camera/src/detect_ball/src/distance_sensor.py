#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from publisher_controlvalue import ControlValuePub
import numpy as np

class MySubscriber(object):

    def __init__(self):
        
        self.encoder = 0.0
        self.desiredcounts = 0.0


        rospy.Subscriber('/centers', Float32, self.centers_callback)
        rospy.Subscriber('/FILLER', Float32, self.distance_callback)


    def centers_callback(self,msg):
        self.centers = msg.data

    def distance_callback(self,msg):
        detected_objects = self.centers
        print(msg.shape)
        if detected_objects:
            for obj in detected_objects:
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
