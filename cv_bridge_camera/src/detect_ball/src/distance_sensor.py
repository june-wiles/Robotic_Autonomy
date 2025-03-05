#!/usr/bin/env python
  import rospy
  from std_msgs.msg import Float32
  from publisher_controlvalue import ControlValuePub

  class MySubscriber(object):

   def __init__(self):
    
    self.encoder = 0.0
    self.desiredcounts = 0.0


    rospy.Subscriber('/centers', Float32, self.desiredcounts_callback)
    rospy.Subscriber('/FILLER', Float32, self.encoder_callback)


def encoder_callback(self,msg):
    self.centers = msg.data

def desiredcounts_callback(self,msg):
    detected_objects = self.centers

        if detected_objects:
            for obj in detected_objects:
                world_coords = self.transform_to_world(obj)
                probability = self.compute_probability(world_coords[0])
                self.publish_marker(world_coords, probability)


def loop(self):
    rospy.logwarn("Starting Loop...")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('subscriber_node', anonymous=True, log_level=rospy.WARN)
    my_subs = MySubscriber()
    my_subs.loop()
