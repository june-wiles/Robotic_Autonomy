#!/usr/bin/env python3
from __future__ import print_function

import roslib
roslib.load_manifest('detect_ball')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker

import numpy as np


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.center_pub = rospy.Publisher("centers",String)
    self.marker_pub = rospy.Publisher("detected_region", Marker, queue_size=10)
    self.bridge = CvBridge()
    self.rgb_image_sub = rospy.Subscriber("/d400/color/image_raw",Image,self.rgb_callback)

  def rgb_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    depth_array = np.array(cv_image, dtype = np.float32)
    (rows,cols,channels) = cv_image.shape
    cv2.imshow("Ref Image window", cv_image)
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #print(hsv_image.shape)
    hue_min = 80*180/360
    hue_max = 150*180/360
    sat_min = 60
    sat_max = 255
    value_min = 60
    value_max = 255
    lower_bound = np.array([hue_min, sat_min, value_min])
    upper_bound = np.array([hue_max, sat_max, value_max])
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = [None]*len(contours)
    radius = [None]*len(contours)
    contours_poly = [None]*len(contours)
    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
    # TODO add circlescores back in (deleted accidently when merging without github)
    max_rad = max(radius)
    max_index = radius.index(max_rad)
    max_center = centers[max_index]
    cv2.circle(cv_image, (int(max_center[0]), int(max_center[1])), int(max_rad), 255)
    cv2.circle(cv_image, (int(max_center[0]), int(max_center[1])), int(5), 50)
    self.center_pub.publish("Center of Largest Contour: ("+ str(max_center[0]) +", " +str(max_center[1])+")")

    cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)
    cv2.imshow("Mask window", mask)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    ball_depth = depth_array[int(max_center[1])][int(max_center[0])]
    # angle calculations
    #TODO determine camera FOV
    camera_FOV = 60*np.pi/180 #deg
    #PLACEHOLDER
    # pixels from center * degrees per pixel = angle from center
    # negative angle is left, positive angle is right
    image_width = depth_array.shape[0]
    angle_xy_plane = (max_center[1] - image_width/2) * (camera_FOV/image_width)
    #TODO filter angle, depth pairs outside of detectable region
    # x = r*sin(theta) because theta is measured from the y axis
    x = ball_depth * np.sin(angle_xy_plane)
    y = ball_depth * np.cos(angle_xy_plane)
    mean_position = [x, y]
    phi = angle_xy_plane - np.pi/2 #rad

    #for covariance transformations
    R_t_to_s = [[cos(phi), sin(phi)],[-sin(phi), cos(phi)]]
    #TODO add covariance calculations
    #robot position
    robot_angle = 0
    robot_pos = [0, 0]
    R_s_to_G = [[cos(robot_angle), sin(robot_angle)],[-sin(robot_angle), cos(robot_angle)]]
    XY_Global = np.matmul( R_s_to_G, mean_position) + robot_pos

    # TODO fix marker code
    marker = Marker()
    marker.header.frame_id = "d400_link"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = ball_depth[0]
    marker.pose.position.y = ball_depth[1]
    marker.pose.position.z = 0.1
    marker.scale.x = marker.scale.y = marker.scale.z = ball_depth[2] * 2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 0.0
    self.marker_pub.publish(marker)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
