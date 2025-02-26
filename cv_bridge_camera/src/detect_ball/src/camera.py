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
import numpy as np


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.center_pub = rospy.Publisher("centers",String)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    pi = 3.14159
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    cv2.imshow("Ref Image window", cv_image)
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
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
    contours_circlescore = [None]*len(contours)
    circle_centers = []
    circle_radii = []
    circle_circlescore = []
    circle_contours = []
    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
        contours_circlescore[i] = cv2.contourArea(c)/(pi*radius[i]**2)
        if contours_circlescore[i] > 0.5 and radius[i] > 5:
            circle_centers.append(centers[i])
            circle_radii.append(radius[i])
            circle_circlescore.append(contours_circlescore[i])
            circle_contours.append(c)
    if(circle_radii != []):
        max_rad = max(circle_radii)
        max_index = circle_radii.index(max_rad)
        max_center = circle_centers[max_index]
        max_circlescore = circle_circlescore[max_index]
        cv2.circle(cv_image, (int(max_center[0]), int(max_center[1])), int(max_rad), 255)
        cv2.circle(cv_image, (int(max_center[0]), int(max_center[1])), int(5), 50)
        self.center_pub.publish("Center of Largest Circular Contour: ("+ str(max_center[0]) +", " +str(max_center[1])+") \n Circle Score: " + str(max_circlescore))
    """detected_circles = cv2.HoughCircles(mask,  
                cv2.HOUGH_GRADIENT, 1, 2, param1 = 50, 
            param2 = 30, minRadius = 1, maxRadius = 40)
    if detected_circles is not None: 
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
    
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2] 
    
            # Draw the circumference of the circle. 
            cv2.circle(mask, (a, b), r, (0, 255, 0), 2) 
    
            # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(mask, (a, b), 1, (0, 0, 255), 3) """
    cv2.drawContours(cv_image, circle_contours, -1, (0, 255, 0), 3)
    cv2.imshow("Mask window", mask)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

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
