#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge ,CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class LineFollower(object):
    def __init__(self):
        self.cvBridge = CvBridge()
        self.sub_image_original = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback, queue_size = 1)
        self.cmd_vel_pub = rospy.Publisher('/ackermann_steering_controller/cmd_vel',Twist,queue_size = 10)

    # ROS Image's topic callback function
    def camera_callback(self, image_msg): 
        try:
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
                print(e)
        cv2.imshow("cv_imcode_image",cv_image)

        height, width, channels = cv_image.shape
        descentre = 50
        rows_to_watch = 100
        #crop_img = cv_image
        crop_img = cv_image[(height)/4 + descentre:(height)/4 + (descentre+rows_to_watch)][1:width]

        #convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img,cv2.COLOR_BGR2HSV)
        cv2.imshow("HSV",hsv)

        # yellow colour in HSV
        lower_yellow = np.array([0,225,225])
        upper_yellow = np.array([91,255,255])

        #Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv,lower_yellow,upper_yellow)
        cv2.imshow("MASK",mask)
        
        #Bitwise-and musk and original image
        res = cv2.bitwise_and(crop_img,crop_img,mask = mask)
        cv2.imshow("RES",res)
  
        #Calculate centroid of the blob of binary image using imageMoments
        m = cv2.moments(mask,False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
                cx, cy = height/2, width/2
      
        # Draw the centroid in the resultut image
        cv2.circle(res , (int(cx) , int(cy)) , 20 , (255,255,0) , 2)
        cv2.waitKey(30)

        #print(cx)
        error_x = cx - width / 2
        
        twist_object = Twist()
        twist_object.linear.x = 2
        twist_object.angular.z = -error_x / 200
        rospy.loginfo("ANGULAR VALUE SENT ===>"+str(twist_object.angular.z))
        self.cmd_vel_pub.publish(twist_object)


def main():
    rospy.init_node('line_follower_object',anonymous=True)
    line_follower_object = LineFollower()
    ctrl_c = False
    rate = rospy.Rate(5)
    while not ctrl_c:
        rate.sleep();

if __name__ == '__main__':
    main()
