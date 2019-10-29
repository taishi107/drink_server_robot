#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,sys,cv2
import numpy as np
sys.path.append('/home/kattun/catkin_ws/src/pan_vision_control/scripts')
from utils import *
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Person_Follow():
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.twist = Twist()
        self.img = [720,1280]


    def yolo_callback(self,data):
        box_size = 0 #バウンディングボックスの大きさ
        gx = 0 #バウンディングボックスの中心座標
        gy = 0 #バウンディングボックスの中心座標

        for i in data.bounding_boxes:
            if i.Class == "person": #人判別
                if box_size < calc_box_size(i.xmin,i.ymin,i.xmax,i.ymax): #一番近い人を探す
                    box_size = calc_box_size(i.xmin,i.ymin,i.xmax,i.ymax)
                    gx,gy = calc_gravity_point(i.xmin,i.ymin,i.xmax,i.ymax)
        
        self.main(gx,gy,box_size)

    def image_callback(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data,'bgr8')


    def main(self,x,y,box_size):
        #左右
        if self.img.shape[1]/2 - x < -30:
            self.twist.angular.z = -0.3
        elif self.img.shape[1]/2 > self.img.shape[1]/2 - x > 30:
            self.twist.angular.z = 0.3
        else:
            self.twist.angular.z = 0

        #前進
        if 0 < box_size <= 150000:
            self.twist.linear.x = 0.1
        elif box_size > 250000:
            self.twist.linear.x = -0.1
        else:
            self.twist.linear.x = 0
        print(self.twist)
        print('~~~~~~~~~~~~~~')
        self.pub.publish(self.twist)


if __name__ == '__main__':
    rospy.init_node('parson_follow')
    Person_Follow()
    rospy.spin()
