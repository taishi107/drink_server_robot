#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,sys,cv2
import numpy as np
sys.path.append('/home/kattun/catkin_ws/src/drink_server_robot/scripts')
from utils import *
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drink_server_control.msg import ParamManipulator

class Person_Follow():
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_callback)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)
        self.cmd_pub = rospy.Publisher('/drink_server_robot/diff_drive_controller/cmd_vel', Twist, queue_size=1)
        self.pos_pub = rospy.Publisher('/pos', ParamManipulator, queue_size=1)
        self.bridge = CvBridge()
        self.twist = Twist()
        self.img = [720,1280]
        self.mode = 0
        self.pm = ParamManipulator()
        self.pos_x, self.pos_y, self.pos_z = 90, 0 ,30 #ハンド位置の初期位置
        self.post_x, self.post_y, self.post_z = 0, 150, 0 #ハンド姿勢の初期姿勢


    def yolo_callback(self,data):
        box_size = 0 #バウンディングボックスの大きさ
        gx = 0 #バウンディングボックスの中心座標
        gy = 0 #バウンディングボックスの中心座標
        big_ymin = 0

        for i in data.bounding_boxes:
            if i.Class == "cup" or i.Class == "bowl": #人判別
                if box_size < calc_box_size(i.xmin,i.ymin,i.xmax,i.ymax): #一番近い人を探す
                    box_size = calc_box_size(i.xmin,i.ymin,i.xmax,i.ymax)
                    gx,gy = calc_gravity_point(i.xmin,i.ymin,i.xmax,i.ymax)
                    big_ymin = i.ymin
        
        self.talker(gx,gy,i.ymin)

    def image_callback(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data,'bgr8')
        cv2.imshow('yolo', self.img)
        cv2.waitKey(1)

    def pos_talker(self):
        self.pm.pos_x = self.pos_x
        self.pm.pos_y = self.pos_y
        self.pm.pos_z = self.pos_z
        self.pm.post_x = self.post_x
        self.pm.post_y = self.post_y
        self.pm.post_z = self.post_z
        self.pos_pub.publish(self.pm)

    def talker(self,x,y,ymin):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.pos_talker()
        
        if self.mode == 0:
            #左右
            if self.img.shape[1]/2 - x < -30:
                self.twist.angular.z = -5
            elif self.img.shape[1]/2 > self.img.shape[1]/2 - x > 30:
                self.twist.angular.z = 5
            else:
                self.twist.angular.z = 0

                #前進
                # if 0 < box_size <= 150000:
                #     self.twist.linear.x = 1
                # elif box_size > 250000:
                #     self.twist.linear.x = -1
                # else:
                #     self.twist.linear.x = 0
                #     print("ok")
                if (self.img.shape[0] - ymin) > 30:
                    self.twist.linear.x = 1
                elif (self.img.shape[0] - ymin) <= 10:
                    self.twist.linear.x = -1
                else:
                    self.twist.linear.x = 0
                    print("ok")

        # elif self.mode == 1
        print(self.twist)
        print('~~~~~~~~~~~~~~')
        self.cmd_pub.publish(self.twist)


if __name__ == '__main__':
    rospy.init_node('parson_follow')
    Person_Follow()

    rospy.spin()
