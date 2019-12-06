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
from std_msgs.msg import Bool

class Person_Follow():
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_callback)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.dep_cb, queue_size=1) #realsense用
        self.cmd_pub = rospy.Publisher('/drink_server_robot/diff_drive_controller/cmd_vel', Twist, queue_size=1)
        self.pos_pub = rospy.Publisher('/pos', ParamManipulator, queue_size=1)
        self.hc_pub = rospy.Publisher('/hc_trigger', Bool, queue_size=1)
        self.pf_pub = rospy.Publisher('/pf_trigger', Bool, queue_size=1)
        self.bridge = CvBridge()
        self.twist = Twist()
        self.img = [720,1280]
        self.img_dep = []
        self.dep_val = 0
        self.mode = 0
        self.pm = ParamManipulator()
        self.pos_x, self.pos_y, self.pos_z = 10, 0 ,80 #ハンド位置の初期位置
        self.post_x, self.post_y, self.post_z = 0, 90, 0 #ハンド姿勢の初期姿勢
        self.hc_flag = False


    def yolo_callback(self,data):
        self.box_size = 0 #バウンディングボックスの大きさ
        self.gx = 0 #バウンディングボックスの中心座標
        self.gy = 0 #バウンディングボックスの中心座標
        self.person_box = 0
        self.dep_hold= 0 

        for i in data.bounding_boxes:
            if self.mode == 0 or self.mode == 1:
                if i.Class == "person": #人判別
                    self.mode = 1
                    if self.box_size < calc_box_size(i.xmin,i.ymin,i.xmax,i.ymax): #一番近い人を探す
                        self.box_size = calc_box_size(i.xmin,i.ymin,i.xmax,i.ymax)
                        self.gx,self.gy = calc_gravity_point(i.xmin,i.ymin,i.xmax,i.ymax)
                        # print("person")

            if self.mode != 3:
                if i.Class == "cup" or i.Class == "bowl": #コップ判別
                    self.mode = 2
                    if self.box_size < calc_box_size(i.xmin,i.ymin,i.xmax,i.ymax): #一番近いコップを探す
                        self.box_size = calc_box_size(i.xmin,i.ymin,i.xmax,i.ymax)
                        self.gx,self.gy = calc_gravity_point(i.xmin,i.ymin,i.xmax,i.ymax)
                        self.dep_listener(self.gx,self.gy)
                        self.dep_hold = self.dep_val
        
        # self.talker(gx,gy,box_size,dep_val)

    def image_callback(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data,'bgr8')
        # cv2.imshow('yolo', self.img)
        # cv2.waitKey(1)

    #デプス画像のコールバック
    def dep_cb(self,data):
        self.img_dep = []
        self.img_dep = self.bridge.imgmsg_to_cv2(data,'32FC1')
        

    #デプス画像から特定位置のデプスを抜き取る
    def dep_listener(self,x,y):
        self.dep_val = 0
        if len(self.img_dep) == 0:
            pass
        elif np.isnan(self.img_dep[y,x]) or np.isinf(self.img_dep[y,x]):
            pass
        else:
            self.dep_val = float(self.img_dep[y,x])
            self.dep_val = round(self.dep_val,5)

    def pos_talker(self):
        self.pm.pos_x = self.pos_x
        self.pm.pos_y = self.pos_y
        self.pm.pos_z = self.pos_z
        self.pm.post_x = self.post_x
        self.pm.post_y = self.post_y
        self.pm.post_z = self.post_z
        self.pos_pub.publish(self.pm)

    def talker(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        # self.pos_talker()
        
        # 人追従モード
        if self.mode == 1:
            #左右
            if self.img.shape[1]/2 - self.gx < -30:
                self.twist.angular.z = -2
                print("Right_person")
            elif self.img.shape[1]/2 > self.img.shape[1]/2 - self.gx > 30:
                self.twist.angular.z = 2
                print("Left_person")
            else:
                self.twist.angular.z = 0

            #前進
            if 0 < self.box_size <= 150000:
                self.twist.linear.x = 0.5
                print("Go_person")
            elif self.box_size > 250000:
                self.twist.linear.x = -0.5
                print("Back_person")
            else:
                self.twist.linear.x = 0
            
            if self.twist.angular.z == 0 and self.twist.linear.x == 0:
                print("コップを出してね")
		jtalk("コップが見つかりません")		

        #コップ追従モード
        if self.mode == 2:
            #左右
            if self.img.shape[1]/2 - self.gx < -150:
                self.twist.angular.z = -2
                print("Right")
            elif self.img.shape[1]/2 > self.img.shape[1]/2 - self.gx > -50:
                self.twist.angular.z = 2
                print("Left")
            else:
                self.twist.angular.z = 0

            # 前進
            if 0 < self.box_size <= 90000:
                self.twist.linear.x = 0.5
                print("Go")
            elif self.box_size > 120000:
                self.twist.linear.x = -0.5
                print("Back")
            else:
                self.twist.linear.x = 0
            # if self.dep_hold > 180 and self.dep_hold != 0:
            #     self.twist.linear.x = 1
            #     print("Go")
            # elif self.dep_hold < 50 and self.dep_hold != 0:
            #     self.twist.linear.x = -1
            #     print("Back")
            # else:
            #     self.twist.linear.x = 0

            if (self.gx != 0 and self.box_size != 0 and self.dep_hold != 0) and self.twist.angular.z == 0 and self.twist.linear.x == 0:
                print("ok")
                self.mode = 3

                # if (self.img.shape[0] - ymin) > 30:
                #     self.twist.linear.x = 1
                #     print("go")
                # elif (self.img.shape[0] - ymin) < 10:
                #     self.twist.linear.x = -1
                #     print("back")
                # else:
                #     self.twist.linear.x = 0
                #     print("ok")

        elif self.mode == 3:
            m = Main()
            self.hc_flag = True
            self.hc_pub.publish(self.hc_flag)
            # print("mode : ",self.mode)
            m.pf_flag = False
            self.pf_pub.publish(m.pf_flag)
            self.mode = 0

        print('~~~~~~~~~~~~~~')
        self.cmd_pub.publish(self.twist)


class Main():
    def __init__(self):
        rospy.Subscriber('/pf_trigger', Bool, self.pf_flag_callback)
        rospy.Subscriber('/hc_trigger', Bool, self.hc_flag_callback)
        self.pf_flag = True
        self.pf = Person_Follow()

    def pf_flag_callback(self,data):
        self.pf_flag = data.data

    def hc_flag_callback(self,data):
        self.pf.hc_flag = data.data

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.pf_flag == True:
                try:
                    self.pf.talker()
                except Exception as e:
                    print(e)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('parson_follow')
    # Person_Follow()
    # rospy.spin()
    m = Main()
    m.run()
