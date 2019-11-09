#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2
import numpy as np
from drink_server_robot.msg import ParamManipulator
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Talker():
    def __init__(self):
        self.pub = rospy.Publisher('/pos', ParamManipulator, queue_size=1)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bb_cb)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.img_cb)
        rospy.Subscriber('/zed_node/depth/depth_registered', Image, self.dep_cb, queue_size=1)
        self.pm = ParamManipulator()
        self.bridge = CvBridge()
        self.pos_x, self.pos_y, self.pos_z = 0, 0, 0 #ハンド位置の初期値
        self.post_x, self.post_y, self.post_z = 0, 0, 0 #ハンド姿勢の初期値
        self.img = [0,0] #画像
        self.img_dep = [] #デプス画像

    #バウンディングボックスのコールバック
    def bb_cb(self,data):
        self.cup_pos = [] #コップの位置
        for i in data.bounding_boxes:
            if i.Class == "cup": #コップ判別
                self.dep_listener((i.xmax+i.xmin)/2, (i.ymax+i.ymin)/2)
                self.cup_pos = [(i.xmax+i.xmin)/2, (i.ymax+i.ymin)/2, self.dep_val]

    #画像のコールバック
    def img_cb(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data,'bgr8')
        cv2.imshow('yolo', self.img)
        cv2.waitKey(1)

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

    def main(self):
        if len(self.cup_pos) != 0:
            #位置合わせ
            if (self.cup_pos[0] - self.img.shape[1]/2) < -40:
                self.pos_x += 1
                print('左')
            elif (self.cup_pos[0] - self.img.shape[1]/2) > 40:
                self.pos_x -= 1
                print('右')

            if (self.cup_pos[1] - self.img.shape[0]/2) < -20:
                self.pos_y += 1
                print('上')
            elif (self.cup_pos[1] - self.img.shape[0]/2) > 20:
                self.pos_y -= 1
                print('下')

            if self.cup_pos[2] > 0.15:
                self.pos_z += 1
                print('前')
            elif self.cup_pos[2] < 0.1 and self.cup_pos[2] == 0:
                self.pos_z -= 1
                print('後')

        self.pm.pos_x = self.pos_x
        self.pm.pos_y = self.pos_y
        self.pm.pos_z = self.pos_z
        self.pub.publish(self.pm)
        print("~~~~~~~~~~~~~~~~~")
        print(self.pm)

if __name__ == '__main__':
    rospy.init_node('hand_control')
    rate = rospy.Rate(5)
    t = Talker()
    while not rospy.is_shutdown():
        try:
            t.main()
        except Exception as e:
            print(e)
        rate.sleep()
