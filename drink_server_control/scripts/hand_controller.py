#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, math
import numpy as np
from drink_server_control.msg import ParamManipulator
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Talker():
    def __init__(self):
        self.pub = rospy.Publisher('/pos', ParamManipulator, queue_size=1)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bb_cb)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.img_cb)
        # rospy.Subscriber('/zed_node/depth/depth_registered', Image, self.dep_cb, queue_size=1) #zed用
        rospy.Subscriber('/camera/infra1/image_rect_raw', Image, self.dep_cb, queue_size=1) #realsense用
        self.pm = ParamManipulator()
        self.bridge = CvBridge()
        self.pos_x, self.pos_y, self.pos_z = 90, 0 ,30 #ハンド位置の初期位置
        self.post_x, self.post_y, self.post_z = 0, 150, 0 #ハンド姿勢の初期姿勢
        self.img = [0,0] #画像
        self.img_dep = [] #デプス画像
        self.cup_pos = [] #コップの位置
        self.mode = 0 #動作のモード用
        self.pos_flag = [False,False] #ハンドの位置の判別用
        self.pos_hold = [0, 0, 0] #ハンドの位置の保持用
        self.ymax = 0

    #バウンディングボックスのコールバック
    def bb_cb(self,data):
        self.cup_pos = [] #コップの位置
        box_size = 0
        self.h, self.w = 0, 0
        for i in data.bounding_boxes:
            if i.Class == "cup" or i.Class == "bowl": #コップ判別
                if box_size < (i.xmax-i.xmin)*(i.ymax-i.ymin):
                    self.dep_listener((i.xmax+i.xmin)/2, (i.ymax+i.ymin)/2)
                    self.cup_pos = [(i.xmax+i.xmin)/2, (i.ymax+i.ymin)/2, self.dep_val]
                    self.h = i.ymax - i.ymin
                    self.w = i.xmax - i.xmin
                    self.ymax = i.ymin
                    box_size = self.w * self.h


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
            if self.mode == 0:
                if (self.cup_pos[0] - self.img.shape[1]/2) < 50:
                    self.pos_y += 1
                    self.pos_flag[0] = False
                    print('左')
                elif (self.cup_pos[0] - self.img.shape[1]/2) > 100:
                    self.pos_y -= 1
                    self.pos_flag[0] = False
                    print('右')
                else:
                    self.pos_flag[0] = True

                # if (self.ymax - self.img.shape[0]) >  -30:
                if (self.cup_pos[1] - self.img.shape[0]/2) < -20:
                    self.pos_x += 1
                    self.pos_flag[1] = False
                    print('上')
                elif (self.cup_pos[1] - self.img.shape[0]/2) > 70:
                    self.pos_x -= 1
                    self.pos_flag[1] = False
                    print('下')
                else:
                    self.pos_flag[1] = True

                # if self.cup_pos[2] > 0.12:
                #     self.pos_x += 1
                #     self.pos_flag[2] = False
                #     print('前')
                # elif self.cup_pos[2] < 0.1 and self.cup_pos[2] != 0:
                #     self.pos_x -= 1
                #     self.pos_flag[2] = False
                #     print('後')
                # else:
                #     self.pos_flag[2] = True

                #self.pos_flagがすべてTrueのとき
                if sum(self.pos_flag) == 2:
                    self.mode = 1
                    #現在の位置を保持
                    self.pos_hold = [self.pos_y, self.pos_z]

            elif self.mode == 1:
                # print(self.cup_pos)
                # print("ｙｚの位置決まり")
                if self.cup_pos[2] > 70:
                    self.pos_z -= 3
                    print('前')
                    # self.pos_flag = [False , False]
                elif self.cup_pos[2] < 50 and self.cup_pos[2] != 0:
                    self.pos_z += 3
                    print('後')
                    # self.pos_flag = [False , False]
                else:
                    print(self.cup_pos)
                self.mode = 0
                # else:
                #     self.mode = 5
 
            #注ぎ動作
            elif self.mode == 2:
                self.pos_y = self.pos_hold[1] + self.h/2 + 5
                self.pos_z = self.pos_hold[2] + 10

                #パブリッシュ
                self.pm.pos_y = self.pos_y
                self.pm.pos_z = self.pos_z
                self.pub.publish(self.pm)
                rospy.sleep(5)

                self.mode = 3
                print("mode : ", self.mode)

            #初期位置に戻る
            elif self.mode == 3:
                self.pos_x, self.pos_y, self.pos_z = 60, 0 ,160
                self.post_x, self.post_y, self.post_z = 0, 80, 0

                self.pm.pos_x = self.pos_x
                self.pm.pos_y = self.pos_y
                self.pm.pos_z = self.pos_z
                self.pm.post_x = self.post_x
                self.pm.post_y = self.post_y
                self.pm.post_z = self.post_z
                self.pub.publish(self.pm)
                rospy.sleep(5)

                self.mode = 0


        self.pm.pos_x = self.pos_x
        self.pm.pos_y = self.pos_y
        self.pm.pos_z = self.pos_z
        self.pm.post_x = self.post_x
        self.pm.post_y = self.post_y
        self.pm.post_z = self.post_z
        self.pub.publish(self.pm)
        print("~~~~~~~~~~~~~~~~~")
        print(self.pm)

if __name__ == '__main__':
    rospy.init_node('hand_control')
    rate = rospy.Rate(30)
    t = Talker()
    while not rospy.is_shutdown():
        try:
            t.main()
        except Exception as e:
            print(e)
        rate.sleep()
