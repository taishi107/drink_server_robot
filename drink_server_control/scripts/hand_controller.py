#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, math
import utils
import numpy as np
from drink_server_control.msg import ParamManipulator
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Int16, String
from time import sleep

class Talker():
    def __init__(self):
        self.pub = rospy.Publisher('/pos', ParamManipulator, queue_size=1)
        self.pf_pub = rospy.Publisher('/pf_trigger', Bool, queue_size=1)
        self.hc_pub = rospy.Publisher('/hc_trigger', Bool, queue_size=1)
        self.er_pub = rospy.Publisher('/er_trigger', Bool, queue_size=1)
        self.drink_pub = [
            rospy.Publisher('drink_server_robot/drink_driver/left_stop', Bool, queue_size=1),
            rospy.Publisher('drink_server_robot/drink_driver/left_release', Bool, queue_size=1),
            rospy.Publisher('drink_server_robot/drink_driver/right_stop', Bool, queue_size=1),
            rospy.Publisher('drink_server_robot/drink_driver/right_release', Bool, queue_size=1)
        ]

        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bb_cb)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.img_cb)
        # rospy.Subscriber('/zed_node/depth/depth_registered', Image, self.dep_cb, queue_size=1) #zed用
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.dep_cb, queue_size=1) #realsense用
        rospy.Subscriber('/emotion', String, self.er_cb)
        self.pm = ParamManipulator()
        self.bridge = CvBridge()
        self.pos_x, self.pos_y, self.pos_z = 10, 0 ,80 #ハンド位置の初期位置
        self.post_x, self.post_y, self.post_z = 0, 90, 0 #ハンド姿勢の初期姿勢
        self.img = [0,0] #画像
        self.img_dep = [] #デプス画像
        self.cup_pos = [] #コップの位置
        self.mode = 0 #動作のモード用
        self.pos_flag = [False,False] #ハンドの位置の判別用
        self.pos_hold = [0, 0, 0] #ハンドの位置の保持用
        self.ymax = 0
        self.pf_flag = False
        self.er_flag = False
        self.emotion = ""
        self.drink_section = 0
        self.drink1_name = "お茶"
        self.drink2_name = "黒霧島"
        self.emotion2 = []
        self.jtalk = True
        utils.jtalk("コップをカメラに水平に出してください")
        

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
        # cv2.imshow('yolo', self.img)
        # cv2.waitKey(1)

    #デプス画像のコールバック
    def dep_cb(self,data):
        self.img_dep = []
        self.img_dep = self.bridge.imgmsg_to_cv2(data,'32FC1')

    def er_cb(self,data):
        #self.emotion = data.data
        self.emotion2.append(data.data)
        #print(self.emotion,"cb")
        

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

    # ParamManipulatorのパブリッシャ
    def pm_publisher(self):
        self.pm.pos_x = self.pos_x
        self.pm.pos_y = self.pos_y
        self.pm.pos_z = self.pos_z
        self.pm.post_x = self.post_x
        self.pm.post_y = self.post_y
        self.pm.post_z = self.post_z
        self.pub.publish(self.pm)

    def main(self):
        y_count = 0
        z_count = 0
        # if len(self.cup_pos) != 0 or self.mode == 1 or self.mode == 2:
        if len(self.cup_pos) != 0 or self.mode != 0:
            #位置合わせ
            # print(self.cup_pos)
            if self.mode == 0:
                print("mode : ",self.mode)
                if (self.cup_pos[0] - self.img.shape[1]/2) < 50:
                    self.pos_y -= 1
                    y_count -= 1
                    self.pos_flag[0] = False
                    print('左')
                elif (self.cup_pos[0] - self.img.shape[1]/2) > 100:
                    self.pos_y += 1
                    y_count += 1
                    self.pos_flag[0] = False
                    print('右')
                else:
                    self.pos_flag[0] = True

                # if (self.ymax - self.img.shape[0]) >  -30:
                if (self.cup_pos[1] - self.img.shape[0]/2) < -20:
                    self.pos_z += 1
                    z_count += 1
                    self.pos_flag[1] = False
                    print('上')
                elif (self.cup_pos[1] - self.img.shape[0]/2) > 70:
                    self.pos_z -= 1
                    z_count -= 1
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
                    self.pos_hold = [y_count, z_count]

            # elif self.mode == 1:
                # print(self.cup_pos)
                # print("ｙｚの位置決まり")
                # if self.cup_pos[2] > 180:
                #     self.pos_x += 3
                #     print('前')
                # elif self.cup_pos[2] < 50 and self.cup_pos[2] != 0:
                #     self.pos_x -= 3
                #     print('後')
                # else:
                #     self.mode = 2

            elif self.mode == 1:
                
                self.pos_x, self.pos_y, self.pos_z = 0.85, 0.0, 190.0
                self.post_x, self.post_y, self.post_z = 0, 60, 0
                self.pm_publisher()
                rospy.sleep(2)
                if self.jtalk:
                    utils.jtalk("飲み物は、" + self.drink1_name + "と、" + self.drink2_name + "、があります")

                    self.jtalk = False
                #print("ok")
                sleep(6)
                
                self.er_flag = True
                self.er_pub.publish(self.er_flag)
                
                
                #if self.emotion == "HAPPINESS":
                    #self.emotion2 = True
                #print(self.emotion2)
                # 飲み物１の選択
                ## 音声を流す
                if self.drink_section == 0:
                    utils.jtalk(self.drink1_name + "が欲しい場合は、笑顔を見せてください")
                    sleep(5)
                    # 表情認識
                    #self.er_flag = True
                    #self.er_pub.publish(self.er_flag)
                    #rospy.sleep(2)
                    
                    if self.emotion2.count("HAPPINESS") > 0:
                        rospy.sleep(3)
                        utils.jtalk(self.drink1_name + "をお注ぎしますね")
                        rospy.sleep(1)
                        self.pos_x, self.pos_y, self.pos_z = 120, 0+self.pos_hold[0] ,120
                        self.post_x, self.post_y, self.post_z = 0, 90, 0 
                        self.pm_publisher()
                        rospy.sleep(2)

                        print("Open drink1")
                        self.drink_pub[0].publish(0)
                        rospy.sleep(5)
                        self.drink_pub[1].publish(1)
                        rospy.sleep(7)
                        
                        print("Close drink1")
                        self.drink_pub[1].publish(0)
                        rospy.sleep(1)
                        self.drink_pub[0].publish(1)
                        rospy.sleep(5)
                        

                        self.er_flag = False
                        self.er_pub.publish(self.er_flag)

                        self.mode = 2
                    else :
                        self.emotion2 = []
                        self.drink_section = 1

                elif self.drink_section == 1:
                    
                    #sleep(4)
                    utils.jtalk(self.drink2_name + "が欲しい場合は、笑顔を見せてください")
                    sleep(5)
                    # 表情認識
                    #self.er_flag = True
                    #self.er_pub.publish(self.er_flag)
                    #rospy.sleep(2)
                    
                    if self.emotion2.count("HAPPINESS") > 0:
                        rospy.sleep(3)
                        utils.jtalk(self.drink2_name + "をお注ぎしますね")
                        rospy.sleep(1)
                        self.pos_x, self.pos_y, self.pos_z = 120, 0+self.pos_hold[0] ,120
                        self.post_x, self.post_y, self.post_z = 0, 90, 0 
                        self.pm_publisher()
                        rospy.sleep(2)

                        print("Open drink2")
                        self.drink_pub[2].publish(0)
                        rospy.sleep(5)
                        self.drink_pub[3].publish(1)
                        rospy.sleep(7)
                        
                        print("Close drink2")
                        self.drink_pub[3].publish(0)
                        rospy.sleep(1)
                        self.drink_pub[2].publish(1)
                        rospy.sleep(5)
                        

                        self.er_flag = False
                        self.er_pub.publish(self.er_flag)
                        self.drink_section = 0
                        self.mode = 2
                    else :
                        sleep(3)
                        utils.jtalk("もう一度、お聞きします")
                        rospy.sleep(2)
                        self.emotion2 = []
                        self.drink_section = 0
                        self.jtalk = True

                self.emotion2 = []

            #排水動作
            elif self.mode == 2:
                utils.jtalk("それでは、花見を楽しんでください")
                sleep(2)
                print("mode : ",self.mode)
                #少し上に上がる
                self.pos_x, self.pos_y, self.pos_z = 100, 0 ,160
                self.post_x, self.post_y, self.post_z = 0, 90, 0
                self.pm_publisher()
                print("Up")
                rospy.sleep(3)
                #左に向く
                self.pos_x, self.pos_y, self.pos_z = 0, -60 ,130
                self.post_x, self.post_y, self.post_z = 0, 90, 0
                self.pm_publisher()
                print("Turn Left")
                rospy.sleep(2)
                #少し下がる
                # self.pos_x, self.pos_y, self.pos_z = -20, -70 ,90
                # self.post_x, self.post_y, self.post_z = 0, 90, 0
                # self.pm_publisher()
                # print("Down")
                # rospy.sleep(1)
                #排水位置
                self.pos_x, self.pos_y, self.pos_z = -20, -130 ,-70
                self.post_x, self.post_y, self.post_z = 0, 160, 0
                self.pm_publisher()
                rospy.sleep(10)

                # self.pm.pos_x = self.pos_x
                # self.pm.pos_y = self.pos_y
                # self.pm.pos_z = self.pos_z
                # self.pm.post_x = self.post_x
                # self.pm.post_y = self.post_y
                # self.pm.post_z = self.post_z
                # self.pub.publish(self.pm)

                #初期位置
                self.pos_x, self.pos_y, self.pos_z = 10, 0 ,80
                self.post_x, self.post_y, self.post_z = 0, 90, 0
                self.pm_publisher()
                rospy.sleep(5)

                self.mode = 3
            
            if self.mode == 3:
                utils.jtalk("次の人を探索します、それでは失礼します")
                sleep(5)
                self.pf_flag = True
                m = Main()
                m.hc_flag = False
                self.pf_pub.publish(self.pf_flag)
                self.hc_pub.publish(m.hc_flag)
                self.mode = 0
                
        # self.pm.pos_x = self.pos_x
        # self.pm.pos_y = self.pos_y
        # self.pm.pos_z = self.pos_z
        # self.pm.post_x = self.post_x
        # self.pm.post_y = self.post_y
        # self.pm.post_z = self.post_z
        # self.pub.publish(self.pm)
        self.pm_publisher()
        print("~~~~~~~~~~~~~~~~~")
        #print(self.emotion)
        # print(self.pm)

class Main():
    def __init__(self):
        rospy.Subscriber('/hc_trigger', Bool, self.hc_flag_callback)
        rospy.Subscriber('/pf_trigger', Bool, self.pf_flag_callback)
        self.hc_flag = False
        self.t = Talker()
    
    def hc_flag_callback(self,data):
        self.hc_flag = data.data

    def pf_flag_callback(self,data):
        self.t.pf_flag = data.data

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.hc_flag == True:
                try:
                    self.t.main()
                except Exception as e:
                    print(e)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hand_control')
    # rate = rospy.Rate(5)
    # t = Talker()
    # while not rospy.is_shutdown():
    #     try:
    #         t.main()
    #     except Exception as e:
    #         print(e)
    #     rate.sleep()
    m = Main()
    m.run()
