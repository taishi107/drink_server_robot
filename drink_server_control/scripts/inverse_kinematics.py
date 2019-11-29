#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from drink_server_control.msg import ParamManipulator
from std_msgs.msg import Float64

class ControlManipulator():
    def __init__(self):
        rospy.Subscriber('/pos', ParamManipulator, self.cb)
        """
        theta1 : /shoulder_revolute_servo_controller/command = pub1
        theta2 : /shoulder_flex_servo_controller/command = pub2
        theta3 : /elbow_servo_controller/command = pub3
        theta4 : /wrist_servo_controller/command = pub4
        """
        self.pub1 = rospy.Publisher('/shoulder_revolute_servo_controller/command',  Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/shoulder_flex_servo_controller/command',  Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/elbow_servo_controller/command',  Float64, queue_size=10)
        self.pub4 = rospy.Publisher('/wrist_servo_controller/command',  Float64, queue_size=10)

        self.pos_x, self.pos_y, self.pos_z = 10, 0, 80 #ハンド位置の初期値
        self.post_x, self.post_y, self.post_z = 0, 90, 0 #ハンド姿勢の初期値
        # 各リンクの長さ
        self.l1 = 83
        self.l2 = 93.5
        self.l3 = 85

    def cb(self, data):
        self.pos_x = data.pos_x
        self.pos_y = data.pos_y
        self.pos_z = data.pos_z
        self.post_x = data.post_x
        self.post_y = data.post_y
        self.post_z = data.post_z

    def inverse_kinematics(self):
        x = self.pos_x
        y = self.pos_y
        z = self.pos_z
        th4 = math.radians(self.post_y)

        # theta1
        th1 = math.atan2(y,x)
        
        #目標座標の計算
        l = math.sqrt(x**2 + y**2)
        x2 = l - self.l3*math.sin(th4)
        y2 = z - self.l3*math.cos(th4)
        #print(l,z)
        #print(x2,y2)
        # theta2
        tmp1 = x2**2 + y2**2 + self.l1**2 - self.l2**2
        #print(tmp1)
        tmp2 = 2*self.l1*math.sqrt(x2**2 + y2**2)
        #print(tmp2)
        th2 = math.acos(tmp1/tmp2) + math.atan2(y2, x2)
        #print(th2)
        #th2 = th2 - math.pi / 2
        #theta3
        tmp3 = y2 - self.l1*math.sin(th2)
        tmp4 = x2 - self.l1* math.cos(th2)
        th3 = math.atan2(tmp3, tmp4) - th2
        
        #theta4
        #th2 = th2 - math.pi/2
        th4 = -1*th4 - (th3 + th2)
        print(th1,th2,th3,th4)
        
        return [th1,th2,th3,th4]

    def main(self):
        rad1,rad2,rad3,rad4 = self.inverse_kinematics()
        print(math.degrees(rad1),math.degrees(rad2 - math.pi/2),math.degrees(rad3),math.degrees(rad4 + math.pi/2))

        #print(rad1,rad2,rad3,rad4)
        #符号忘れたから実機で確認
        self.pub1.publish(-1*rad1)
        self.pub2.publish(-1*(rad2 - math.pi/2))
        self.pub3.publish(rad3)
        self.pub4.publish((rad4 + math.pi/2))

        

if __name__ == '__main__':
    rospy.init_node('inverse_kinetics')
    rate = rospy.Rate(5)
    cm = ControlManipulator()
    while not rospy.is_shutdown():
        try:
            cm.main()
        except Exception as e:
            print(e)
        rate.sleep()

