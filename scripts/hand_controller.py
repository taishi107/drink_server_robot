#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from drink_server_robot.msg import ParamManipulator

class Talker():
    def __init__(self):
        self.pub = rospy.Publisher('/pos', ParamManipulator, queue_size=1)
        self.pm = ParamManipulator()
        self.pos_x, self.pos_y, self.pos_z = 0, 0, 0
        self.post_x, self.post_y, self.post_z = 0, 0, 0

    def main(self):
        self.pos_x = 1

        self.pm.pos_x = self.pos_x
        self.pub.publish(self.pm)
        print("~~~~~~~~~~~~~~~~~")
        print(self.pm)

if __name__ == '__main__':
    rospy.init_node('hand_control')
    rate = rospy.Rate(10)
    t = Talker()
    while not rospy.is_shutdown():
        try:
            t.main()
        except Exception as e:
            print(e)
        rate.sleep()
