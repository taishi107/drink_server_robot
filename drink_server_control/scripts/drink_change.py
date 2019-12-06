#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool

if __name__ == '__main__':
    rospy.init_node('drink_change')
    flag = True
    right_pub = [
            rospy.Publisher('drink_server_robot/drink_driver/right_stop', Bool, queue_size=1),
            rospy.Publisher('drink_server_robot/drink_driver/right_release', Bool, queue_size=1),
            rospy.Publisher('drink_server_robot/drink_driver/right_replacement', Bool, queue_size=1)
    ]
    left_pub = [
            rospy.Publisher('drink_server_robot/drink_driver/left_stop', Bool, queue_size=1),
            rospy.Publisher('drink_server_robot/drink_driver/left_release', Bool, queue_size=1),
            rospy.Publisher('drink_server_robot/drink_driver/left_replacement', Bool, queue_size=1)
    ]
    try:
        while not rospy.is_shutdown():
            direction = raw_input('a: All Open, r: Right Close, l: Left Close, q: Quit > ')
            if 'a' in direction:
                right_pub[2].publish(True),left_pub[2].publish(True)

            if 'r' in direction:
                right_pub[0].publish(True),right_pub[1].publish(False)
                print("Right OK")

            if 'l' in direction:
                left_pub[0].publish(True),left_pub[1].publish(False)
                print("Left OK")

            if 'q' in direction:
                right_pub[0].publish(True),left_pub[0].publish(True)
                right_pub[1].publish(False), left_pub[1].publish(False)
                print("Quit")
                break

    except rospy.ROSInterruptException:
        pass