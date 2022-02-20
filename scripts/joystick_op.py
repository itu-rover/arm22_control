#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: 	B. Burak Payzun
# Date: 	2021-02-15
#
###########################

import rospy
import serial
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Joy


class fk_teleop:
    def __init__(self):
        self.axes = [0, 0, 0, 0, 0, 0, 0]
        self.buttons = [0, 0, 0, 0, 0, 0, 0]
        self.is_resetting = False
        # self.port = rospy.get_param("/serial/port")
        # self.baudrate = rospy.get_param("/serial/baudrate")
        # self.port = "/dev/heroRK"
        # self.baudrate = 115200
        self.ser = serial.Serial("/dev/heroSteering", 115200)

        rospy.init_node("fk_teleop")

        self.joy = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.rate = rospy.Rate(20)

        comm_check_byte = 0
        while not rospy.is_shutdown():
            comm_check_byte ^= 1
            message_to_send = ""
            message_to_send += "S"
            message_to_send += str(int(max(0, min(5 +
                                   (self.axes[1] * self.buttons[0])*5, 9))))
            message_to_send += str(int(max(0, min(5 +
                                   (self.axes[1] * self.buttons[1])*5, 9))))
            message_to_send += str(int(max(0, min(5 +
                                   (self.axes[1] * self.buttons[2])*5, 9))))
            message_to_send += str(int(max(0, min(5 +
                                   (self.axes[1] * self.buttons[3])*5, 9))))
            message_to_send += str(int(max(0, min(5 +
                                   (self.axes[1] * self.buttons[4])*5, 9))))
            message_to_send += str(int(max(0, min(5 +
                                   (-self.axes[1] * self.buttons[5])*5, 9))))
            message_to_send += str(comm_check_byte)
            message_to_send += "F"
            if self.is_resetting:
                message_to_send = "RRRRR"
            print(message_to_send)
            self.ser.write(message_to_send)
            self.rate.sleep()

    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons
        self.is_resetting = (data.axes[2] + data.axes[5] < -1.5)


if __name__ == "__main__":
    fk_teleop()
