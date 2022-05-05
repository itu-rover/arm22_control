#!/usr/bin/env python

from arm22_control.srv import serial, serialResponse
import rospy

def handle(req):
    print("Got serial message %s" % req)
    rospy.sleep(0.015)
    return serialResponse(req.serial_msg)

def serial_echo():
    rospy.init_node('serial_echo_server')
    s = rospy.Service('serial_echo', serial, handle)
    rospy.spin()

if __name__ == "__main__":
    serial_echo()
