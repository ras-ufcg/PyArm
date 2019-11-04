#!/usr/bin/env python
import serial
import rospy
import time
import struct
import sys
from py_arm_communication.msg import joints

ser = serial.Serial('/dev/ttyUSB0')

def callback(data):
    rospy.loginfo("Callback")

    joint_list = list(data.joint)
    
    for joint_state in joint_list:
        ser.write(str(chr(joint_state)))
    
    s = ser.read(5)
    rospy.loginfo("Message received")
    rospy.loginfo(s)
    input_list = list(s)
    rospy.loginfo("Joints variables")
    rospy.loginfo(input_list)
    
def listener():
    rospy.init_node('communication_node', anonymous=True)

    rospy.Subscriber("joints", joints, callback)

    rospy.loginfo("Iniciando o no")

    rospy.spin()
    ser.close() 

if __name__ == '__main__':
    listener()