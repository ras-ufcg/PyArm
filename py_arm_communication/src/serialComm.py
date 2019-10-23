#!/usr/bin/env python
import serial
import rospy
import time
import struct
from py_arm_communication.msg import joints

ser = serial.Serial('/dev/ttyUSB0') # open serial port

def callback(data):
    rospy.loginfo("Callback")

    joint_list = list(data.joint) # Fazer funcionar
    message = ""
    
    for joint_state in joint_list:
        if joint_state > 99:
            message += str(joint_state)
        else:
            if joint_state > 9:
                message += "0" + str(joint_state)
            else:
                message += "00" + str(joint_state)
    ser.write(message)
    
    s = ser.read(5)
    #s = ser.read_until()
    rospy.loginfo("Message received")
    rospy.loginfo(s)
    input_list = list(s)
    rospy.loginfo("Joints variables: ")
    rospy.loginfo(input_list)
    
def listener():
    rospy.init_node('communication_node', anonymous=True)

    rospy.Subscriber("joints", joints, callback)

    rospy.loginfo("Iniciando o no")

    rospy.spin()
    ser.close() 

if __name__ == '__main__':
    listener()