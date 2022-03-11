#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

# Function to read angle from Arduino and then write torque to Arduino
def get_angle():
    pub = rospy.Publisher('torque', String, queue_size=10)                    # publisher
    rospy.init_node('get_angle', anonymous=True)                                # initialise ROS node
    rate = rospy.Rate(10)                                                       # rate = 10 Hz
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)     # set up communication with Arduino
    rospy.loginfo("Angle detection node initialised.")
    rate.sleep()
    
    # Loop to read angles and write torque
    while not rospy.is_shutdown():
        rospy.loginfo("Detecting angle...")
        angle = arduino.readline().decode('utf-8')                              # read angle from Arduino
        rospy.loginfo(angle)                                                    # print angle to the screen

        # Calculate torque and write to Arduino
        torque = angle*1.01
        arduino.write(bytes(torque, 'utf-8'))
        rospy.loginfo("Wrote torque: ")
        rospy.loginfo(torque)

        pub.publish(torque)                                                     # publish data as ROS topic
        rate.sleep()

if __name__ == '__main__':
    try:
        get_angle()
    except rospy.ROSInterruptException:                                         # until keyboard interruption
        pass
