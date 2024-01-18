#! /usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import  Int16
from guppy.msg import msg_GC_command

import Adafruit_PCA9685
import os



sub_bot_Connection = 0

#.........i2c_Detect.................
#data =[name for name in os.listdir("/sys/bus/pci/devices/0000:00:19.0/i2c_designware.4") ]
#matching = [s for s in data if "i2c" in s]
#i2c_port = int(matching[0][-1])

pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=1)
pwm.set_pwm_freq(200)
gripper_command=0
#..........................
esc_start = 1260 ; servo_offset = 12

def GC_command_CB(msg):
    global gripper_command   

    gripper_command = msg.rc_sw.grp  

def robot_ConnectionCB(msg):
    global sub_bot_Connection
    sub_bot_Connection = msg.data  

if __name__ == '__main__':
    pwm.set_pwm(0, 0, esc_start)

    subSetpoint = rospy.Subscriber("GC_command", msg_GC_command, GC_command_CB)
    subConnection = rospy.Subscriber("Connection", Int16, robot_ConnectionCB)

    rospy.init_node('gripper', anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if (sub_bot_Connection == 1):

            if gripper_command == 6:
                pwm.set_pwm(6, 0,esc_start+140 ) 
            elif  gripper_command == 5:
                pwm.set_pwm(6, 0,esc_start-140 )
            else:
                pwm.set_pwm(6, 0,esc_start )
  
        else:
            pwm.set_pwm(6, 0,esc_start )
        rate.sleep()

        

