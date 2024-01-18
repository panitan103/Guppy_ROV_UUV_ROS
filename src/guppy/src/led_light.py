#! /usr/bin/env python
from __future__ import division
import time
from unittest import case
import rospy
from std_msgs.msg import Int16, String
import Adafruit_PCA9685
from guppy.msg import msg_GC_command
import os


sub_led = 0
PVsub_led = 0
sub_bot_Connection = 0

#.........i2c_Detect.................
#data =[name for name in os.listdir("/sys/bus/pci/devices/0000:00:19.0/i2c_designware.4") ]
#matching = [s for s in data if "i2c" in s]
#i2c_port = int(matching[0][-1])

pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=1)
pwm.set_pwm_freq(200)

#..........................
esc_start = 842

def robot_ledCB(msg):
    global sub_led
    sub_led = msg.rc_sw.sw_C
    #print(sub_led)  
    
def robot_ConnectionCB(msg):
    global sub_bot_Connection
    sub_bot_Connection = msg.data  
    
if __name__ == '__main__':
    pwm.set_pwm(15, 0, esc_start)
    subCG_command = rospy.Subscriber("GC_command", msg_GC_command, robot_ledCB)
    subConnection = rospy.Subscriber("Connection", Int16, robot_ConnectionCB)

    rospy.init_node('led_light', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if (sub_bot_Connection == 1):
            if sub_led != PVsub_led :
                if sub_led == 0:
                    pwm.set_pwm(7, 0, 842)
                elif sub_led == 1:
                    pwm.set_pwm(7, 0, 1260)
                elif sub_led == 2:
                    pwm.set_pwm(7, 0, 1678)
            PVsub_led = sub_led
        else:
            pwm.set_pwm(15, 0, 842)
        rate.sleep()

        

