#! /usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import Int16
from simple_pid import PID
from guppy.msg import msg_GC_command

import Adafruit_PCA9685
import os

pid_bot = PID(2.5,0.50, 0.00, setpoint=20, sample_time=0.05,output_limits = (-300, 300))

surge = 0   ; heave = 0           
yaw = 0     ;sway = 0

Depth_setpoint = 0  ; Depth = 0    ; sub_bot_Connection = 0

#.........i2c_Detect.................
#data =[name for name in os.listdir("/sys/bus/pci/devices/0000:00:19.0/i2c_designware.4") ]
#matching = [s for s in data if "i2c" in s]
#i2c_port = int(matching[0][-1])

pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=1)
pwm.set_pwm_freq(200)

#..........................
servo_offset = 15
esc_start = 1260

def GC_command_CB(msg):
    global surge    ; global heave
    global yaw      ; global sway
    global Depth_setpoint
    surge =  msg.engy_twist.linear_x
    heave = msg.engy_twist.linear_z
    yaw = msg.engy_twist.angular_z
    sway = msg.engy_twist.linear_y
    Depth_setpoint = msg.Depth_setpoint  

def robot_Depth_CB(msg):
    global Depth
    Depth = msg.data  

def robot_ConnectionCB(msg):
    global sub_bot_Connection
    sub_bot_Connection = msg.data  

if __name__ == '__main__':
    pwm.set_pwm(4, 0, esc_start)
    pwm.set_pwm(5, 0, esc_start)

    sub_depth = rospy.Subscriber("robot_Depth", Int16, robot_Depth_CB)
    subSetpoint = rospy.Subscriber("GC_command", msg_GC_command, GC_command_CB)
    subConnection = rospy.Subscriber("Connection", Int16, robot_ConnectionCB)

    rospy.init_node('esc_all', anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if (sub_bot_Connection == 1):
            esc4 = 0 
            esc5 = 0 
     # Heave control........................................
            if Depth_setpoint > 5500 :
                pid_bot.reset()
                esc4 = heave
                esc5 = heave
                print("{0}, {1}".format(esc4, esc5))
            else:
                pid_bot.setpoint = Depth_setpoint
                output = int(pid_bot(Depth))
                esc4 = output
                esc5 = output
                print("Auto {0}, {1}".format(output, output))
            print(esc4)
            if esc4==0:
                pwm.set_pwm(4, 0, esc_start)
            elif esc4<0:
                pwm.set_pwm(4, 0, esc_start + esc4 - servo_offset)
            else:
                pwm.set_pwm(4, 0, esc_start + esc4 + servo_offset)

            if esc5==0:
                pwm.set_pwm(5, 0, esc_start)
            elif esc5<0:
                pwm.set_pwm(5, 0, esc_start + esc5 - servo_offset)
            else:
                pwm.set_pwm(5, 0, esc_start + esc5 + servo_offset)
        else:       
            pwm.set_pwm(4, 0, esc_start)
            pwm.set_pwm(5, 0, esc_start)
        rate.sleep()

        

