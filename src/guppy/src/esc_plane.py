#! /usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import String, Int16, Float32
from simple_pid import PID
from geometry_msgs.msg import Point32, Twist
from guppy.msg import msg_GC_command

import Adafruit_PCA9685
import os

pid_yaw = PID(0.2, 0, 0.00, setpoint=20, sample_time=0.05,output_limits = (-200, 200)) #0.01

surge = 0   ; heave = 0           
yaw = 0     ;sway = 0
angle_lock_Degree = 0.00
sub_bot_Connection = 0
imu_roll = 0    ; imu_pitch = 0
imu_yaw = 0     ; angle_lock_Enable = 0
#.........i2c_Detect.................
#data =[name for name in os.listdir("/sys/bus/pci/devices/0000:00:19.0/i2c_designware.4") ]
#matching = [s for s in data if "i2c" in s]
#i2c_port = int(matching[0][-1])

pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=1)
pwm.set_pwm_freq(200)

#..........................
esc_start = 1260 ; servo_offset = 15

def GC_command_CB(msg):
    global surge    ; global heave
    global yaw      ; global sway
    global Depth_setpoint
    global angle_lock_Enable
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

def ahrsCB(msg):
    global imu_roll ; global imu_pitch ; global imu_yaw
    imu_roll =  int(msg.x)
    imu_pitch =  int(msg.y)
    imu_yaw =  int(msg.z)

if __name__ == '__main__':
    pwm.set_pwm(0, 0, esc_start) ; pwm.set_pwm(1, 0, esc_start)
    pwm.set_pwm(2, 0, esc_start) ; pwm.set_pwm(3, 0, esc_start)

    subSetpoint = rospy.Subscriber("GC_command", msg_GC_command, GC_command_CB)
    subConnection = rospy.Subscriber("Connection", Int16, robot_ConnectionCB)
    subAHRS = rospy.Subscriber("ahrs", Point32, ahrsCB)

    rospy.init_node('esc_all', anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if (sub_bot_Connection == 1):
            esc0 = 0 ; esc1 = 0 ; esc2 = 0 ; esc3 = 0 
     # Surge control........................................
            esc0 -= surge 
            esc1 += surge 
            esc2 += surge 
            esc3 -= surge
     # Yaw control........................................
            if yaw < 450 :
                pid_yaw.setpoint = 500
                pid_yaw.reset()
                esc0 -= yaw 
                esc1 += yaw 
                esc2 -= yaw 
                esc3 += yaw
            else:
                if pid_yaw.setpoint == 500: angle_lock_Degree = imu_yaw
                pid_yaw.setpoint = angle_lock_Degree
                output = int(pid_yaw(imu_yaw))
                esc0 -= output 
                esc1 += output 
                esc2 -= output 
                esc3 += output
     # sway control........................................
            esc0 -= sway 
            esc1 -= sway 
            esc2 += sway 
            esc3 += sway
     # Output control........................................
            
            if esc0==0:
                pwm.set_pwm(0, 0, esc_start)          
            elif esc0 < 0:
                pwm.set_pwm(0, 0, esc_start + esc0 - servo_offset)  
            else:
                pwm.set_pwm(0, 0, esc_start + esc0 + servo_offset)  
            
            if esc1==0:
                pwm.set_pwm(1, 0, esc_start) 
            elif esc1<0:
                pwm.set_pwm(1, 0, esc_start + esc1 - servo_offset)
            else:
                pwm.set_pwm(1, 0, esc_start + esc1 + servo_offset)
            
            
            if esc2==0:
                pwm.set_pwm(2, 0, esc_start)  
            elif esc2<0:
                pwm.set_pwm(2, 0, esc_start + esc2 - servo_offset)
            else:
                pwm.set_pwm(2, 0, esc_start + esc2 + servo_offset)
                
            
            if esc3==0:
                pwm.set_pwm(3, 0, esc_start)      
            elif  esc3<0:
                pwm.set_pwm(3, 0, esc_start + esc3 - servo_offset)  
            else:
                pwm.set_pwm(3, 0, esc_start + esc3 + servo_offset)  
            
                     
  
        else:
            pwm.set_pwm(0, 0, esc_start) ; pwm.set_pwm(1, 0, esc_start)
            pwm.set_pwm(2, 0, esc_start) ; pwm.set_pwm(3, 0, esc_start)
        rate.sleep()

        

