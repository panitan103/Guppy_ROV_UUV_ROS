#! /usr/bin/env python

import pandas as pd
import time
import asyncore
import socket
import rospy
from std_msgs.msg import String, Int16,Int32, Float32, Float64
from guppy.msg import msg_voltage
from guppy.msg import msg_GC_command
from geometry_msgs.msg import Point32, Twist, Point

Depth = 0       ; voltage = 0
water = 0       ; current = 0
roll = 0        ; pitch = 0     ; yaw = 0

command=[]
def DepthCB(msg):
    global Depth
    #print(msg.data)
    Depth = msg.data

def voltageCB(msg):
    global voltage
    global water
    global current
    voltage =  msg.voltage
    water =  msg.water
    current =  msg.current
    #print(voltage)

def ahrsCB(msg):
    global roll
    global pitch
    global yaw
    roll =  msg.x
    pitch =  msg.y
    yaw =  msg.z

def logger():
    global file_location,localtime,command

    data_water = int(water/1000)
    localtime = time.localtime()
    #localtime.tm_year, localtime.tm_mon, localtime.tm_mday, localtime.tm_hour, localtime.tm_min, localtime.tm_sec
    date_time = "{0}.{1}.{2}".format(localtime.tm_hour, localtime.tm_min, localtime.tm_sec) 

    command.append([date_time,Depth, round(voltage,2)/100, current/100,data_water, round(roll,2), round(pitch,2), round(yaw,2)])

    df = pd.DataFrame(command, columns =['Time','Depth', 'Voltage','Current', 'data_water', 'roll', 'pitch', 'yaw'], dtype = float)
    df.to_csv(file_location,index=False)  
    #print(result)
    print(df)
    r.sleep()
if __name__ == '__main__':
    subDepth = rospy.Subscriber("robot_Depth", Int16, DepthCB)
    subAHRS = rospy.Subscriber("ahrs", Point32, ahrsCB)
    subVolt = rospy.Subscriber("voltage", msg_voltage, voltageCB)

    rospy.init_node('logger', anonymous=True)    
    r = rospy.Rate(5)
    localtime = time.localtime()
    file_location='/home/guppy/catkin_ws/src/guppy/log/{0}-{1}-{2}_{3}:{4}:{5}.csv'.format(localtime.tm_year, localtime.tm_mon, localtime.tm_mday, localtime.tm_hour, localtime.tm_min, localtime.tm_sec)

    while not rospy.is_shutdown():
        logger()