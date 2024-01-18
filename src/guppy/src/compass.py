#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin, cos, pi, degrees

import rospy
import tf
from geometry_msgs.msg import (Point32, Pose2D, Quaternion, TransformStamped,
                               Twist, Vector3)
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32,Int16
from sensor_msgs.msg import Imu
roll = 0.00 ; pitch = 0.00  ; yaw = 0.00

def imuCB(msg):
    global roll ; global pitch ; global yaw
    q = msg.orientation
    quaternion = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = degrees(euler[1]) 
    if (degrees(euler[0])) < 0 :
        pitch = (degrees(euler[0])) + 180
    else:
        pitch = (degrees(euler[0])) - 180
    yaw = (degrees(euler[2])) * (-1)


if __name__ == '__main__':
    rospy.init_node('bot_compass')
    subImu = rospy.Subscriber("imu/data", Imu, imuCB)
    compass_pub = rospy.Publisher('ahrs', Point32, queue_size = 10)

    rate = 20.0

    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        pubIMU = Point32()
        pubIMU.x = roll
        pubIMU.y = pitch
        pubIMU.z = yaw       
        compass_pub.publish(pubIMU)
        
        r.sleep()
