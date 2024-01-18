#! /usr/bin/env python
from __future__ import division
import time
import asyncore
import socket
import rospy
from std_msgs.msg import String, Int16,Int32, Float32, Float64
from guppy.msg import msg_voltage
from guppy.msg import msg_GC_command
from geometry_msgs.msg import Point32, Twist, Point

HOST_NAME = '' ; HOST_PORT = 5060

Depth = 0       ; voltage = 0
water = 0       ; current = 0
roll = 0        ; pitch = 0     ; yaw = 0

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

class EchoHandler(asyncore.dispatcher_with_send):
    def handle_read(self):
        data_water = int(water/1000)
        command = "x {0} {1} {2} {3} {4} {5} {6} x".format(Depth, voltage, current, data_water, roll, pitch, yaw) # cm mV mDegree
        self.send(command.encode('utf-8'))

        data = self.recv(1024)
        
        if not data:
            self.close()
        
        data = data.decode('utf-8')
        dataMessage = data.split(' ')
        print(dataMessage)
        #print(data)
        if dataMessage[0] != 'x':
            return
        if dataMessage[10] != 'x':
            return
        #commands = "{0} {1} {2}".format(dataMessage[8], dataMessage[9], dataMessage[10]) # cm mV mDegree
        #print(commands)

        data1 = int(dataMessage[1])
        data2 = int(dataMessage[2])
        data3 = int(dataMessage[3])
        data4 = int(dataMessage[4])
        data5 = int(dataMessage[5])
        data6 = int(dataMessage[6])
        data7 = int(dataMessage[7])
        data8 = int(dataMessage[8])
        data9 = int(dataMessage[9])

        print("{0} {1} {2} {3} {4}".format(data1,data2,data3,data4,data5))

        msg_GC = msg_GC_command()
        msg_GC.engy_twist.linear_z = data1
        msg_GC.engy_twist.angular_z = data2
        msg_GC.engy_twist.linear_x = data3
        msg_GC.engy_twist.linear_y = data4
        msg_GC.Depth_setpoint = data5

        msg_GC.rc_sw.sw_C = data6
        msg_GC.rc_sw.vr_A = data7 
        msg_GC.rc_sw.vr_B = data8
        msg_GC.rc_sw.grp = data9         
        pubconnect.publish(1)
        pub_CG_command.publish(msg_GC)

    def handle_close(self):
        print ("Disconnection from")
        self.close()
        pubconnect.publish(0)

class EchoServer(asyncore.dispatcher):
    def __init__(self, ip, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((ip, port))
        self.listen(1)

    def handle_accept(self):
        sock, addr = self.accept()
        print ("Connection from", addr)
        EchoHandler(sock)

if __name__ == '__main__':
    pub_CG_command = rospy.Publisher('GC_command', msg_GC_command, queue_size = 20)
    subDepth = rospy.Subscriber("robot_Depth", Int16, DepthCB)
    subAHRS = rospy.Subscriber("ahrs", Point32, ahrsCB)
    subVolt = rospy.Subscriber("voltage", msg_voltage, voltageCB)
    pubconnect = rospy.Publisher('Connection', Int16, queue_size=20)

    rospy.init_node('tcp_pub', anonymous=True)    

    s = EchoServer(HOST_NAME, HOST_PORT)
    asyncore.loop() 
