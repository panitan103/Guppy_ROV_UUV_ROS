#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import Int16
from time import sleep; from serial import Serial;import struct
from guppy.msg import msg_GC_command

pan = 3
tilt = 3
PVpan = 3
PVtilt = 3
speed = 3000

pan_center = 490
pan_right = 200
pan_left = 700

tilt_center = 515
tilt_up = 200
tilt_down = 700
'''
pan_center = 640
pan_right = 250
pan_left = 1000
tilt_center = 610
tilt_up = 250
tilt_down = 750
'''

#..........................
class LX16A:
  def __init__(self,Port="/dev/ttyS4",Baudrate=115200, Timeout= 0.001):
     self.serial = Serial(Port,baudrate=Baudrate,timeout=Timeout)
     self.serial.setDTR(1);self.TX_DELAY_TIME = 0.00002
     self.Header = struct.pack("<BB",0x55,0x55)
  def sendPacket(self,packet):
     packet1=bytearray(packet);sum=0
     for a in packet1: sum=sum+a
     fullPacket = bytearray(self.Header + packet + struct.pack("<B",(~sum) & 0xff))
     self.serial.write(fullPacket); sleep(self.TX_DELAY_TIME)
  def sendReceivePacket(self,packet,receiveSize):
     t_id = packet[0];t_command = packet[2]
     self.serial.flushInput();self.serial.timeout=0.1;self.sendPacket(packet)
     r_packet = self.serial.read(receiveSize+3); return r_packet 
  def motorOrServo(self,id,motorMode,MotorSpeed):
     packet = struct.pack("<BBBBBh",id,7,29,motorMode,0,MotorSpeed)
     self.sendPacket(packet)# motorMode 1=motor MotorSpeed=rate, 2=servo 
  def moveServo(self,id,position,rate=1000):
     packet = struct.pack("<BBBHH",id,7,1,position,rate)
     self.sendPacket(packet)  # Move servo 0-1000, rate(ms) 0-30000(slow)
  def moveServoStop(self,id):
     packet = struct.pack("<BBB",id,3,12)
     self.sendPacket(packet)
  def readPosition(self,id):
     packet = struct.pack("<BBB",id,3,28)
     rpacket = self.sendReceivePacket(packet,5)
     s = struct.unpack("<BBBBBhB",rpacket);return s[5] 
  def LoadUnload(self,id,mode):
     packet = struct.pack("<BBBB",id,4,31,mode)
     self.sendPacket(packet)#Activate motor 0=OFF 1 =Active
  def setID(self,id,newid):# change the ID of servo
     packet = struct.pack("<BBBB",id,4,13,newid)
     self.sendPacket(packet)
  def readservosABS(self):
    m=[0]*6
    for a in range(0,6): m[a]= self.readPosition(a+1)
    return m
  def readservos(self, c):
    m=[0]*6
    for a in range(0,6): m[a]= self.readPosition(a+1)-c[a]
    return m
  def servosoff(self):
    for a in range(1,7): self.LoadUnload(a,0)
  def servoson(self):
    for a in range(1,7): self.LoadUnload(a,1)
  def close(self):
    self.serial.close()

def robot_panCB(msg):
    global pan
    global tilt
    pan = msg.rc_sw.vr_A
    tilt = msg.rc_sw.vr_B

if __name__ == '__main__':
    subPan = rospy.Subscriber("GC_command", msg_GC_command, robot_panCB)
    rospy.init_node('camera_pantilt', anonymous=True)
    rate = rospy.Rate(10)
    m1 = LX16A()

    while not rospy.is_shutdown():
        print(pan,PVpan,tilt, PVtilt )
        if pan != PVpan :
            if pan == 0:
                m1.moveServoStop(0)
            elif pan == 1:
                m1.moveServo(0,pan_right,speed) #pan
            elif pan == 2:
                m1.moveServo(0,pan_left,speed) #pan
            elif pan == 3:
                m1.moveServo(0,pan_center,speed) #pan
        PVpan = pan

        if tilt != PVtilt :
            if tilt == 0:
                m1.moveServoStop(1)
            elif tilt == 1:
                m1.moveServo(1,tilt_down,speed) #pan
            elif tilt == 2:
                m1.moveServo(1,tilt_up,speed) #pan
            elif tilt == 3:
                m1.moveServo(1,tilt_center,speed) #pan
        PVtilt = tilt
        rate.sleep()

        

