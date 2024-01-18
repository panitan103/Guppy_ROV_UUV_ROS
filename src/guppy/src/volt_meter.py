#! /usr/bin/env python

import rospy
import sys
import os
import time

sys.path.append('../')
from std_msgs.msg import Int32, Int16
from guppy.msg import msg_voltage

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_ADS1115 import ADS1115

ADS1115_REG_CONFIG_PGA_6_144V        = 0x00 # 6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_4_096V        = 0x02 # 4.096V range = Gain 1
ADS1115_REG_CONFIG_PGA_2_048V        = 0x04 # 2.048V range = Gain 2 (default)
ADS1115_REG_CONFIG_PGA_1_024V        = 0x06 # 1.024V range = Gain 4
ADS1115_REG_CONFIG_PGA_0_512V        = 0x08 # 0.512V range = Gain 8
ADS1115_REG_CONFIG_PGA_0_256V        = 0x0A # 0.256V range = Gain 16
ads1115 = ADS1115()



if __name__ == "__main__":
	pub_volt = rospy.Publisher('voltage', msg_voltage, queue_size = 10)
	pubconnect = rospy.Publisher('Connection', Int16, queue_size=20)

	rospy.init_node('analog_input')
	ads1115.set_addr_ADS1115(0x48)
    #Sets the gain and input voltage range.
	ads1115.set_gain(ADS1115_REG_CONFIG_PGA_6_144V)
    #Get the Digital Value of Analog of selected channel
	r = rospy.Rate(5)
	
	while not rospy.is_shutdown():
		values0 = int(ads1115.read_voltage(0)['r'])
		values1 = int(ads1115.read_voltage(1)['r'])
		values2 = int(ads1115.read_voltage(2)['r'])
		values3 = int(ads1115.read_voltage(3)['r'])

			
		msg = msg_voltage()
		print("raw : {0} {1} {2}".format(values1, values0, values3))
		#1583
		msg.current = (values0-1583.0)*(3.3/4095.0)*100/(33*0.001)
		# msg.current = ((values0-1583.0)*100/4095.0)*100

		# msg.current = values0/1550*(-3.4)
		msg.voltage = ((values1/1248.0)*12.87)*100
		msg.water = values3

		print("trans : {0} {1} {2}".format( msg.current, msg.voltage,msg.water))

		pub_volt.publish(msg)
		r.sleep()
