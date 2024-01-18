#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16
import ms5837
import time

# Depth Kalman Parameter
#err_measure_0 = 5.0
#err_estimate_0 = 40.0
#pro_variance_0 = 0.5

#kalman_gain_0 = 0.0
#current_estimate_0 = 0.0
#last_estimate_0 = 0.0

sensor = ms5837.MS5837_30BA(1)  #ms5837.MS5837_30BA
# time.sleep(5)

if __name__ == "__main__":
    pub_Depth = rospy.Publisher('robot_Depth', Int16, queue_size = 20)
    rospy.init_node('pub_Depth', anonymous=True)
    r = rospy.Rate(20)
    if not sensor.init():
        print("Sensor could not be initialized")
    try:     
 
        while not rospy.is_shutdown():
            # Read Front Senser
            try:
                if sensor.read():
                    
                    freshwaterDepth = int(sensor.depth()* (100.0) )# * 100 to cm 
                    

                    # kalman_gain_0 = err_estimate_0/(err_estimate_0 + err_measure_0)
                    # current_estimate_0 = last_estimate_0 + kalman_gain_0 * (freshwaterDepth - last_estimate_0)
                    # err_estimate_0 =  (1.0 - kalman_gain_0)*err_estimate_0 + abs(last_estimate_0 - current_estimate_0) * pro_variance_0
                    # last_estimate_0 = current_estimate_0

                pub_Depth.publish(freshwaterDepth)
                print(freshwaterDepth)
            except:
                print("Error") 
            r.sleep()
           
    except KeyboardInterrupt:
        pass
