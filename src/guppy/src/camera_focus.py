#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import Int16
import subprocess
from guppy.msg import msg_GC_command

sub_focus = 0
PVsub_focus = 0
#..........................

def robot_focusCB(msg):
    global sub_focus
    sub_focus = msg.focus  

if __name__ == '__main__':
    set_contrast = 170
    set_saturation = 170

    subFocus = rospy.Subscriber("CG_command", msg_GC_command, robot_focusCB)

    rospy.init_node('camera_focus', anonymous=True)
    rate = rospy.Rate(10)

    contrast_absolutecommnd = "--set-ctrl=contrast=" + str(set_contrast)
    saturation_absolutecommnd = "--set-ctrl=saturation=" + str(set_saturation)
    subprocess.call(["v4l2-ctl", "--device=/dev/video0",contrast_absolutecommnd])
    subprocess.call(["v4l2-ctl", "--device=/dev/video0",saturation_absolutecommnd])

    while not rospy.is_shutdown():
        if sub_focus != PVsub_focus :
            if sub_focus == 0 :
                subprocess.call(["v4l2-ctl", "--device=/dev/video0","--set-ctrl=focus_auto=1"])
            else:
                focus_absolutecommnd = "--set-ctrl=focus_absolute=" + str(sub_focus)
                subprocess.call(["v4l2-ctl", "--device=/dev/video0","--set-ctrl=focus_auto=0"])
                subprocess.call(["v4l2-ctl", "--device=/dev/video0",focus_absolutecommnd])

        PVsub_focus = sub_focus
        rate.sleep()

        

