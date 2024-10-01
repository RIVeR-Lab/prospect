#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from stewart_end_effector.srv import StewartControl, StewartControlRequest, StewartControlResponse

def do_bounce():
    rospy.wait_for_service('stewart_control')
    try:
        stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)
        while True:
            for z in range(90,130,2):
                stewart_command(0,0,z, 0, 0, 0)
                rospy.sleep(1)
            for z in range(130,90,-2):
                stewart_command(0,0,z, 0, 0, 0)
                rospy.sleep(1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    do_bounce()