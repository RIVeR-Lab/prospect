#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from stewart_end_effector.srv import StewartControl, StewartControlRequest, StewartControlResponse

def do_circle(radius: int):
    rospy.wait_for_service('stewart_control')
    try:
        stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)
        Z_HOME = rospy.get_param('z_home')
        while True:
            for x in range(0,360, 10):
                theta = np.deg2rad(x)
                stewart_command(radius * np.cos(theta), radius * np.sin(theta), Z_HOME, 0, 0, 0)
                rospy.sleep(0.1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    do_circle(50)