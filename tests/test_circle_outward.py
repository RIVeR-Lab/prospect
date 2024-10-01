#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from stewart_end_effector.srv import StewartControl, StewartControlRequest, StewartControlResponse

def do_circle(radius: int):
    rospy.wait_for_service('stewart_control')
    try:
        stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)

        h = 100
        Z_HOME = rospy.get_param('z_home')
        while True:
            for x in range(0,360, 10):
                alpha = np.deg2rad(x)

                x = radius * np.cos(alpha)
                y = radius * np.sin(alpha)

                denom = np.sqrt(radius**2 + h**2)

                psi = 0
                theta = np.arcsin(-x/denom)
                phi = np.arcsin(y/denom)

                print(f'(theta, phi) = ({theta}, {phi})')

                stewart_command(x, y, Z_HOME, psi, -theta, -phi)
                rospy.sleep(0.1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    do_circle(30)