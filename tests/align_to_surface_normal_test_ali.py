#!/usr/bin/env python3

import numpy as np
import rospy

from stewart_end_effector.msg import CoordinateData
from stewart_end_effector.scripts.findSurfaceNormalTransform import findSurfaceNormalTransform
from stewart_end_effector.srv import StewartControl, StewartControlRequest, StewartControlResponse



# TODO: write a callback function and then after it has been called and successfully moved the platform to the new normal, exit the program
def callback(coordinate_data):
    tof_readings = coordinate_data.z
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", tof_readings)


    # NOTE: need to move the stewart platform to the following pose before running this script
    xyz_in = [0, 0, 90]
    psi, theta, phi = [0, 0, 0]

    # in mm
    surface_offset = 50
    SENSOR_RADIUS = 12            # TODO: Set proper radius (12 is just a guess, check the pack n go)


    """ TOF SENSOR CLOCKING: 
    #1 is on the y-axis (theta=90deg)
    #2 is below the -x-axis (theta=210deg)
    #3 is below the x-axis (theta=330deg)

    Therefore SENSOR_CLOCKING needs to be 90deg = pi/2
    """

    # SENSOR_CLOCKING = np.pi/2     # TODO: This is the clocking that Vedant measured manually but for some reason is wrong (double check the above logic)
    SENSOR_CLOCKING = np.pi     # TODO: Set proper clocking
    trans = findSurfaceNormalTransform(psi, theta, phi, xyz_in, tof_readings, surface_offset, SENSOR_RADIUS, SENSOR_CLOCKING)

    print(trans)
    print(f'psi theta phi: {np.flip(trans.rpy())}')
    print(f'translation: {trans.t}')
    
    try:
      stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)

      x, y, z = trans.t
      phi, theta, psi = trans.rpy()
      print(f'Commanding to move to {x, y, z, psi, theta, phi}') 
      stewart_command(x,y,z,psi,theta,phi)
      rospy.sleep(0.1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.signal_shutdown("Shutting down...")

     
def listener():
  rospy.init_node('listener', anonymous=True)
   
  rospy.Subscriber("/tof/coords", CoordinateData, callback)
  rospy.spin()


# reading tof data, getting the current pose, finding surface normal pose, and then calling the IK solver
if __name__ == '__main__':
    listener()