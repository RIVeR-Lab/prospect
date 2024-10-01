#!/usr/bin/env python3

import numpy as np
import rospy

class SetParams(object):
    def __init__(self):
        self.s = rospy.get_param('/setParams/s')
        self.a = rospy.get_param('/setParams/a')

    def calculations(self):
        # beta_i is the angle between the plane of rotation of servo arm i
        beta = list(map(lambda deg: float(np.radians(deg)), [180, 0, 300, 120, 60, 240]))

        # rotate the given point by the given angle (in degrees) around the z-axis
        def rotate(point, theta_deg):
            np_point = np.array(point)
            theta_rad = np.radians(theta_deg)
            rot_matrix = np.array(
                [[np.cos(theta_rad), -np.sin(theta_rad), 0], 
                 [np.sin(theta_rad), np.cos(theta_rad), 0],
                 [0,0,1]])
                
            res = rot_matrix @ np_point
            return [float(res[0]), float(res[1]), float(res[2])]

        def mirror_and_pattern(point):
            point1 = point
            point2 = [-point1[0], point1[1], point1[2]]
            point3 = rotate(point1, 120)
            point4 = rotate(point2, 120)
            point5 = rotate(point1, 240)
            point6 = rotate(point2, 240)
            return [point1, point2, point3, point4, point5, point6]

        b1 = [rospy.get_param('/setParams/b1_x'), rospy.get_param('/setParams/b1_y'), rospy.get_param('/setParams/b1_z')]
        b = mirror_and_pattern(b1)
        # b_i is the vector from the bottom origin to the link origins (in mm)

        p1 = [rospy.get_param('/setParams/p1_x'), rospy.get_param('/setParams/p1_y'), rospy.get_param('/setParams/p1_z')]
        p = mirror_and_pattern(p1)
        # p_i is the vector from the top origin to the link terminus (in mm)

        rospy.set_param('b', b)
        rospy.set_param('p', p)
        rospy.set_param('s', self.s)
        rospy.set_param('a', self.a)
        rospy.set_param('beta', beta)

        # Calculating the Z coordinate of the stewart platform's home position
        # z_home = float(np.sqrt(self.s**2 - self.a**2))
        rospy.set_param('z_home', 88)

    def run(self):
        self.calculations()

if __name__ == '__main__':
    rospy.init_node('SetParams', anonymous=True)
    try:
        my_node = SetParams()
        my_node.run()
    except rospy.ROSInterruptException:
        my_node.shutdown()
    
    
