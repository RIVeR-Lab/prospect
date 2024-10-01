#!/usr/bin/env python3


import rospy
from stewart_end_effector.srv import StewartControl, StewartControlResponse

import tf
import tf_conversions
import tf2_ros
import geometry_msgs.msg

from optitrack_validation import stewart_command_and_sleep, average_tf


if __name__ == '__main__':
    rospy.init_node('OptitrackValidation', anonymous = True)

    Z_HOME = rospy.get_param('z_home')
    
    origin = StewartControl()
    origin.x, origin.y, origin.z, origin.psi, origin.theta, origin.phi = 0,0,Z_HOME,0,0,0
    stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)
    stewart_command_and_sleep(stewart_command, origin)

    origin_tfs = []
    listener = tf.TransformListener()
    while not rospy.is_shutdown() and len(origin_tfs) < 5:
        try:
            (trans,rot) = listener.lookupTransform('/stewart_world', '/stewart_top_origin', rospy.Time(0))
            origin_tfs.append((trans, rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    origin_tf = average_tf(origin_tfs)
    print(origin_tf)

    rospy.set_param('zero_translation', origin_tf[0])
    rospy.set_param('zero_rotation', origin_tf[1])