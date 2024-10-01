#!/usr/bin/env python3

import rospy

import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('my_static_tf2_broadcaster')

    # these values are set by running zero_platform_and_record_origin_tf.py
    trans, rot = (rospy.get_param('zero_translation'), rospy.get_param('zero_rotation'))


    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "stewart_world"
    static_transformStamped.child_frame_id = "stewart_zero"

    static_transformStamped.transform.translation.x = trans[0]
    static_transformStamped.transform.translation.y = trans[1]
    static_transformStamped.transform.translation.z = trans[2]

    
    static_transformStamped.transform.rotation.x = rot[0]
    static_transformStamped.transform.rotation.y = rot[1]
    static_transformStamped.transform.rotation.z = rot[2]
    static_transformStamped.transform.rotation.w = rot[3]
    broadcaster.sendTransform(static_transformStamped)

    rospy.spin()