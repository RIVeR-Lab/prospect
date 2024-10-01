#!/usr/bin/env python3

"""Script to move the stewart platform to randomly sampled configurations 
and compare desired poses with actual poses"""

import json
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R

import tf
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from stewart_end_effector.srv import StewartControl, StewartControlResponse
from optitrack_validation import average_tf


# the z coordinate of the stewart platform home position
Z_HOME = rospy.get_param('z_home')
latest_positions  = []

# TODO: write unit tests for this function
def stewart_pose_to_tf(pose):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    # TODO: check that these are the right frames (and accept them as command line args)
    t.header.frame_id = "stewart_zero"
    t.child_frame_id = "stewart_opti_v2"

    t.transform.translation.x = pose.x
    t.transform.translation.y = pose.y
    t.transform.translation.z = pose.z

    q = tf_conversions.transformations.quaternion_from_euler(pose.phi, pose.theta, pose.psi)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


# sleeps and then executes a stewart command and returns whether it was successful
def stewart_command_and_sleep(stewart_command, pose) -> bool:
    res = stewart_command(pose.x, pose.y, pose.z, pose.psi, pose.theta, pose.phi)
    rospy.sleep(4)
    print(f'{latest_positions},{res.positions},')
    return res.work
    


# TODO: check offset from top to base and increase position and rotation bounds
# None -> Tuple[Array]
def randomly_sample_poses():
    rospy.wait_for_service('stewart_control')
    

    # commanded and perceived positions (in meters)
    commanded_t = []
    perceived_t = []

    # commanded and perceived rotations (as quaternions)
    commanded_r = []
    perceived_r = []

    listener = tf.TransformListener()
    num_sampled = 0

    xmin = 0
    xmax = 0
    ymin = 0
    ymax = 0
    zmin = 90
    zmax = 110

    thetamin = -0.4
    thetamax = 0.4

    phimin = -0.4
    phimax = 0.4

    def from_bounds(min, max):
        return np.random.uniform(min, max)

    while num_sampled < 50:
        pose = StewartControl()
        pose.x,pose.y,pose.z,pose.psi,pose.theta,pose.phi = from_bounds(xmin, xmax), from_bounds(ymin, ymax), from_bounds(zmin, zmax), 0, from_bounds(thetamin, thetamax), from_bounds(phimin, phimax)

        stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)
        work = stewart_command_and_sleep(stewart_command, pose)
        if not work:
            continue
        
        num_sampled += 1
        pose_tf = stewart_pose_to_tf(pose)
        # Transforming the pose from mm to m to compare to the OptiTrack measurements
        expected_trans = np.array((pose_tf.transform.translation.x/1000, pose_tf.transform.translation.y/1000, pose_tf.transform.translation.z/1000))
        expected_rot = np.array((pose_tf.transform.rotation.x, pose_tf.transform.rotation.y, pose_tf.transform.rotation.z, pose_tf.transform.rotation.w))

        actual_tfs = []
        while not rospy.is_shutdown() and len(actual_tfs) < 5:
            try:
                (trans,rot) = listener.lookupTransform('/stewart_zero', '/stewart_top_origin', rospy.Time(0))
                actual_tfs.append((trans, rot))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        actual_tf = average_tf(actual_tfs)
        actual_trans, actual_rot = actual_tf

        (actual_trans, actual_rot) = listener.lookupTransform('/stewart_zero', '/stewart_top_origin', rospy.Time(0))

        commanded_t.append(expected_trans)
        commanded_r.append(expected_rot)
        perceived_t.append(np.array(actual_trans))
        perceived_r.append(np.array(actual_rot))

    commanded_t = np.array(commanded_t)
    perceived_t = np.array(perceived_t)
    commanded_r = np.array(commanded_r)
    perceived_r = np.array(perceived_r)

    # correcting for the offset from the top platform to the base platform (since commands are given wrt to base but perceived position is of top)
    perceived_to_command_z_trans = Z_HOME / 1000
    perceived_with_base_offset_t = perceived_t
    perceived_with_base_offset_t[:,2] = perceived_t[:,2] + perceived_to_command_z_trans

    print(f'Z OFFSET: {perceived_to_command_z_trans}')

    for i in range(len(commanded_t)):
        print(f'expected: {commanded_t[i]}')
        print(f'actual: {perceived_with_base_offset_t[i]}')


    # Position Error
    fig = plt.figure()

    ax = plt.axes(projection='3d')
    ax.scatter(commanded_t[:,0], commanded_t[:,1], commanded_t[:,2],c='blue')
    ax.scatter(perceived_with_base_offset_t[:,0], perceived_with_base_offset_t[:,1], perceived_with_base_offset_t[:,2], c='red')
    
    ax.set_xlim(-.1, .1)
    ax.set_ylim(-.1, .1)
    ax.set_zlim(-.1, .1)
    plt.show()

    def position_error(per, com):
        return np.sqrt((per[0]-com[0])**2 + (per[1]-com[1])**2 + (per[2]-com[2])**2)

    pos_err = [position_error(perceived_with_base_offset_t[i], commanded_t[i]) for i in range(len(commanded_t))]
    
    fig, ax = plt.subplots()

    plt.hist(pos_err, bins=20)   
    plt.title("Position Error")
    plt.xlabel("Distance from desired position (meters)")
    plt.show()

    avg_pos_err = sum(pos_err) / len(pos_err)
    print(f'Average position error: {avg_pos_err}')

    # Rotation Error
    rot_err = []
    for i in range(len(commanded_r)):
        per = perceived_r[i]
        com = commanded_r[i]

        per_mag = np.sqrt(sum([per[j] ** 2 for j in range(4)]))
        com_mag = np.sqrt(sum([com[j] ** 2 for j in range(4)]))

        per_unit = [per[j] / per_mag for j in range(4)]
        com_unit = [com[j] / com_mag for j in range(4)]

        # compute inner product
        # ip = sum([per[j] * com[j] for j in range(4)])
        ip = sum([per_unit[j] * com_unit[j] for j in range(4)])

        err = np.arccos(ip)
        rot_err.append(err)

    fig, ax = plt.subplots()

    plt.hist(rot_err, bins=20)   
    plt.show()

    avg_rot_err = sum(rot_err) / len(rot_err)
    print(f'Average rotation error metric: {avg_rot_err}')

    return commanded_t.tolist(), commanded_r.tolist(), perceived_t.tolist(), perceived_r.tolist()

def update_latest_servos(msg):
    global latest_positions 
    latest_positions = msg.data

def dump_to_json(results):
    commanded_t, commanded_r, perceived_t, perceived_r = results
    N = len(commanded_r)
    comparison_list = [{"commanded_trans": commanded_t[i], "perceived_trans": perceived_t[i], "commanded_rot": commanded_r[i], "perceived_rot": perceived_r[i]} for i in range(N)]

    with open("kinematics_random_verification_data.json", "w") as json_file:
        json.dump(comparison_list, json_file)

if __name__ == '__main__':
    rospy.init_node('OptitrackValidation', anonymous = True)

    sub = rospy.Subscriber('actual_motor_positions', Float32MultiArray, update_latest_servos)
    
    origin = StewartControl()
    origin.x, origin.y, origin.z, origin.psi, origin.theta, origin.phi = 0,0,Z_HOME,0,0,0
    stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)
    stewart_command_and_sleep(stewart_command, origin)

    results = randomly_sample_poses()
    dump_to_json(results)