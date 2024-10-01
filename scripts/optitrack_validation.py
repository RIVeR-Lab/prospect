#!/usr/bin/env python3

"""Script to move the stewart platform to various configurations 
to OptiTrack system measurements to desired poses"""

import json
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
from stewart_end_effector.srv import StewartControl, StewartControlResponse

import tf
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Run the appropriate rosbag command before running this script
# 1. generate a list of desired poses for the platform to reach
# 2. for each pose
#   a. command the platform to go to that pose
#   b. sleep for a short while (~50ms)
#   c. record the desired pose as a tf and lookup the tf between the stewart_zero and stewart_opti_v2 frames (or whichever are appropriate)

# the z coordinate of the stewart platform home position
Z_HOME = rospy.get_param('z_home')
latest_positions  = []

def desired_test_circle_inward_poses():
    desired_poses = []
    
    h = 100
    radius = 20
    for x in range(0,360, 5):
        alpha = np.deg2rad(x)

        x = radius * np.cos(alpha)
        y = radius * np.sin(alpha)

        denom = np.sqrt(radius**2 + h**2)

        psi = 0
        theta = np.arcsin(-x/denom)
        phi = np.arcsin(y/denom)

        pose = StewartControl()
        pose.x,pose.y,pose.z,pose.psi,pose.theta,pose.phi = x,y,Z_HOME,psi,theta,phi 
        desired_poses.append(pose)
    return desired_poses

def desired_test_circle_outward_poses():
    desired_poses = []
    
    h = 100
    radius = 20
    for x in range(0,360, 5):
        alpha = np.deg2rad(x)

        x = radius * np.cos(alpha)
        y = radius * np.sin(alpha)

        denom = np.sqrt(radius**2 + h**2)

        psi = 0
        theta = np.arcsin(-x/denom)
        phi = np.arcsin(y/denom)

        pose = StewartControl()
        pose.x,pose.y,pose.z,pose.psi,pose.theta,pose.phi = x,y,Z_HOME,psi,-theta,-phi 
        desired_poses.append(pose)
    return desired_poses


def desired_test_bounce_poses():
    desired_poses = []
    for z in range(90,130,2):
        pose = StewartControl()
        pose.x,pose.y,pose.z,pose.psi,pose.theta,pose.phi = 0,0,z,0,0,0 
        desired_poses.append(pose)
    for z in range(130,90,-2):
        pose = StewartControl()
        pose.x,pose.y,pose.z,pose.psi,pose.theta,pose.phi = 0,0,z,0,0,0 
        desired_poses.append(pose)
    return desired_poses

def desired_test_circle_poses():
    desired_poses = []
    radius = 20
    for x in range(0,360, 10):
        theta = np.deg2rad(x)
        pose = StewartControl()
        pose.x,pose.y,pose.z,pose.psi,pose.theta,pose.phi = radius * np.cos(theta), radius * np.sin(theta), Z_HOME, 0, 0, 0 
        desired_poses.append(pose)
    
    return desired_poses

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


def average_tf(tfs):
    sum_trans = [0,0,0]
    sum_rot = [0,0,0,0]
    for tf in tfs:
        trans = tf[0]
        rot = tf[1]
        sum_trans = [sum(x) for x in zip(trans, sum_trans)]
        sum_rot = [sum(x) for x in zip(rot, sum_rot)]
    

    avg_trans = [sum_coord / len(tfs) for sum_coord in sum_trans]
    avg_rot = [sum_coord / len(tfs) for sum_coord in sum_rot]
    return (avg_trans, avg_rot)


def stewart_command_and_sleep(stewart_command, pose):
    res = stewart_command(pose.x, pose.y, pose.z, pose.psi, pose.theta, pose.phi)
    print(f'{latest_positions},{res.positions},')
    rospy.sleep(1)

def update_latest_servos(msg):
    global latest_positions 
    latest_positions = msg.data

# List[StewartControl] -> List[PoseInfo]
def iterate_over_poses(poses):
    # TODO: add error handling for service calls
    rospy.wait_for_service('stewart_control')

    commanded_t = []
    perceived_t = []
    commanded_r = []
    perceived_r = []

    listener = tf.TransformListener()
    for pose in poses:
        stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)
        stewart_command_and_sleep(stewart_command, pose)
        

        # compute the expected tf of the given pose with the translation in meters and rotation as a quaternion 
        pose_tf = stewart_pose_to_tf(pose)
        expected_trans = (pose_tf.transform.translation.x/1000, pose_tf.transform.translation.y/1000, pose_tf.transform.translation.z/1000)
        expected_rot = (pose_tf.transform.rotation.x, pose_tf.transform.rotation.y, pose_tf.transform.rotation.z, pose_tf.transform.rotation.w)


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
        perceived_t.append(actual_trans)
        perceived_r.append(actual_rot)

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
    
    # ax.set_xlim(-.1, .1)
    # ax.set_ylim(-.1, .1)
    # ax.set_zlim(-.1, .1)
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


def dump_to_json(results):
    commanded_t, commanded_r, perceived_t, perceived_r = results
    N = len(commanded_r)
    comparison_list = [{"commanded_trans": commanded_t[i], "perceived_trans": perceived_t[i], "commanded_rot": commanded_r[i], "perceived_rot": perceived_r[i]} for i in range(N)]

    with open("kinematics_verification_data.json", "w") as json_file:
        json.dump(comparison_list, json_file)


if __name__ == '__main__':
    rospy.init_node('OptitrackValidation', anonymous = True)
    sub = rospy.Subscriber('actual_motor_positions', Float32MultiArray, update_latest_servos)

    origin = StewartControl()
    origin.x, origin.y, origin.z, origin.psi, origin.theta, origin.phi = 0,0,Z_HOME,0,0,0
    stewart_command = rospy.ServiceProxy('stewart_control', StewartControl)
    stewart_command_and_sleep(stewart_command, origin)

    results = iterate_over_poses(desired_test_bounce_poses())
    dump_to_json(results)