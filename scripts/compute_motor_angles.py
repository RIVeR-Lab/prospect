#!/usr/bin/env python3

import numpy as np
import rospy
from stewart_end_effector.srv import ComputeMotorAngles, ComputeMotorAnglesResponse


b, p, a, s, beta = None, None, None, None, None

# the max angle to which a servo can be rotated 
MAX_SERVO_ANGLE_RADIANS = np.radians(90)
# the minimum angle to which a servo can be rotated (normalized to be between -pi and pi)
MIN_SERVO_ANGLE_RADIANS = np.radians(-60) 


# Computes l^2 - (s^2 - a^2)
# Float -> Float
def compute_L(l_i):
    return np.square(l_i) - (np.square(s) - np.square(a))

# Computes 2a*l_i_z
# Float -> Float
def compute_M(l_i_z):
    return 2*a*l_i_z

# Computes 2a[cos(beta_i)(l_i_x) + sin(beta_i)(l_i_y)]
# Int Float Float -> Float
def compute_N(i, l_i_x, l_i_y):
    return 2*a*(np.cos(beta[i])*(l_i_x) + np.sin(beta[i])*(l_i_y))

# Compute the angle (in radians) of the ith motor based on its effective leg vector and the desired pose
# Note: returns np.nan if there is no angle that can reach the desired pose
# Int np.array -> Float
def compute_motor_angle(i, l_i_vec):
    # x_b, y_b, z_b = b[i]
    # AHA, this is wrong! (x_p, y_p, z_p) is the coordinates of the link terminus point on the top platform
    # x_p, y_p, z_p = req.x, req.y, req.z

    l_i_x, l_i_y, l_i_z = tuple(l_i_vec)
    l_i = np.linalg.norm(l_i_vec)

    # TODO: change notation to the Ethan paper format
    L = compute_L(l_i)
    M = compute_M(l_i_z)
    N = compute_N(i, l_i_x, l_i_y)

    try:
        arcsin_arg = L/(np.sqrt(np.square(M) + np.square(N))) 
        alpha_i = np.arcsin(arcsin_arg) - np.arctan2(N, M)
        return alpha_i
    except:
        return np.nan

# Computes the effective leg vector of the ith leg based on the 
# translation vector from base to platform and the rotation matrix
# Int np.array[np.array[float]] ComputeMotorAngles -> np.array
def compute_effective_leg_vec(i, rotation_matrix, req):
    x,y,z = req.x, req.y, req.z
    T = np.array([x,y,z])
    # TODO: why do b and p not match the rosparams at this point in the code?
    l_i_vec = T + (rotation_matrix @ p[i]) - b[i]
    # l_i = np.linalg.norm(l_i_vec)
    return l_i_vec

# Computes the 3d rotation matrix given the following angles (in radians): 
# psi (around z-axis), theta (around y-axis), and phi (around x-axis) 
# ComputeMotorAngles -> np.array[np.array[float]]
def compute_rotation_matrix(req):
    psi, theta, phi = req.psi, req.theta, req.phi
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)

    r_z = np.array([[c_psi, -s_psi, 0],[s_psi, c_psi, 0],[0, 0, 1]])
    r_y = np.array([[c_theta, 0, s_theta],[0, 1, 0],[-s_theta, 0, c_theta]])
    r_x = np.array([[1, 0, 0],[0, c_phi, -s_phi],[0, s_phi, c_phi]])
    return (r_z @ r_y) @ r_x 

# Normalize all motor angles to be in the range [-pi, pi]
# List[float] -> List[float]
def normalize_motor_angles_pi_minus_pi(motor_angles):
    def normalize(x_rad):
        res = (x_rad % 2*np.pi)
        if res > np.pi:
            res = res - (2*np.pi)
        return res

    return list(map(normalize, motor_angles))

# Normalize all motor angles to be in the range [0, 2pi]
# List[float] -> List[float]
def normalize_motor_angles_zero_twopi(motor_angles):
    def normalize(x_rad):
        return x_rad % (2*np.pi)

    return list(map(normalize, motor_angles))

# Given an angle in radians normalized to the range [-pi, pi], ensure that it is not np.nan and between the servo angle bounds 
def is_valid_angle(angle):
    return not np.isnan(angle) and angle < MAX_SERVO_ANGLE_RADIANS and angle > MIN_SERVO_ANGLE_RADIANS

# Ensure that all motor angles are "valid"
# List[float] -> void
def validate_angles(motor_angles):
    for i in range(len(motor_angles)):
        angle = motor_angles[i]
        if not is_valid_angle(angle):
            raise ValueError(f'Invalid angle {angle} at index {i}')

# normalize motor angles to (-pi, pi), check if all motor angles are valid 
# List[float] -> List[float]
def postprocess_motor_angles(motor_angles):
    # motor_angles = normalize_motor_angles_pi_minus_pi(motor_angles)
    validate_angles(motor_angles)
    return motor_angles

# Computes the angles (in radians, between -pi and pi) for each motor to reach 
# the desired pose by calculating the rotation matrix and the effective length of each leg
# ComputeMotorAngles -> ComputeMotorAnglesResponse
def handle_compute_motor_angles(req):
    rotation_matrix = compute_rotation_matrix(req)
    effective_leg_vec = [compute_effective_leg_vec(i, rotation_matrix, req) for i in range(6)]
    print(f'{effective_leg_vec=}')
    motor_angles = [compute_motor_angle(i, effective_leg_vec[i]) for i in range(6)]
    print(f'{motor_angles=}')
    # TODO: figure out why we still need to update the odd numbered servos despite each servo having a different beta value 
    motor_angles = postprocess_motor_angles(motor_angles)
    print(motor_angles)
    print(f'motor_angles (in degrees): {" ".join(list(map(lambda rad: str(np.degrees(rad)), motor_angles)))}')
    return ComputeMotorAnglesResponse(motor_angles)

def compute_motor_angles_server():
    rospy.init_node('compute_motor_angles_server')
    s = rospy.Service('compute_motor_angles', ComputeMotorAngles, handle_compute_motor_angles)
    print("Ready to compute motor angles (in radians)")
    rospy.spin()

if __name__ == "__main__":
    # TODO: handle these param names better (maybe take them in as command line args)
    # list of lists of base point coordinates
    b = np.array(rospy.get_param("b"))
    # list of lists of platform point coordinates
    p = np.array(rospy.get_param("p"))
    # length of leg
    s = rospy.get_param("s")
    # length of crank arm
    a = rospy.get_param("a")
    # list of angle that motor rotation plane makes with x-axis (in radians)
    beta = rospy.get_param("beta")

    # TODO: compute h_0 and alpha_0 from stewart platform consts
    compute_motor_angles_server()