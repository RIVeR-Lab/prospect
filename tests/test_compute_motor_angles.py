import numpy as np
import unittest
from unittest.mock import patch

from stewart_end_effector.srv import ComputeMotorAngles, ComputeMotorAnglesResponse


s = 103.5
a = 30

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

beta = list(map(lambda deg: float(np.radians(deg)), [180, 0, 300, 120, 60, 240]))

p1 = [-10.5, -46.7, 0]
b = mirror_and_pattern(p1)

p2 = [-7.5, -46.7, 0]
p = mirror_and_pattern(p2)

@patch('compute_motor_angles.a', a) 
@patch('compute_motor_angles.s', s) 
@patch('compute_motor_angles.beta', beta)
@patch('compute_motor_angles.b', b)
@patch('compute_motor_angles.p', p)
class TestComputeMotorAngleMethods(unittest.TestCase):
    def setUp(self):
        pass

    def test_compute_L(self):
        from compute_motor_angles import compute_L
        self.assertAlmostEqual(compute_L(3), -9803.25)
    
    def test_compute_M(self):
        from compute_motor_angles import compute_M
        self.assertAlmostEqual(compute_M(5), 300)
    
    # def test_compute_N(self):
    #     from compute_motor_angles import compute_N
    #     # 2a[cos(beta_i)(x_p - x_b) + sin(beta_i)(y_p - y_b)]
    #     self.assertAlmostEqual(compute_N(1, 10, 5, 20, 10), 60*(5*np.cos(np.radians(60)) + 10 * np.sin(np.radians(60))))

    def test_compute_N2(self):
        from compute_motor_angles import compute_N
        self.assertAlmostEqual(compute_N(0, 10, 10), -600)


    def test_compute_rotation_matrix(self):
        from compute_motor_angles import compute_rotation_matrix
        req = ComputeMotorAngles()
        req.x, req.y, req.z, req.psi, req.theta, req.phi = 0,0,110,0,0,0
        rot = compute_rotation_matrix(req) 
        expected_rot = np.identity(3) 
        np.testing.assert_array_almost_equal(rot, expected_rot)

    def test_compute_effective_leg_vec(self):
        from compute_motor_angles import compute_rotation_matrix, compute_effective_leg_vec
        req = ComputeMotorAngles()
        req.x, req.y, req.z, req.psi, req.theta, req.phi = 0,0,120,0,0,0
        rot = compute_rotation_matrix(req) 
        l1 = compute_effective_leg_vec(0, rot, req)

        expected_l1 = np.array([3, 0, 120])
        np.testing.assert_array_almost_equal(l1, expected_l1)
    
    def test_compute_effective_leg_vec2(self):
        from compute_motor_angles import compute_rotation_matrix, compute_effective_leg_vec
        req = ComputeMotorAngles()
        req.x, req.y, req.z, req.psi, req.theta, req.phi = 10,10,90,0.1,0.2,0.3
        rot = compute_rotation_matrix(req) 
        l1 = compute_effective_leg_vec(0, rot, req)

        expected_l1 = np.array([14.9, 11.32, 77.95])
        np.testing.assert_array_almost_equal(l1, expected_l1)

    # def test_compute_motor_angle(self):
    #     from compute_motor_angles import compute_rotation_matrix, compute_effective_leg_vec, compute_motor_angle
    #     req = ComputeMotorAngles()
    #     req.x, req.y, req.z, req.psi, req.theta, req.phi = 0,0,110,0,0,0
    #     rot = compute_rotation_matrix(req) 
    #     l1 = compute_effective_leg_vec(1, rot, req)
    #     alpha1 = compute_motor_angle(1, l1, req)

    #     from compute_motor_angles import compute_L, compute_M, compute_N
    #     x_b, y_b, z_b = b[1]
    #     x_p, y_p, z_p = req.x, req.y, req.z
    #     L = compute_L(l1)
    #     M = compute_M(z_p, z_b)
    #     N = compute_N(1, x_p, x_b, y_p, y_b)

    #     expected_alpha1 = np.arcsin(L/(np.sqrt(np.square(M) + np.square(N)))) - np.arctan2(N, M)
    #     self.assertAlmostEqual(alpha1, expected_alpha1)
    
    def test_compute_motor_angle(self):
        from compute_motor_angles import compute_rotation_matrix, compute_effective_leg_vec, compute_motor_angle
        req = ComputeMotorAngles()
        req.x, req.y, req.z, req.psi, req.theta, req.phi = 0,0,120,0,0,0
        rot = compute_rotation_matrix(req) 
        l1 = compute_effective_leg_vec(0, rot, req)
        alpha1 = compute_motor_angle(0, l1)

        expected_alpha1 = 0.69 # radians
        print(f"alpha1: {alpha1}")
        self.assertAlmostEqual(alpha1, expected_alpha1)

    def test_compute_motor_angle2(self):
        from compute_motor_angles import compute_rotation_matrix, compute_effective_leg_vec, compute_motor_angle
        req = ComputeMotorAngles()
        req.x, req.y, req.z, req.psi, req.theta, req.phi = 10,10,90,0.1,0.2,0.3
        rot = compute_rotation_matrix(req) 
        l1 = compute_effective_leg_vec(0, rot, req)
        alpha1 = compute_motor_angle(0, l1)

        expected_alpha1 = -0.563 # radians
        self.assertAlmostEqual(alpha1, expected_alpha1)
    
    # def test_handle_compute_motor_angles(self):
    #     from compute_motor_angles import handle_compute_motor_angles, compute_rotation_matrix, compute_effective_leg_vec, compute_motor_angle
    #     req = ComputeMotorAngles()
    #     req.x, req.y, req.z, req.psi, req.theta, req.phi = 0,0,110,0,0,0
    #     rot = compute_rotation_matrix(req) 
    #     l1 = compute_effective_leg_vec(0, rot, req)
    #     alpha1 = compute_motor_angle(0, l1, req)

    #     compute_motor_angles_request = handle_compute_motor_angles(req)
    #     self.assertAlmostEqual(compute_motor_angles_request.motor_angles[0], alpha1)
    

if __name__ == '__main__':
    unittest.main()