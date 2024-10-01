#!/usr/bin/env python3

# -*- coding: utf-8 -*-
import numpy as np
import rospy
import spatialmath as spm

from stewart_end_effector.msg import TOFSensorValues


'''
INPUTS:
---
psi, theta, phi     Starting YPR angles in radians. 
xyz_in              Starting stewart cartesian position.
tof_readings        Distance readings measured by the time of flight sensors.
surface_offset      Desired distance away from the measured surface.
SENSOR_RADIUS       Radius of circumscribed triangle.
SENSOR_CLOCKING     The orientation of the time of flight sensors with respect to the platform.

OUTPUTS: 
---
The SE3 transform between the 
'''

def findSurfaceNormalTransform(psi, theta, phi, xyz_in, tof_readings, surface_offset, SENSOR_RADIUS, SENSOR_CLOCKING):
    ###############################
    # Step 1: Compute End Effector Transformation
    ###############################
    # Transform matrices based on input roll pitch yaw angles and XYZ coordinates:
    #
    #   | R_xyz_in |   | R_ypr_in |
    #   |---------| x |----------|
    #   | t_xyz_in|   | t_ypr_in |
    #
    endEffectorCenter = spm.SE3(xyz_in)*spm.SE3.RPY([phi, theta, psi], order='zyx')

    ###############################
    # Step 2: Determine the ToF Sensor Positions
    ###############################
    # Triangle vertices for ToF sensors:
    #
    #   | v1 |
    #   | v2 |
    #   | v3 |
    #
    vertices = drawTriangle(SENSOR_RADIUS, SENSOR_CLOCKING)    # the ToF sensors are arranged in a triangle. Find these vertices:
    tofsensors  =  endEffectorCenter * spm.SE3(np.hstack((vertices, np.zeros((3,1)))))  
    
    ###############################
    # Step 3: Find Surface Contact Points using ToF Readings
    ###############################
    # Sensed locations:
    #
    #   | s1 |
    #   | s2 |
    #   | s3 |
    #
    sensedlocations = endEffectorCenter * spm.SE3(np.hstack((vertices, np.vstack(tof_readings))))
    sensedpoints = sensedlocations.t
    
    ###############################
    # Step 4: Calculate the Surface Normal
    ###############################
    # Vector normal to sensed points:
    #
    #   | nx |
    #   | ny |
    #   | nz |
    #
    normal_vector = findPlaneNormal(sensedpoints[0], sensedpoints[1], sensedpoints[2])
    
    ###############################
    # Step 5: Find the Centroid of the Sensed Points
    ###############################
    # Mean of sensed points:
    #
    #   | mean_x |
    #   | mean_y |
    #   | mean_z |
    #
    centroid = np.mean(sensedpoints, 0)
    
    ###############################
    # Step 6: Compute the Final Transformation Matrix
    ###############################
    # Basis vectors and transformation:
    #
    #   | bAx |       | bBx |       | nx  |
    #   | bAy |   x   | bBy |   x   | ny  |
    #   | bAz |       | bBz |       | nz  |
    #
    basisXAxis = normalize(sensedpoints[0]-centroid) 
    basisYAxis = np.cross(normal_vector, basisXAxis)
    R = np.transpose(np.vstack((basisXAxis, basisYAxis, normal_vector)))
    targetpoint = spm.SE3.Rt(R, t=centroid)*spm.SE3.Rz(-SENSOR_CLOCKING)
    
    ###############################
    # Step 7: Apply the Surface Offset
    ###############################
    # Final transformation:
    #
    #   | R_target |       | 0  |
    #   |----------|   x   | 0  |
    #   | t_target |       |-off|
    #
    targetTransform = targetpoint * spm.SE3([0, 0, -surface_offset])
    
    return targetTransform

def drawTriangle(R, theta):
    # This function creates the vertices of an equilateral triangle that is circumscribed by a circle 
    # with a given radius "R" and starting angle "theta".

    # Vertex 1:
    # [ R*cos(theta)       ]
    # [ R*sin(theta)       ]
    #
    # Vertex 2:
    # [ R*cos(theta + 2π/3) ]
    # [ R*sin(theta + 2π/3) ]
    #
    # Vertex 3:
    # [ R*cos(theta + 4π/3) ]
    # [ R*sin(theta + 4π/3) ]
    #

    return np.array([[R*np.cos(theta), R*np.sin(theta)], 
                     [R*np.cos(theta+2*np.pi/3), R*np.sin(theta+2*np.pi/3)], 
                     [R*np.cos(theta+4*np.pi/3), R*np.sin(theta+4*np.pi/3)]])

def normalize(vector):
    return vector/np.linalg.norm(vector)

def findPlaneNormal(p1, p2, p3):
    result = np.cross(p2-p1, p3-p1); # calculate the cross product between the two vectors composing the plane
    return normalize(result) # convert result to a unit vector


def test_plane_translation():
    psi, theta, phi = 0,0,0
    xyz_in = [0,0,100]
    tof_readings = [120, 120, 120]
    surface_offset = 100
    trans = findSurfaceNormalTransform(psi, theta, phi, xyz_in, tof_readings, surface_offset, 22.71, np.pi/2)
    print(f'trans: {trans.t}')
    print(f'rot: {trans.rpy()}')

def test_no_move():
    psi, theta, phi = 0,0,0
    xyz_in = [0,0,100]
    tof_readings = [100, 100, 100]
    surface_offset = 100
    trans = findSurfaceNormalTransform(psi, theta, phi, xyz_in, tof_readings, surface_offset, 22.71, np.pi/2)
    print(f'trans: {trans.t}')
    print(f'rot: {trans.rpy()}')

def test_45_degrees_pitch():
    psi, theta, phi = 0,0,0
    xyz_in = [0,0,100]
    tof_readings = [82, 49, 80]
    surface_offset = 50
    trans = findSurfaceNormalTransform(psi, theta, phi, xyz_in, tof_readings, surface_offset, 22.71, np.pi/2)
    print(f'trans: {trans.t}')
    print(f'rot: {trans.rpy()}')

if __name__ == '__main__':
    psi, theta, phi = 0,0,0
    xyz_in = [0,0,100]
    tof_readings = [100, 75, 85]
    surface_offset = 100
    trans = findSurfaceNormalTransform(psi, theta, phi, xyz_in, tof_readings, surface_offset, 22.71, np.pi/2)
    print(f'trans: {trans.t}')
    print(f'rot: {trans.rpy()}')    