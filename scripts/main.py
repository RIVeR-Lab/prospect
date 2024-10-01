import rospy
import tf

import numpy as np
import open3d as o3d
import json
from os import path
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN
from stewart_end_effector.srv import StewartControl, Light

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from time import sleep
from copy import deepcopy
import matplotlib.pyplot as plt
from math import radians
import moveit_commander
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from stewart_end_effector.msg import CoordinateData
from spectrometer_drivers.msg import Spectra
from datetime import datetime

from visualization_msgs.msg import MarkerArray, Marker

class ScanPlanner():
    def __init__(self, group_name="manipulator"):
        rospy.init_node("Scan_Planner", anonymous=False)

        # POSE LOOKUP BETWEEN CAM AND ARM
        self.listener = tf.TransformListener()
        self.cam_to_base_tf = self.lookup_transform("depth_camera_link", "base_link")
        trans, rot = self.cam_to_base_tf
        self.cam_to_base = Pose()
        self.cam_to_base.position.x = trans[0]
        self.cam_to_base.position.y = trans[1]
        self.cam_to_base.position.z = trans[2]
        self.cam_to_base.orientation.x = rot[0]
        self.cam_to_base.orientation.y = rot[1]
        self.cam_to_base.orientation.z = rot[2]
        self.cam_to_base.orientation.w = rot[3]
        self.max_operating_dist = .2
        # PATH PLANNING
        self.cloud = None
        self.clean_cloud = None
        # self.spectra = None
        rospy.Subscriber('points2', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/ibsen_vnir/spectral_data', Spectra, self.spectra_callback)
        # rospy.Subscriber('/combined_spectra', Spectra, self.spectra_callback)
        rospy.Subscriber('/tof/coords', CoordinateData, self.distance_callback)
        self.spectral_viewpoint_sychonrization = {
            'Wavelengths': [],
            'scan_pts': [],
            'Spectra': [],
            'Points': [], # XYZ coordinate points
            'Rotations': [], # Rotations (RPY should be fine)
            'Distances': [],
             'idx': [] # Offset distance from the surface as measured by the TOF triad
        }
        # Subscribe to spectral messages
        
        while self.cloud is None:
            print("waiting for first cloud...")
            sleep(1)
        # Create a static transform broadcaster
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        self.robot_commander = moveit_commander.RobotCommander()
        self.scene_interface = moveit_commander.PlanningSceneInterface()
        self.group_commander = moveit_commander.MoveGroupCommander(group_name)
        self.group_commander.set_planner_id("RRTConnect")
        self.group_commander.set_planning_time(1000)
        self.group_commander.set_goal_tolerance(.01)
        # Set the position tolerance in meters
        self.group_commander.set_goal_orientation_tolerance(radians(3)) # Approximately 3 degrees tolerance
        #self.group_commander.set_planner_id("PRM")
        self.planning_frame = self.group_commander.get_planning_frame()
        self.eef_link = self.group_commander.get_end_effector_link()
        self.group_names = self.robot_commander.get_group_names()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.marker_pub = rospy.Publisher("/planning_markers_viz", MarkerArray, queue_size=2)
        rospy.sleep(2)
        self.configure_moveit()
     
        self.stewart_commander = rospy.ServiceProxy('stewart_control', StewartControl)
        self.stewart_lights = rospy.ServiceProxy('/stewart/light_power', Light)
        self.z_home = 90
        self.stewart_commander( 0, 0, self.z_home, 0, 0, 0)

        scan_plan = self.proc_cloud_and_grab_wp(deepcopy(self.cloud))

        self.move_thru_wp(scan_plan['arm_poses'], 
                          scan_plan["stewart_poses"], 
                          scan_plan["stewart_poses_in_camera_frame"], 
                          scan_plan["scanned_pts"])
        
        self.write_spectra_spatial_to_file()

    def configure_moveit(self):
        p = PoseStamped()

        p.header.frame_id = self.planning_frame
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = -.06
        self.scene.add_box("table", p, (2, 2, .1))

        p = PoseStamped()

        p.header.frame_id = self.planning_frame
        p.pose.position.x = .1
        p.pose.position.y = -.15
        p.pose.position.z = 0.0
        self.scene.add_box("camera_stand", p, (.1, .1, 3))

        p = PoseStamped()

        p.header.frame_id = self.eef_link
        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = .07
        self.scene.add_box("stewart_platform", p, (.12, .12, .07))

        self.robot_commander.get_joint("shoulder_lift_joint").min_bound = radians(-170)
        self.robot_commander.get_joint("shoulder_lift_joint").max_bound = radians(-91)
        self.robot_commander.get_joint('elbow_joint').max_bound = radians(0)
        self.robot_commander.get_joint('elbow_joint').min_bound = radians(-170)
        self.robot_commander.get_joint('shoulder_pan_joint').max_bound = radians(90)
        self.robot_commander.get_joint('shoulder_pan_joint').min_bound = radians(-90)
        self.robot_commander.get_joint('wrist_1_joint').min_bound = radians(-230)
        self.robot_commander.get_joint('wrist_1_joint').max_bound = radians(30)
        self.robot_commander.get_joint('wrist_2_joint').min_bound = radians(-5)
        self.robot_commander.get_joint('wrist_2_joint').max_bound = radians(120)

        _ = input("Press enter to asssume up for scanning")
        joint_goal = self.group_commander.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = radians(-90)
        joint_goal[2] = 0
        joint_goal[3] = radians(-90)
        joint_goal[4] = 0
        joint_goal[5] = 0
        self.group_commander.go(joint_goal, wait=True)
        self.group_commander.stop()
        
    def write_spectra_spatial_to_file(self, file_prefix="spectral_spatial", custom_name=""):
        class NumpyEncoder(json.JSONEncoder):
            """Custom encoder for numpy data types."""
            def default(self, obj):
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                if isinstance(obj, np.generic):
                    return obj.item()
                return super().default(obj)
        
        # Format the current date and time as a string: YYYY-MM-DD_HH-MM-SS
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        # Determine the filename
        base_path = path.expanduser('~/prospect/')
        json_filename = f"{file_prefix}_{timestamp}_spectra.json"
        npy_filename = f"{file_prefix}_{timestamp}_spatial.npy"
        
        if custom_name:
            json_filename = f"{custom_name}_spectra.json"
            npy_filename = f"{custom_name}_spatial.npy"
        
        # Write the dictionary to a file in JSON format
        with open(path.join(base_path, json_filename), 'w') as file:
            json.dump(self.spectral_viewpoint_sychonrization, file, indent=4, cls=NumpyEncoder)
        
        # Write point cloud to file
        np.save(path.join(base_path, npy_filename), self.clean_cloud)

        print(f"Data written to {path.join(base_path, json_filename)}")

    # PATH EXEC
    def move_thru_wp(self, wps, stewart_commands, stewart_commands_in_camera_frame, scanned_pts, drop_in_height = .07):
        x = input("press enter to assume scan position")
        joint_goal = self.group_commander.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = radians(-130)
        joint_goal[2] = radians(-37)
        joint_goal[3] = radians(-98)
        joint_goal[4] = radians(82)
        joint_goal[5] = radians(0)

        self.group_commander.go(joint_goal, wait=True)
        self.group_commander.stop()

        # Orient to the first pose
        for idx, wp in enumerate(wps[:1]):
            p = Pose()
            print('================')
            print(wp)
            p.position.x = wp[0][0]
            p.position.y = wp[0][1]
            p.position.z = wp[0][2] + drop_in_height
            p.orientation.x = wp[1][0]
            p.orientation.y = wp[1][1]
            p.orientation.z = wp[1][2]
            p.orientation.w = wp[1][3]
            self.group_commander.set_pose_target(p)
            print("attempting to plan")
            self.group_commander.plan()
            x = input("Press enter to exec plan")
            self.group_commander.go(wait=True)
            self.group_commander.stop()
            self.group_commander.clear_pose_targets()

        # switch to cartesian planning
        for idx, wp in enumerate(wps):
            print(f"waypoint {idx} of {len(wps)}")
            current_pose = self.group_commander.get_current_pose().pose
            above_wp = [[wp[0][0], wp[0][1], wp[0][2] + drop_in_height], *wp[1]]
            sleep(2)

            if np.linalg.norm(wp[0]) < .28:
                print("desired pose too close to arm, continuing!:::")
                continue
            
            if np.linalg.norm(wp[0]) > .65:
                print(" ")
                print("!!!!!desired pose is too far from the arm, continuing!!!!!!")
                print(" ")
                continue

            self.cartesian_plan_to_wp(above_wp, current_pose, wait_for_user=False)
            stewart_trans_command_in_mm = 1000 * np.array(stewart_commands[idx][0])
            stewart_rot = R.from_quat(stewart_commands[idx][1]).as_matrix()
            stewart_rot = R.from_matrix(stewart_rot).as_euler("ZYX")
            stewart_response = self.stewart_commander(*stewart_trans_command_in_mm, stewart_rot[0], stewart_rot[1], stewart_rot[2])
            
            if stewart_response.work is False:
                print("WARNING STEWART POSE NOT REACHED")
                continue
            
            sleep(2)
            frac = self.cartesian_plan_to_wp(wp, current_pose=current_pose, wait_for_user=False)
            ########################################
            ###### SCAN PROCESS STARTS HERE
            if frac >= .99:
                print(f"{frac = }")
                self.do_scan(stewart_commands_in_camera_frame[idx][0], 
                             stewart_commands_in_camera_frame[idx][1], 
                             scanned_pts[idx], idx)

                # scan both at once
                # stewart_response = self.stewart_commander(*stewart_trans_command_in_mm, 0, 0, 0)
                # sleep(2)

                # self.do_scan(stewart_commands_in_camera_frame[idx][0], R.from_matrix(np.eye(3)).as_quat(), scanned_pts[idx])

                self.write_spectra_spatial_to_file(custom_name="MIDDLE")
            else:
                print(f"COULD NOT PLAN TO WP, DID NOT COLLECT SCAN {frac = }")
            ########################################
            # Move to the above drop in position
            self.cartesian_plan_to_wp(above_wp, current_pose, wait_for_user=False)
            # reset stewart platform
            self.stewart_commander( 0, 0, self.z_home, 0, 0, 0)
            sleep(2)
    
        print("Collected entire data !!!!! :))))))")

    def do_scan(self, stew_trans: np.ndarray, stew_rot: np.ndarray, scanned_pt, idx):
        self.stewart_lights('OFF')
        sleep(2)
        # Grab the current distance from the ToF sensors
        platform_d = self.distance
        self.spectral_viewpoint_sychonrization['Distances'].append(platform_d)
        # Turn lights back on
        self.stewart_lights('ON')
        sleep(3)
        # Grab the current spectral signature
        spectra = self.spectra
        self.spectral_viewpoint_sychonrization['Spectra'].append([float(z) for z in spectra])
        # Grab the position
        self.spectral_viewpoint_sychonrization['Points'].append([float(z) for z in stew_trans]) # good 
        self.spectral_viewpoint_sychonrization['Rotations'].append([float(z) for z in stew_rot])
        self.spectral_viewpoint_sychonrization['scan_pts'].append([float(z) for z in scanned_pt])
        self.spectral_viewpoint_sychonrization['idx'].append(idx)
        #print(self.spectral_viewpoint_sychonrization)
        print(f"ADDED POINT TO STATE DICTIONARY, {len(self.spectral_viewpoint_sychonrization['Spectra'])} TOTAL POINTS SCANNED")


    def cartesian_plan_to_wp(self, wp, current_pose, wait_for_user=True):
        p = Pose()
        print(wp)
        p.position.x = wp[0][0]
        p.position.y = wp[0][1]
        p.position.z = wp[0][2]
        p.orientation.x = current_pose.orientation.x
        p.orientation.y = current_pose.orientation.y
        p.orientation.z = current_pose.orientation.z
        p.orientation.w = current_pose.orientation.w

        print("Attempting to plan")
        (plan, frac) = self.group_commander.compute_cartesian_path(
            [p],
            0.005,        # eef_step: resolution of 1 cm
            0.0)         # jump_threshold: disabled
        if wait_for_user:
            x = input("Press enter to exec plan")
        self.group_commander.execute(plan, wait=True)
        self.group_commander.clear_pose_targets()
        return frac


    def proc_cloud_and_grab_wp(self, sample_cloud):
        '''
        Clean up the cloud 
        '''
        sample_cloud = ScanPlanner.crop_point_cloud(sample_cloud,  [-.5, .5], [-.3, 0], [-1, 1])
        sample_cloud = ScanPlanner.remove_tabletop(sample_cloud)
        sample_cloud = ScanPlanner.dbscan_select_largest_cluster(sample_cloud)
        '''
        Visualize the cloud
        '''
        self.publish_marker_array(sample_cloud, "depth_camera_link", [255, 0, 0], start_index=0)
        '''
        Establish the bounding box of the cloud, as well the center/rotation of the bbox
        '''
        min_bbox = ScanPlanner.find_minimal_bbox(sample_cloud)
        cloud_center, cloud_rot = np.array(min_bbox.center), np.array(min_bbox.R)
       
        '''
        Set up constants and stuff
        '''
        upwards_offset = .1
        SCAN_DISTANCE_FROM_ROCK = .08

        STEWART_TCP_TO_STEWART_BASE_OFFSET = .09
        STEWART_BASE_TO_ARM_TCP_OFFSET = .035

        VOXEL_UNIFORM_SUBSAMPLING_SIZE = .06
        upwards_axis_dir = np.array([0, 0, -1]) # relative to cloud rotation
        
        upward_projection_normal = R.from_matrix(cloud_rot).apply(np.array(upwards_axis_dir))
        upward_projection_normal = upward_projection_normal / np.linalg.norm(upward_projection_normal)
        '''
        Estimate normals, voxelize, and define upwards axis
        '''
        upward_projection_normal = R.from_matrix(cloud_rot).apply(np.array(upwards_axis_dir))
        upward_projection_normal = upward_projection_normal / np.linalg.norm(upward_projection_normal)

        top_plane_eq = ScanPlanner.find_plane_equation(upward_projection_normal, (cloud_center + (upward_projection_normal * upwards_offset)))
        self.clean_cloud = sample_cloud 
        sample_cloud_pcd = ScanPlanner.numpy_to_pcd(sample_cloud)
        sample_cloud_pcd = sample_cloud_pcd.voxel_down_sample(voxel_size=VOXEL_UNIFORM_SUBSAMPLING_SIZE)
        sample_cloud_pcd.estimate_normals(fast_normal_computation=False)
        sample_cloud_pcd.orient_normals_consistent_tangent_plane(k=6)
        points = np.array(sample_cloud_pcd.points)
        normals = np.array(sample_cloud_pcd.normals)
        scan_plan = np.array([ScanPlanner.project_point_to_plane_along_vector(point, normal, top_plane_eq) \
                      for (point, normal) in zip(points, normals)])
        correlated_scan_and_top_pts_to_og_pts = [[top_point, normal, og_pt] for (top_point, normal, og_pt) \
                                                in zip(scan_plan[:, 0], scan_plan[:, 1], points)]
        correlated_scan_and_top_pts_to_og_pts = np.array(correlated_scan_and_top_pts_to_og_pts)
        correlated_scan_and_top_pts_to_og_pts = np.array(correlated_scan_and_top_pts_to_og_pts)
        '''
        construct a plan for the arm and stewart platform based off of the processed cloud
        '''
        STEWART_POSES = []
        STEWART_POSES_IN_CAMERA_FRAME = []
        ARM_POSES = []
        SCANNED_PT_LOC = []
        STEWART_TCP_PTS = []
        # reference rotations
        ARM_ROT = R.from_quat([self.cam_to_base.orientation.x, 
                               self.cam_to_base.orientation.y,
                               self.cam_to_base.orientation.z,
                               self.cam_to_base.orientation.w]).as_matrix()
        ARM_ROT = R.from_euler("XZ", [180, 270], degrees=True).apply(ARM_ROT)
        STEWART_BASE_ROT = R.from_euler("Z", -270, degrees=True).apply(ARM_ROT)
        
        def unit_vector(vector):
            """ Returns the unit vector of the vector.  """
            return vector / np.linalg.norm(vector)

        def angle_between(v1, v2):
            v1_u = unit_vector(v1)
            v2_u = unit_vector(v2)
            return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
        
        for idx in range(len(correlated_scan_and_top_pts_to_og_pts)):
            absolute_vec = correlated_scan_and_top_pts_to_og_pts[idx, 1] # normal vector off it
            ang = angle_between(absolute_vec, upwards_axis_dir)

            if ang > .6:
                print(f"Normal vector too extreme, continuing, has angle {ang} relative to unit")
                continue


            if np.linalg.norm(absolute_vec) < .0001:
                print("warn: zero normal vector found")
                continue
            else:
                absolute_vec_normalized = absolute_vec / np.linalg.norm(absolute_vec)
                SCANNED_PT_LOC.append(correlated_scan_and_top_pts_to_og_pts[idx, 2])

            STEWART_TCP_PT = correlated_scan_and_top_pts_to_og_pts[idx, 2] + (absolute_vec_normalized * SCAN_DISTANCE_FROM_ROCK) # where the sensor will be
            STEWART_TCP_PTS.append(STEWART_TCP_PT)

            self.publish_line(correlated_scan_and_top_pts_to_og_pts[idx, 2], STEWART_TCP_PT, "depth_camera_link", [0, 0, 255], line_id=idx, line_width=0.001)
                         
            STEWART_BASE_PT = STEWART_TCP_PT + [0, 0, -STEWART_TCP_TO_STEWART_BASE_OFFSET] #TODO: double check that's ablation
            # STEWART_BASE_PT = STEWART_TCP_PT + (absolute_vec_normalized * STEWART_TCP_TO_STEWART_BASE_OFFSET)
            ARM_TRANS = STEWART_BASE_PT + R.from_matrix(ARM_ROT).apply([0.0, 0.0, -STEWART_BASE_TO_ARM_TCP_OFFSET])
            
            # construct the rotation of the stewart tcp base rot
            STEWART_TCP_ROT = ScanPlanner.construct_rotation_matrix(absolute_vec_normalized * -1)
        
            possible_rots = [R.from_euler("Z", jdx, degrees=True).apply(STEWART_TCP_ROT) for jdx in range(0, 360, 1)]
            possible_rot_rel_angle = [np.dot(STEWART_BASE_ROT[:, 1], rot[:, 1]) / (np.linalg.norm(rot[:, 1]) * np.linalg.norm(STEWART_BASE_ROT[:, 1])) for rot in possible_rots]
            possible_rot_rel_angle = np.arccos(possible_rot_rel_angle)
            min_angle = np.argmin(possible_rot_rel_angle)
            STEWART_TCP_ROT = possible_rots[min_angle] # REPRESENTS: the transform at the tip of the stewart platform aligned but with y axis flipped
        
            self.broadcast_static_transform(STEWART_BASE_PT, R.from_matrix(STEWART_BASE_ROT).as_quat(), "depth_camera_link", f"stewart base {idx}")
            self.broadcast_static_transform(ARM_TRANS, R.from_matrix(ARM_ROT).as_quat(), "depth_camera_link", f"future arm pose {idx}")
            self.broadcast_static_transform(STEWART_TCP_PT, R.from_matrix(STEWART_TCP_ROT).as_quat(), "depth_camera_link", f"stewart tcp {idx}")
            STEWART_BASE_TO_TCP_OFFSET, STEWART_BASE_TO_TCP_ROT = self.lookup_transform(f"stewart base {idx}", f"stewart tcp {idx}")
            self.broadcast_static_transform(STEWART_BASE_TO_TCP_OFFSET, STEWART_BASE_TO_TCP_ROT, f"stewart base {idx}", f"stewart tcp recon {idx}")
            ARM_TRANS_IN_FRAME, ARM_ROT_IN_FRAME = self.lookup_transform(f"base_link", f"future arm pose {idx}")
            STEWART_POSES_IN_CAMERA_FRAME.append((STEWART_TCP_PT, R.from_matrix(STEWART_TCP_ROT).as_quat()))
            STEWART_POSES.append((STEWART_BASE_TO_TCP_OFFSET, STEWART_BASE_TO_TCP_ROT))
            ARM_POSES.append((ARM_TRANS_IN_FRAME, ARM_ROT_IN_FRAME))

        '''
        Visualize the waypoints (arm points nad points on the rock itself)
        '''
        self.publish_marker_array(STEWART_TCP_PTS, "depth_camera_link", [0, 255, 0], start_index=500, radius=.001)
        self.publish_marker_array(correlated_scan_and_top_pts_to_og_pts[:, 2], "depth_camera_link", [0, 0, 255], start_index=1000, radius=.001)

        # plt.show()   

        scan_plan = {"arm_poses": ARM_POSES,
                     "stewart_poses": STEWART_POSES,
                     "stewart_poses_in_camera_frame": STEWART_POSES_IN_CAMERA_FRAME,
                     "scanned_pts": SCANNED_PT_LOC}

        return scan_plan
    
    def broadcast_static_transform(self, translation, quaternion, header_frame, child_frame):

        # Create a TransformStamped message
        transform_stamped = TransformStamped()

        # Fill the header information
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = header_frame
        transform_stamped.child_frame_id = child_frame

        # Set the translation
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]

        # Set the rotation
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        # Broadcast the static transform
        self.broadcaster.sendTransform(transform_stamped)    
     
    def pointcloud_callback(self, msg):
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_array = np.array(list(pc_data))
        self.cloud = pc_array
    
    def spectra_callback(self, msg):
        # Grab the latest spectral measurement and hold onto it for attribution
        self.wavelengths = msg.wavelengths
        self.spectral_viewpoint_sychonrization['Wavelengths'] = self.wavelengths
        self.spectra = list(msg.data)
    
    def distance_callback(self, msg):
        # Extract the distance from the spectrometer
        dist = msg.z
        self.distance = dist[0]

    def lookup_transform(self, target_frame, source_frame):
        # Initialize the ROS Node
  
        try:
            # Wait for the transformation to become available
            self.listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))

            # Get the transformation between the frames at the current time
            transformation = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            return transformation
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to look up transformation.")

    def publish_line(self, point_start, point_end, parent_frame, color, line_id=0, line_width=0.01):
        marker_array = MarkerArray()
        marker = Marker()

        marker.header.frame_id = parent_frame
        marker.header.stamp = rospy.Time.now()

        # set shape, Line strip is 4
        marker.type = Marker.LINE_STRIP
        marker.scale.x = line_width
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0 

        # Set the pose of the marker to be at the origin of the coordinates you're working with
        # For LINE_STRIP, orientation does not affect the line between points
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Add the start and end points of the line
        start_point = Point()
        start_point.x, start_point.y, start_point.z = point_start
        end_point = Point()
        end_point.x, end_point.y, end_point.z = point_end

        marker.points.append(start_point)
        marker.points.append(end_point)

        # Set the marker ID and append it to the MarkerArray
        marker.id = line_id
        marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

    def publish_marker_array(self, coordinates, parent_frame, color, start_index=0, radius=.001):
        marker_array = MarkerArray()

        for i, coord in enumerate(coordinates):
            marker = Marker()

            marker.header.frame_id = parent_frame
            marker.header.stamp = rospy.Time.now()

            # set shape, Arrow: 0; Cube: 1; Sphere: 2; Cylinder: 3
            marker.type = 2  # Sphere

            # Set the scale of the marker
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = radius * 2.0

            # Set the color
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0

            # Set the pose of the marker
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = coord
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.id = start_index + i
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
    
    @staticmethod
    def construct_rotation_matrix(v):
        """Construct a rotation matrix to align Z-axis with the given vector."""
        # Normalize the given vector
        v_normalized = v / np.linalg.norm(v)
        
        # Z-axis vector (0, 0, 1)
        z_axis = np.array([0, 0, 1])
        
        # Rotation axis (cross product of z_axis and v_normalized)
        rotation_axis = np.cross(z_axis, v_normalized)
        
        # Angle of rotation (dot product then arccos to find the angle between z_axis and v_normalized)
        angle = np.arccos(np.dot(z_axis, v_normalized))
        
        # Normalize the rotation axis
        rotation_axis_normalized = rotation_axis / np.linalg.norm(rotation_axis)
        
        # Rodrigues' rotation formula components
        K = np.array([[0, -rotation_axis_normalized[2], rotation_axis_normalized[1]],
                    [rotation_axis_normalized[2], 0, -rotation_axis_normalized[0]],
                    [-rotation_axis_normalized[1], rotation_axis_normalized[0], 0]])
        I = np.eye(3)
        
        # Rotation matrix using Rodrigues' formula
        rotation_matrix = I + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
        
        return rotation_matrix

    # Some useful utils
    @staticmethod
    def rotation_matrix_to_align_vectors(vec1, vec2):
        v1 = np.array(vec1) / np.linalg.norm(vec1)
        v2 = np.array(vec2) / np.linalg.norm(vec2)
        cross_product = np.cross(v1, v2)
        dot_product = np.dot(v1, v2)
        cross_product_matrix = np.array([
            [0, -cross_product[2], cross_product[1]],
            [cross_product[2], 0, -cross_product[0]],
            [-cross_product[1], cross_product[0], 0]
        ])
        rotation_matrix = np.eye(3) + cross_product_matrix + \
                        np.dot(cross_product_matrix, cross_product_matrix) * \
                        (1 / (1 + dot_product))

        return rotation_matrix
        
    @staticmethod
    def numpy_to_pcd(xyz):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    @staticmethod
    def pcd_to_numpy(pcd):
        return np.asarray(pcd.points)

    @staticmethod
    def find_minimal_bbox(cloud: np.ndarray) -> np.ndarray:
        pcd = ScanPlanner.numpy_to_pcd(cloud)
        return pcd.get_minimal_oriented_bounding_box()

    @staticmethod
    def find_plane_equation(normal_vector, point_on_plane):
        A, B, C = normal_vector
        x, y, z = point_on_plane
        D = -(A * x + B * y + C * z)
        return A, B, C, D

    @staticmethod
    def project_points_to_plane(points: np.ndarray, plane: np.ndarray) -> np.ndarray:
        a, b, c, d = plane  
        denominator = a ** 2 + b ** 2 + c ** 2
        projected_points = []
        for point in points:
            dist_along_normal = (a * point[0] + b * point[1] + c * point[2] + d) / denominator
            projected_point = np.array([
                point[0] - a * dist_along_normal,
                point[1] - b * dist_along_normal,
                point[2] - c * dist_along_normal
            ])
            projected_points.append(projected_point)
        return np.array(projected_points)

    @staticmethod
    def project_point_to_plane_along_vector(point, direction_vector, plane_equation):
        # Unpack the inputs
        x0, y0, z0 = point
        vx, vy, vz = direction_vector
        A, B, C, D = plane_equation

        # Solve for t
        # The equation is A(x0 + tvx) + B(y0 + tvy) + C(z0 + tvz) + D = 0
        t = -(A*x0 + B*y0 + C*z0 + D) / (A*vx + B*vy + C*vz)

        # Find the intersection point
        x = x0 + t*vx
        y = y0 + t*vy
        z = z0 + t*vz

        return (x, y, z), direction_vector

    @staticmethod
    def down_sample(pts, voxel_size=0.003):
        p = ScanPlanner.numpy_to_pcd(pts).voxel_down_sample(voxel_size=voxel_size)
        return ScanPlanner.pcd_to_numpy(p)
    
    @staticmethod
    def plane_equation_from_normals(N1, N2):
        N1 = N1 / np.linalg.norm(N1)
        N2 = N2 / np.linalg.norm(N2)
        N = np.cross(N1, N2)
        x0, y0, z0 = 0, 0, 0
        A, B, C = N
        D = -(A * x0 + B * y0 + C * z0)
        return A, B, C, D

    @staticmethod
    def crop_point_cloud(point_cloud, x_bounds, y_bounds, z_bounds):
        x_min, x_max = x_bounds
        y_min, y_max = y_bounds
        z_min, z_max = z_bounds

        # Create boolean masks for points within the specified bounds
        x_mask = (point_cloud[:, 0] >= x_min) & (point_cloud[:, 0] <= x_max)
        y_mask = (point_cloud[:, 1] >= y_min) & (point_cloud[:, 1] <= y_max)
        z_mask = (point_cloud[:, 2] >= z_min) & (point_cloud[:, 2] <= z_max)

        # Combine the masks to get the final mask
        final_mask = x_mask & y_mask & z_mask

        # Use the final mask to get the cropped point cloud
        cropped_point_cloud = point_cloud[final_mask]

        return cropped_point_cloud

    @staticmethod
    def remove_tabletop(cloud: np.ndarray, distance_thresold = 0.02):
        pcd = ScanPlanner.numpy_to_pcd(cloud)
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_thresold,
                                            ransac_n=3,
                                            num_iterations=1000)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        return ScanPlanner.pcd_to_numpy(outlier_cloud)

    @staticmethod
    def dbscan_select_largest_cluster(cloud: np.ndarray, eps = .01, min_samples = 20):
        model = DBSCAN(eps=eps, min_samples=min_samples)
        pred = model.fit_predict(cloud)
        unique_labels = np.unique(model.labels_[model.labels_ != -1])
        clusters = []
        for label in unique_labels:
            indices = np.argwhere(model.labels_ == label).flatten()
            clusters.append(cloud[indices])
            
        cluster_lens = [len(cluster) for cluster in clusters]
        largest_cluster_idx = np.argmax(cluster_lens)
        return clusters[largest_cluster_idx]

    @staticmethod
    def normalize_vector(vector):
        vector = np.array(vector)
        if vector.size == 0:
            raise ValueError("can't normalize: Input vector is empty.")
        magnitude = np.linalg.norm(vector)
        if magnitude == 0:
            raise ValueError("Cannot normalize a zero-length vector.")
        normalized_vector = vector / magnitude
        return normalized_vector


if __name__ == "__main__":
    sp = ScanPlanner()
    rospy.spin()
