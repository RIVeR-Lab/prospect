import rospy 
import moveit_commander
import ros_numpy
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from point_cloud_proc.srv import TabletopClustering
from stewart_end_effector.srv import StewartControl
from move_to_motor_angles import MoveMotorAngles as StewartAngleCommander

from typing import Union, List

class RasterPlanGenerator():
    """
    Authored By Gary Lvov
    """
    def __init__(self, group_name: str = "manipulator"):
        rospy.init_node("Raster_Plan_Generator", anonymous=False)

        ### MoveIt ###

        use_moveit=False

        if use_moveit:
            self.robot_commander = moveit_commander.RobotCommander()
            self.scene_interface = moveit_commander.PlanningSceneInterface()
            self.group_commander = moveit_commander.MoveGroupCommander(group_name)
            self.planning_frame = self.group_commander.get_planning_frame()
            self.eef_link = self.group_commander.get_end_effector_link()
            self.group_names = self.robot_commander.get_group_names()
            self.robot_commander.get_joint('shoulder_lift_joint').max_bound = 0
            self.robot_commander.get_joint('shoulder_lift_joint').min_bound = -1.685

            print(f"{self.planning_frame = } {self.eef_link = } {self.group_names = }")
            rospy.logdebug("RasterPlanGenerator has succesfully initialized moveit.")
        
        ### Topics and srvs ###
        # self.trajectory_viz_pub = rospy.Publisher('/move_group/display_path_coords', Marker, queue_size = 10)
        rospy.wait_for_service('/object_clustering/cluster_objects')
        self.cluster_objects_srv = rospy.ServiceProxy('/object_clustering/cluster_objects', TabletopClustering)
        # rospy.wait_for_service('/stewart_control')
        #self.stewart_control_srv = rospy.ServiceProxy("/stewart_control")
        rospy.logdebug("Succesfully initialized ROS interface")

        #self.scan_surface_srv = rospy.Service('/scan_surface', self.scan_surface)
        print("About to grab cloud")
        self.grab_cloud()

    def scan_surface(self):
        self.intialize_arm_for_raster()
        cloud = self.grab_cloud()
        arm_waypoints = self.generate_arm_waypoints(cloud)
        arm_trajectory = self.send_arm_through_waypoints(arm_waypoints)
    
    def send_pose_to_stewart_arm(self, translation: Union[np.ndarray, List] = [0, 0, 0], 
                                 rotation: Union[np.ndarray, List] = np.eye(3)) -> bool:
        compute_angles_req = StewartControl()
        compute_angles_req.x = translation[0]
        compute_angles_req.y = translation[1]
        compute_angles_req.z = translation[2]

        compute_angles_req.psi = rotation[0]
        compute_angles_req.theta = rotation[1]
        compute_angles_req.phi = rotation[2]
        resp = self.stewart_control_srv(compute_angles_req)
        return resp

    def intialize_arm_for_raster(self, 
                                 joint_angles: Union[np.ndarray, List[float]] = [-2.117, -1.685,-1.742, -1.273,  1.641, -1.550]):
        joint_goal =  self.group_commander.get_current_joint_values()
        for i in range(len(joint_angles)):
            joint_goal[i] = joint_angles[i]
        self.group_commander.go(joint_goal, wait=True)
        self.group_commander.stop()

    def grab_cloud(self, cluster_idx:int = 0) -> np.ndarray:
        clustered_objects = self.cluster_objects_srv()
        ros_cloud = clustered_objects[cluster_idx].cloud
        xyz_cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(ros_cloud)
        numpy_cloud = np.array([[point.x, point.y, point.z] for point in xyz_cloud])
        print('saving cloud')
        np.save("sample_cloud_1.npy", numpy_cloud)
        print('saved cloud')
        return numpy_cloud
    
    def send_arm_through_waypoints(self, waypoints: np.ndarray, wait: bool = True) -> np.ndarray:
        # cartesian path interpolated at resolution of 1 cm, thus 0.01 in the eef_step in Cartesian translation
        (plan, fraction) = self.group_commander.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group_commander.execute(plan, wait=wait)

    def visualize_waypoints(self, waypoints: List[Point]):
        marker = Marker()
        
        marker.header.frame_id = '/base_link'
        marker.header.stamp = rospy.Time.now()

        marker.type = Marker.SPHERE_LIST
        marker.id = 0

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        # Convert pose objects to point objects
        marker.points = []
        for idx, pose in enumerate(waypoints):
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y 
            point.z = pose.position.z
            marker.points.append(point)

            # assign a unique id to each marker
            marker.id = idx
            
        self.trajectory_viz_pub.publish(marker)

if __name__ == "__main__":
    rpg = RasterPlanGenerator()
    rospy.spin()