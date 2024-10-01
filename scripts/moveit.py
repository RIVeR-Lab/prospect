#!/usr/bin/python2
import copy
import rospy
import ros_numpy
import moveit_commander
from stewart_end_effector.srv import PlayPen
from point_cloud_proc.srv import TabletopClustering
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import pi

#robot commander object initializiation
robot = moveit_commander.RobotCommander()

#planningsceneobject initiliazation
scene = moveit_commander.PlanningSceneInterface()

#MoveGroupCommander
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

#DisplayTrajectory publisher (for RViz visualition)
pub = rospy.Publisher('/move_group/display_path_coords', Marker, queue_size = 10)

#Getting Basic info

#name of reference frame
planning_frame = group.get_planning_frame()
print("================== Reference frame: %s" % planning_frame)

#name of end effector link for group
eef_link = group.get_end_effector_link()
print("================== End effector: %s" % eef_link)

#List of all gorups in the robot
group_names = robot.get_group_names()
print("================== Robot Groups:", robot.get_group_names())

#Robot state
print("================== Printing robot state")
print(robot.get_current_state())
print("")

def raster_movement(msg):
    #-2.683
    #-1.685

    robot.get_joint('shoulder_lift_joint').max_bound = 0
    robot.get_joint('shoulder_lift_joint').min_bound = -1.685

    #join vals/movement 
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -2.117
    joint_goal[1] = -1.685
    joint_goal[2] = -1.742
    joint_goal[3] = -1.273
    joint_goal[4] = 1.641
    joint_goal[5] = -1.550

    group.go(joint_goal, wait=True)
    group.stop()

    cluster_objects_srv = rospy.ServiceProxy('/object_clustering/cluster_objects', TabletopClustering)
    cluster_objects = cluster_objects_srv()
    point_cloud = cluster_objects.objects[0].cloud
    
    pt_x = []
    pt_y = []
    pt_z = []

    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud)
    for point in xyz_array:
        pt_x.append(point[0])
        pt_y.append(point[1])
        pt_z.append(point[2])

    waypoints = []

    wpose = group.get_current_pose().pose
    x_rot = wpose.orientation.x
    y_rot = wpose.orientation.y
    z_rot = wpose.orientation.z
    w_rot = wpose.orientation.w

    x_coords = [min(pt_x), max(pt_x)]
    y_coords = [min(pt_y), max(pt_y)]
    z = max(pt_z) + 0.015 + 0.13335 
    reverse = False
    
    for y in [val/100.0 for val in range(int(y_coords[0]*100), int(y_coords[1]*100), 1)]:
        if reverse:
            x_start = x_coords[1]
            x_end = x_coords[0]
        else:
            x_start = x_coords[0]
            x_end = x_coords[1]

        for x in [val/100.0 for val in range(int(x_start*100), int(x_end*100), 1)]:
            wpose.position.x = x
            wpose.position.y = y
            wpose.position.z = z
            wpose.orientation.x = x_rot
            wpose.orientation.y = y_rot
            wpose.orientation.z = z_rot
            wpose.orientation.w = w_rot
            waypoints.append(copy.deepcopy(wpose))
        
        reverse = not reverse

    #return to start
    wpose.position.x = 0.28
    wpose.position.y = 0.2
    wpose.position.z = z
    waypoints.append(copy.deepcopy(wpose))

    # cartesian path interpolated at resolution of 1 cm, thus 0.01 in the eef_step in Cartesian translation
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

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
        
    pub.publish(marker)

    #Execute
    group.execute(plan, wait=True)

    marker.action = Marker.DELETEALL
    pub.publish(marker)

    return True

if __name__ == "__main__":
    #initialize moveit_commander and rospy node
    rospy.init_node('moveit', anonymous = True)

    # Register a service with the callback to compute_motor_angles
    play_pen_control = rospy.Service("/ur3e/rect_coords", PlayPen, raster_movement)
    rospy.spin()
