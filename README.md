# stewart_end_effector
Design and software for controlling a servo actuated stewart manipulator platform

Below is a description of and steps to run a Python script which accepts a desired pose of the Stewart platform and actuates six servos to attain that pose. The six servos must be connected via USB to the computer that is running the script, and the servos must each have a distinct ID from 1-6.

### compute_motor_angles.py:
Start a ROS service, /compute_motor_angles, which accepts the args x, y, z, psi, theta, phi (representing a desired pose of the top platform) and returns the corresponding motor angles (in radians) 
### move_to_motor_angles.py: 
A Python script which accepts the args x, y, z, psi, theta, phi (representing a desired pose of the top platform) and moves the Dynamixel servos to the required positions. Depends on the /compute_motor_angles service
### set_params.py: 
A Python script that sets the Stewart Platform parameters (i.e. crank arm length). Must be run before compute_motor_angles.py

## Example run:
1. roscore
1. rosrun set_params.py 
1. rosrun stewart_end_effector compute_motor_angles.py
1. rosrun move_to_motor_angles.py
1. rosservice call /stewart_control 5 20 95 0.1 0.2 0.3

### Troubleshooting:
1. Make sure all Dynamixel servos have unique IDs (from 1-6) through the Dynamixel Wizard ID inspection
1. move_to_motor_angles.py hardcodes information about the hardware. Double check that the information (especially the servo IDs and DEVICENAME) is correct 

### PROSPECT
Running the ur3e arm in conjunction with the kinect camera, and raster movement


ur_robot_driver can be installed from: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

ur3e_moveit_config can be installed from: http://wiki.ros.org/ur3_moveit_config

azure_kinect_ros_driver can be installed from: https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/building.md

Configure host pc to be 192.168.0.86 subnet 255.255.255.0 default gateway 0.0.0.0
Configure robot pc to be 192.168.0.85

In different terminals, run the following commands: 

    roscore

    roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.0.85 kinematics_config:=$(rospack find stewart_end_effector)/config/robot_calibration.yaml

    #run rostest0 on the robot itself

    roslaunch ur3e_moveit_config moveit_planning_execution.launch

    roslaunch ur3e_moveit_config moveit_rviz.launch

    rosrun tf2_ros static_transform_publisher -0.2336 -0.152 0.774 1.57 1.57 -1.57 base_link camera_base

    roslaunch azure_kinect_ros_driver driver.launch overwrite_robot_model:=false rgb_point_cloud:=false depth_mode:=NFOV_UNBINNED 

    rosrun stewart_end_effector tof_triad.py

    roslaunch spectrometer_drivers ibsen.launch

    roslaunch stewart_end_effector master.launch

    python3 scripts/main.py