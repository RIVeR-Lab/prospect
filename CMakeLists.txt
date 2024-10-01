cmake_minimum_required(VERSION 2.8.3)
project(stewart_end_effector)
add_compile_options(-std=c++14)

# Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  # point_cloud_proc
  rviz_visual_tools
  tf2_eigen
)

# Declare ROS message files
add_message_files(
  FILES
  CoordinateData.msg
  TOFSensorValues.msg 
)

# Declare ROS service files
add_service_files(
  FILES
  ComputeMotorAngles.srv
  StewartControl.srv
  Light.srv
  PlayPen.srv
)

# Find Eigen package
#find_package(Eigen3 REQUIRED)


# Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
)

# Define catkin package details
catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

# Include catkin directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIRS}
)

# Define executables
# add_executable(object_cluster_server src/object_cluster_server.cpp)
# target_link_libraries(object_cluster_server ${catkin_LIBRARIES})

add_executable(eyeball_align src/eyeball_align.cpp)
target_link_libraries(eyeball_align ${catkin_LIBRARIES})

# Install Python scripts
install(DIRECTORY src/stewart_end_effector/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)
