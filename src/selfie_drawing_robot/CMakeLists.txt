set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


cmake_minimum_required(VERSION 3.0.2)
project(selfie_drawing_robot)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roscpp
    rospy
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# Motion planning
add_executable(test_script src/test_script.cpp)
add_executable(operations src/operations.cpp src/Robot.cpp)
# catkin_install_python(PROGRAMS
#   src/motion_planning_tutorial.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

add_executable(motion_planning_tutorial src/motion_planning_tutorial.cpp)

target_link_libraries(test_script
  ${catkin_LIBRARIES})
target_link_libraries(operations
  ${catkin_LIBRARIES})
target_link_libraries(motion_planning_tutorial
  ${catkin_LIBRARIES})


# Line detection

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(test_line_detection src/test_line_detection.cpp src/line_detection.cpp)
target_link_libraries(test_line_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

