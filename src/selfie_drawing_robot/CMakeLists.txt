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
    std_srvs
    pilz_industrial_motion_planner
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(rostest REQUIRED)


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
include_directories(${GTEST_INCLUDE_DIRS})


# Motion planning
add_library(operations src/operations.cpp)

add_executable(current_program src/current_program.cpp)

target_link_libraries(operations
  ${catkin_LIBRARIES})
target_link_libraries(current_program
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  operations)

catkin_add_gtest(testMP testing/test.cpp)
target_link_libraries(testMP ${catkin_LIBRARIES} ${GTEST_LIBRARIES} operations)


# Line detection

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(test_line_detection src/test_line_detection.cpp src/line_detection.cpp)
target_link_libraries(test_line_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

