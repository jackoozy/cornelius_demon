cmake_minimum_required(VERSION 3.0.2)
project(selfie_drawing_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
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
  pilz_industrial_motion_planner
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OpenCV REQUIRED)

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
  ${OpenCV_INCLUDE_DIRS}
)

# Set the Python interpreter to use your conda environment
set(PYTHON_EXECUTABLE /home/lucas/cornEnv/bin/python)

# Motion planning
add_executable(test_script src/test_script.cpp)
add_executable(operations src/operations.cpp src/Robot.cpp)
add_executable(current_program src/current_program.cpp)

target_link_libraries(test_script
  ${catkin_LIBRARIES}
)

target_link_libraries(operations
  ${catkin_LIBRARIES}
)

target_link_libraries(current_program
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
)

# Line detection
add_executable(test_line_detection src/test_line_detection.cpp src/line_detection.cpp)
target_link_libraries(test_line_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Install Python scripts
catkin_install_python(PROGRAMS
  src/userInterface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
