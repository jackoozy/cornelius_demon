#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/cornelius_ws/devel/setup.bash
rosrun selfie_drawing_robot test_line_detection camera
