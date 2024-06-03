#!/bin/bash

# Start roscore
roscore &
ROSCORE_PID=$!

# Give roscore some time to start
sleep 2

# Activate the first virtual environment and run userInterface.py
source ~/cornEnv/bin/activate
./userInterface.py &
USER_INTERFACE_PID=$!

# Activate the second virtual environment and run ros_node.py
source ~/cornEnvStable/bin/activate
./ros_node.py &
ROS_NODE_PID=$!

# Function to clean up background processes
cleanup() {
  echo "Stopping background processes..."
  kill $USER_INTERFACE_PID
  kill $ROS_NODE_PID
  kill $ROSCORE_PID
}

# Set the cleanup function to be called on script exit
trap cleanup EXIT

# Wait for both scripts to complete
wait $USER_INTERFACE_PID
wait $ROS_NODE_PID
