#!/bin/bash

# Define the port number
PORT=65432

# Function to free the port
free_port() {
  echo "Checking for processes using port $PORT..."
  PIDS=$(sudo lsof -t -i:$PORT)
  if [ ! -z "$PIDS" ]; then
    echo "Killing processes using port $PORT: $PIDS"
    sudo kill -9 $PIDS
  else
    echo "No processes were using port $PORT"
  fi
}

# Free the port before starting any processes
free_port

# Start roscore
roscore &
ROSCORE_PID=$!

# Give roscore some time to start
sleep 2

# Activate the first virtual environment and run userInterface.py, needs python 3.12.2
source ~/cornEnv/bin/activate
./userInterface.py &
USER_INTERFACE_PID=$!

# Activate the second virtual environment and run ros_node.py, needs python 3.8.10
source ~/cornEnvStable/bin/activate
./ros_node.py &
ROS_NODE_PID=$!

# Function to clean up background processes and free the port
cleanup() {
  echo "Stopping background processes..."
  kill $USER_INTERFACE_PID
  kill $ROS_NODE_PID
  kill $ROSCORE_PID

  # Free the port
  free_port
}

# Set the cleanup function to be called on script exit
trap cleanup EXIT

# Wait for both scripts to complete
wait $USER_INTERFACE_PID
wait $ROS_NODE_PID

# Explicitly exit the script, triggering the cleanup function
exit 0
