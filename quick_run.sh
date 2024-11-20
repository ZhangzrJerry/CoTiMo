#!/bin/bash

cd /ws

# Build the workspace
catkin_make

# Source the setup file
source devel/setup.bash

# Check if the path_planning package exists
if [ -d "src/path_planning" ]; then
  # Check if the demo.launch file exists
  if [ -f "src/path_planning/launch/demo.launch" ]; then
    # Launch the demo
    roslaunch path_planning demo.launch
  else
    echo "Error: demo.launch file not found in path_planning package."
  fi
else
  echo "Error: path_planning package not found in the workspace."
fi