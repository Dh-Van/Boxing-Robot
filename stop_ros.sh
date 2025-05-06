#!/bin/bash

echo "Attempting graceful shutdown of ROS nodes..."
rosnode kill -a

# Use sudo with pkill -f to target specific processes by command line
# Target the camera node explicitly if you know its launch command pattern
sudo pkill -f usb_cam # Or the specific launch file/node name pattern

# Target core ROS processes
sudo pkill -f rosmaster
sudo pkill -f rosout
sudo pkill -f roscore

# Target roslaunch if it was used
sudo pkill -f roslaunch

echo "ROS shutdown attempt complete."
echo "Please check if processes are still running (e.g., using 'ps aux | grep ros')."