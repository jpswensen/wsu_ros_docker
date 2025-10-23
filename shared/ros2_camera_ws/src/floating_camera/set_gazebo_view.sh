#!/bin/bash
# Set Gazebo GUI camera position

# Usage: ./set_gazebo_view.sh [x] [y] [z] [roll] [pitch] [yaw]

# Default: Behind the floating camera looking forward
X=${1:--5.0}
Y=${2:-0.0}
Z=${3:-2.0}
ROLL=${4:-0.0}
PITCH=${5:-0.0}
YAW=${6:-0.0}

echo "Setting Gazebo GUI camera to:"
echo "  Position: ($X, $Y, $Z)"
echo "  Orientation: Roll=$ROLL, Pitch=$PITCH, Yaw=$YAW"

# Method 1: Using gz camera command (Gazebo 11)
gz camera -c gzclient_camera -f "$X,$Y,$Z,$ROLL,$PITCH,$YAW"

# If that doesn't work, try publishing to Gazebo topic
# This requires gazebo to be running with the proper transport setup
