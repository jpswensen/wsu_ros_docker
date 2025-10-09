This file is intended to help with some fixups to ROS2:

1. There is a problem when starting the turtlebot3 simulations with RViz and SLAM that the SLAM algorithm starts running before the laser range finder is giving sensor readings. This causes a "phantom obstacle" to show up right at the center of the robot and makes it hard to start navigating.

    Solution: Copy the bringup_launch.py to /opt/ros/humble/share/nav2_bringup/launch/
    You will need to do this as sudo, as that is a read-only folder for the default user.
