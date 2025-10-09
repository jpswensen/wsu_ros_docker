# wsu_ros_docker
A docker image for wsu robotics classes that has ROS+Gazebo, Python, and Matlab pre-installed

There is a folder called "shared" that allows easy transfer of files between the host filesystem and the docker filesystem. In the docker container, this is located at /home/me485/shared. I recommend doing your schoolwork in this folder so that if something accidentally happens to the docker container, you don't lose your work.

# Installation

Follow these steps to get the docker image created, the container created and running, and 

  1. Clone the repository
    ```
    git clone https://github.com/jpswensen/wsu_ros_docker.git
    ```
  2. Install Docker and/or Docker Desktop (https://www.docker.com/products/docker-desktop/)
  3. Open a terminal windows (Terminal.app on MacOS, Windows Terminal+PowerShell recommended for Windows, any terminal in Linux)
  4. Navigate to the location where you cloned the repository
    ```
    cd wsu_ros_docker
    ```
  5. Use docker-compose to create the docker image and a docker container based on the image
    ```
    docker-compose up
    ```
  6. Wait 20-ish minutes for the image to be created (future containers based on this image won't take this long)
  7. Access the novnc remote desktop via a web browser at http://localhost:3000

# Usage

## Test ROS and ROS+Gazebo with floating_camera package

To do a simple test of whether ROS is working correctly, open two terminals and execute each of the following commands in the different terminals

```
ros2 run demo_nodes_cpp talker
```

```
ros2 run demo_nodes_cpp listener
```

To verify the custom floating_camera package is working, we first need to build and source the workspace with this package and then launch

```
cd ~/shared/ros2_camera_ws
colcon build
source install/setup.bash
ros2 launch floating_camera camera_world.launch.py
```

You should see Gazebo launch with a camera and sixteen spheres of various colors in the field of view of the camera. To verify it launched correctly, list the topics that the new package provides:

```
ros2 topic list
```

You should see a variety of topics associated with the floating_camera for subscribing to images, pose, and velocity, and a topic for commanding the camera velocity.

## Test that Python ROS2 packages are correctly installed. 

Change to the ros2_python_test folder in the shared folder and run a custom listener script in Python. If the demo talker node is not still running, please follow the instructions above to start it in its own terminal window.

```
cd ~/shared/ros2_python_test

python3 custom_listener.py
```

Observe that the custom listener is receiving messages on the /chatter topic and printing them out.

## Test that Matlab ROS2 is communicating correctly with ROS2 and the floating_camera package

TODO: Not yet working well

# Current known problems

1. For some reason, there is a weird interaction between Matlab and ROS2. If I start the listener in Matlab before I start the publisher in the terminal, they aren't connected. However, if I do it the other way around it works. After they have achieved connection the first time, then subsequent order of starting doesn't matter.

2. None of the topics from the floating_camera package are showing up in Matlab.
