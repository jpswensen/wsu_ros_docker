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

After the initial creation of the image and container, you can either use the command line with the docker-compose command with the up or down subcommand in the wsu_ros_docker folder to bring the container up and down, or you can use the stop/play buttons in Docker Desktop.


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

1. The first time running Matlab, you have to run as sudo (I don't know why yet). Put in your WSU email and it will kick you to a WSU single-signon
2. Complete the licensing process by logging into your WSU account and picking the Total Academic Headcount license
3. Finish and don't launch
4. Run Matlab from the terminal without sudo


# Current known problems

1. For some reason, there is a weird interaction between Matlab and ROS2. If I start the talker node in the terminal first, then the listener in Matlab can kill the nodes run from the terminal. 
  * Open matlab from a terminal
  * Launch the demo talker node
    ```
    ros2 run demo_nodes_cpp talker
    ```
  * Run the MatlabListener script from Matlab
  * The output of the demo talker script looks like
  ```
  [INFO] [1760036296.573883920] [talker]: Publishing: 'Hello World: 1'
  [INFO] [1760036297.573633443] [talker]: Publishing: 'Hello World: 2'
  [INFO] [1760036298.573870925] [talker]: Publishing: 'Hello World: 3'
  [INFO] [1760036299.573848074] [talker]: Publishing: 'Hello World: 4'
  [INFO] [1760036300.573673975] [talker]: Publishing: 'Hello World: 5'
  [INFO] [1760036301.573608513] [talker]: Publishing: 'Hello World: 6'
  [INFO] [1760036302.573712534] [talker]: Publishing: 'Hello World: 7'
  [INFO] [1760036303.573784313] [talker]: Publishing: 'Hello World: 8'
  [INFO] [1760036304.573691314] [talker]: Publishing: 'Hello World: 9'
  [ros2run]: Killed
  ```
  * There is arguably no possible way that the Matlab script could cause the other node to crash unless it was doing something with ROS2 running under the hood.
2. If I run the MatlabListener script first, and then run the demo talker node, it seems to be working correctly at first glance. However, if I terminate the script in Matlab and then issue a "clear all" command, once again the demo talker node process running in the terminal is killed.    
3. If I use the launch file floating_camera packages to launch Gazebo and the floating_camera node, then open Matlab and run 'ros2 topic list' from Matlab, it first prints out the list of topics, but then both Matlab and ROS2+Gazebo crashes in the terminal
  * Open a terminal, cd to the floating_camera workspace, source it, and launch the camera and world in gazebo
  ```
  cd ~/shared/ros2_camera_ws
  colcon build
  source install/setup.bash
  ros2 launch floating_camera camera_world.launch.py
  ```
  * Open Matlab and run the command
  ```
  ros2 topic list
  ```
  * Quickly observe the topics printout out correctly in Matlab before the crash occurs.
