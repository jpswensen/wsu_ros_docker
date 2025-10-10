# wsu_ros_docker
A Docker image for WSU robotics classes with ROS 2 + Gazebo, Python, and MATLAB pre-installed.

There is a folder called "shared" that allows easy transfer of files between the host filesystem and the Docker filesystem. In the Docker container, this is mounted at `/home/me485/shared`. Do your schoolwork in this folder so that if something happens to the container, you don't lose your work.

# Installation

Follow these steps to build the image, create the container, and access the desktop inside the container.

Prereqs
- Docker Desktop installed and running
- ~4 CPU / 6–8 GB RAM recommended for Gazebo

  1. Clone the repository
  
  ```
    git clone https://github.com/jpswensen/wsu_ros_docker.git
  ```
  2. Install Docker and/or Docker Desktop (https://www.docker.com/products/docker-desktop/)
  3. Open a terminal window (Terminal.app on macOS, Windows Terminal+PowerShell on Windows, any terminal in Linux)
  4. Navigate to the location where you cloned the repository
  ```
    cd wsu_ros_docker
  ```
  5. Use docker-compose to build the image and create the container
  ```
    docker-compose up
  ```
  6. Wait ~20 minutes for the first build (subsequent starts are faster)
  7. Access the noVNC desktop in a web browser at http://localhost:3000

After the initial creation of the image and container, you can either use the command line in the `wsu_ros_docker` folder or the buttons in Docker Desktop to manage it:
- Start (detached): `docker-compose up -d`
- Stop and remove: `docker-compose down`
- Tail logs: `docker-compose logs -f`


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

## Test that the python wrapper for the Matlab ROS2 floating_camera package is working

This repo includes a MATLAB wrapper that controls the ROS 2 floating camera by calling Python (`rclpy`) from MATLAB via `py.*`. Use this when you don’t have MATLAB’s ROS Toolbox.

What you’ll do
1) Launch the Gazebo world and floating camera
2) Start MATLAB and point it at the ROS-enabled Python
3) Use the `FloatingCamera` MATLAB class to view images and send commands

Step-by-step
1. In a terminal inside the container, build and launch the world (if not already running):
  ```
  cd ~/shared/ros2_camera_ws
  colcon build
  source install/setup.bash
  ros2 launch floating_camera camera_world.launch.py
  ```
  You should see Gazebo with the floating camera and colored spheres.

2. Start MATLAB from a terminal inside the container so it inherits the ROS environment. The first run will prompt for WSU SSO and license selection (see “Current known problems” below for first-run notes). Once MATLAB is open, in the Command Window:
  ```matlab
  % Make sure the MATLAB client is on the path
  addpath('~/shared/MatlabRosClient');

  % Point MATLAB at the system Python where rclpy is available
  pyenv('Version','/usr/bin/python3');

  % Create and start the camera client
  cam = FloatingCamera();
  cam.start();

  % Fetch and display an image (returns [] until the first frame arrives)
  img = cam.getImage();
  cam.show(img);   % or simply: cam.show();

  % Publish a gentle yaw command (Twist: vx,vy,vz, wx,wy,wz)
  cam.publishTwist(0,0,0, 0,0,0.2);

  % Stop when done
  cam.stop();
  ```

3. Prefer an example? Run the provided script:
  ```matlab
  cd ~/shared/MatlabRosClient
  open('FloatingCameraExample.m');
  % Press Run
  ```

Troubleshooting
- If MATLAB cannot import `rclpy`, verify in a terminal: `python3 -c "import rclpy"`. If that works, ensure `pyenv('Version','/usr/bin/python3')` in MATLAB and that MATLAB was launched from a terminal where ROS is sourced.
- If no images appear, confirm the topic exists: `ros2 topic list` should include `/floating_camera/floating_camera/image_raw`.
- Use `cam.isRunning()` to check the client is spinning; call `cam.stop()` before re-running scripts.

## Test that native Matlab ROS2 is communicating correctly with ROS2 and the floating_camera package

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

Notes/workarounds
- Launch MATLAB from a terminal after starting ROS nodes and avoid `clear all` while nodes are active; prefer calling object `stop()` methods first.
- If you need to reset MATLAB’s Python state, try `clear classes; clear all; close all;` and re-run the `pyenv(...)` line.
