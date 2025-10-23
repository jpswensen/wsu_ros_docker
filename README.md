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

## Clipboard between host and guest (noVNC)

Summary
- Guest → Host works: copying in the guest appears in the noVNC Clipboard panel.
- Host → Guest requires an explicit user action in the noVNC Clipboard panel due to browser security; keyboard paste (Ctrl+Shift+V) pastes the remote clipboard, not your host clipboard.

Recommended workflows
1) Quick paste from host to guest via the noVNC Clipboard panel:
   - Open the noVNC toolbar (Ctrl+Alt+Shift) → Clipboard (📋).
   - Click the "Paste from local"/down-arrow button, or click into the panel and press Cmd/Ctrl+V to import your host clipboard.
   - Close the toolbar and press Ctrl+Shift+V in the guest terminal to paste (this pastes the now-updated remote clipboard).

2) Right-click paste inside the guest terminal:
   - Right-click → Paste often succeeds because the browser allows paste on that user gesture.
   - Note: the noVNC Clipboard inspector won’t auto-update to show host clipboard contents unless you import/paste into it.

Browser settings
- Ensure your browser allows clipboard for http://localhost:3000.
  - Chrome/Edge: Lock icon → Site settings → Clipboard → Allow. Reload the page.
  - Firefox: accept the clipboard prompt when shown. If needed, set `dom.events.asyncClipboard` to `true` in `about:config`.

Terminal tips inside the guest
- Use a modern terminal (e.g., `xfce4-terminal`) for standard shortcuts Ctrl+Shift+C/V.
- If selection/middle-click paste works but keyboard paste is inconsistent, start the X clipboard bridge:
  ```
  autocutsel -fork -selection PRIMARY
  autocutsel -fork -selection CLIPBOARD
  ```
  This keeps PRIMARY and CLIPBOARD in sync inside the guest. It doesn’t replace the browser’s need for a user gesture to import host clipboard, but it improves consistency once the remote clipboard is set.

## Test that native Matlab ROS2 is communicating correctly with ROS2 and the floating_camera package

TODO: Not yet working well

1. The first time running Matlab, you have to run as sudo (I don't know why yet). Put in your WSU email and it will kick you to a WSU single-signon
2. Complete the licensing process by logging into your WSU account and picking the Total Academic Headcount license
3. Finish and don't launch
4. Run Matlab from the terminal without sudo


# Current known problems

None


