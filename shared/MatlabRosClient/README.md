# MATLAB ROS 2 Client (Python interop) — Floating Camera

Control a ROS 2 camera from MATLAB without the ROS Toolbox by calling the Python ROS 2 libraries (`rclpy`) via MATLAB's `py.*` interop. Images are brought into MATLAB for display; no OpenCV UI is used. This is designed for student projects and teaching labs.

Contents
- `FloatingCamera.m` — MATLAB wrapper that loads/controls the Python node and exposes a simple API.
- `floating_camera_py.py` — Python ROS 2 node and a small wrapper class.
- `FloatingCameraExample.m` — Example using a capture-first control loop and MATLAB plotting.

## Requirements

- Linux desktop
- ROS 2 Humble (or compatible) installed and sourced (e.g., `/opt/ros/humble`)
  - Python packages: `rclpy`, `numpy` available to your selected Python
- Python 3.10 (ensure your MATLAB supports this version via `pyenv`)
- MATLAB with image display capability (uses `imshow`)

Quick sanity checks in a terminal:

```bash
python3 -c "import rclpy, numpy as np; print('rclpy OK')"
ros2 topic list
```

If imports fail, source ROS before launching MATLAB: `source /opt/ros/humble/setup.bash`.

## How it works

`FloatingCamera.m` imports the bundled `floating_camera_py.py` (or writes a copy to a temp dir if needed), then spins a `SingleThreadedExecutor` in a Python thread. The Python node:
- Subscribes to `/floating_camera/floating_camera/image_raw` (sensor_msgs/Image)
- Subscribes to `/floating_camera/pose` (geometry_msgs/PoseStamped)
- Publishes `/floating_camera/cmd_vel` (geometry_msgs/Twist)

Images are decoded in Python using NumPy and returned to MATLAB as raw RGB bytes, which MATLAB reshapes for display.

## Setup in MATLAB

1) Point MATLAB to your ROS-enabled Python (adjust path to match your system):

```matlab
pyenv('Version','/usr/bin/python3');
```

2) Start the client and node:

```matlab
cam = FloatingCamera();
cam.start();
```

3) Show a frame and publish a small command (optional):

```matlab
img = cam.getImage();    % [] if no frame yet
cam.show(img);           % or cam.show(); to fetch internally
cam.publishTwist(0,0,0, 0,0,0.2);
```

4) Stop when done:

```matlab
cam.stop();
```

Tip: The object attempts to stop in its destructor, but it's good practice to call `cam.stop()` explicitly.

## For students: capture-first control loop

Most projects should follow this pattern each cycle:

```matlab
% 1) Read the current pose (optional if not needed)
pose = cam.getPose();

% 2) Read the current image
img = cam.getImage();

% 3) Compute control from pose/image and send it
%    Example: gentle sinusoidal yaw rate based on time t
yaw_rate = 0.5 * sin(2*pi*0.1*t);
cam.publishTwist(0,0,0, 0,0,yaw_rate);

% 4) Visualize the image (pass img for consistency; show() also fetches if empty)
cam.show(img);
```

See `FloatingCameraExample.m` for a complete loop with timing.

## API overview

- `start()` / `stop()` — Start/stop the Python ROS 2 node and executor thread.
- `isRunning()` — True when the Python wrapper indicates it’s spinning.
- `getImage()` — Returns the latest image as HxWx3 uint8 (RGB) or `[]` if none yet.
- `getPose()` — Returns a struct with fields:
  - `position.x,y,z`
  - `orientation.x,y,z,w`
  - `stamp.sec, stamp.nanosec`
  - `frame_id`
  Returns `[]` if no pose yet.
- `publishTwist(lx,ly,lz, ax,ay,az)` — Publish geometry_msgs/Twist.
- `show([img])` — Display provided image, or fetch latest if omitted/empty. Uses handle reuse for speed.

## Topics and remapping

Default topics the Python node uses:
- Image: `/floating_camera/floating_camera/image_raw` (sensor_msgs/Image)
- Pose: `/floating_camera/pose` (geometry_msgs/PoseStamped)
- CmdVel: `/floating_camera/cmd_vel` (geometry_msgs/Twist)

If your system uses different names:
- Change the topic strings in `floating_camera_py.py`, or
- Remap topics in a ROS 2 launch setup that starts your publishers.

Helpful ROS 2 commands:

```bash
ros2 topic list
ros2 topic echo /your/image/topic
ros2 topic info /your/image/topic
```

## Troubleshooting

- MATLAB cannot import `rclpy`:
  - Ensure `pyenv` points to a Python where `import rclpy` succeeds in a terminal.
  - Launch MATLAB from a terminal after sourcing ROS so environment variables carry over (PYTHONPATH, LD_LIBRARY_PATH, etc.).

- No images appear in MATLAB:
  - Confirm the image topic exists and is publishing `sensor_msgs/Image`.
  - Verify encodings are standard (`rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`).
  - Ensure the node is running (`cam.isRunning()` should be true).

- Method not found after edits (e.g., `get_pose`):
  - The MATLAB constructor reloads the Python module, but stale MATLAB state can linger.
  - Run `clear classes; clear all; close all;` in MATLAB and try again.

- Clean shutdown / re-run loops:
  - Call `cam.stop()` before re-running examples.

## Notes and limitations

- This approach uses pure NumPy in Python for image unpacking; no `cv_bridge` or OpenCV UI is required.
- The plotting uses MATLAB handle reuse and `drawnow limitrate` for speed.
- Topic names and QoS are currently fixed in the Python file; make edits there if needed.

---

If you’re adapting this for a class or lab, point students to the capture-first loop above and `FloatingCameraExample.m`. That example is intentionally minimal and mirrors the common “sense → plan → act → visualize” workflow they’ll implement in projects.