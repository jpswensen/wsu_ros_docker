# MATLAB ROS 2 Client (Python interop) — Floating Camera

This repo shows how to control a ROS 2 camera from MATLAB without the ROS Toolbox, by calling the Python ROS 2 libraries (`rclpy`) via MATLAB's `py.*` interop.

Files:
- `FloatingCamera.m` — MATLAB wrapper that loads and controls the Python module.
- `floating_camera_py.py` — Python ROS 2 node and a tiny wrapper class.
- `FloatingCameraExample.m` — Example that moves the camera and displays frames in MATLAB.

## Requirements

- Linux desktop
- ROS 2 Humble (or compatible) installed in `/opt/ros/<distro>` and Python packages available to your chosen Python
  - `rclpy` (from ROS 2)
- Python 3.10 (what this machine currently uses). Ensure your MATLAB version supports this Python.

Quick sanity checks in a terminal:

```bash
python3 -c "import rclpy; print('rclpy OK')"
ros2 topic list
```

If any import fails, make sure the ROS environment is set (often by sourcing `/opt/ros/humble/setup.bash`).

## How it works

`FloatingCamera.m` will try to import the bundled `floating_camera_py.py` directly. As a fallback, it can write a copy to a temp folder if needed.

The Python node:
- Subscribes to `/floating_camera/floating_camera/image_raw` (sensor_msgs/Image)
- Subscribes to `/floating_camera/pose` (geometry_msgs/PoseStamped)
- Publishes `/floating_camera/cmd_vel` (geometry_msgs/Twist)
  (No OpenCV UI; images are fetched to MATLAB for plotting.)

## Run from MATLAB

1) Point MATLAB at the same Python used above (adjust the path as needed):

```matlab
pyenv('Version','/usr/bin/python3');   % Verify with pyenv to see it took effect
```

2) In MATLAB, run:

```matlab
cam = FloatingCamera();
cam.start();
% Optional: send a small yaw command manually
% cam.publishTwist(0,0,0, 0,0,0.4);
```

You should see logs printed in MATLAB. Use `cam.show()` to display frames. To stop:

```matlab
cam.stop();
```

The object destructor also tries to stop on `clear demo` or when it goes out of scope.

## Feeding topics

This demo expects topics with these default names:
- Image: `/floating_camera/floating_camera/image_raw` (sensor_msgs/Image)
- Pose: `/floating_camera/pose` (geometry_msgs/PoseStamped)

If your producer publishes different names, either:
- Remap (launch) the Python node with remaps, or
- Edit the topic strings in `floating_camera_py.py` to match your environment.

You can explore available topics with:

```bash
ros2 topic list
ros2 topic echo /your/image/topic
```

## Troubleshooting

- MATLAB cannot import `rclpy` or `cv_bridge`:
  - Ensure MATLAB `pyenv` points to a Python where these import successfully in a terminal.
  - If you typically `source /opt/ros/humble/setup.bash`, start MATLAB from a terminal after sourcing so it inherits environment vars (PYTHONPATH, LD_LIBRARY_PATH, etc.).

- Node runs but no images:
  - Verify the image topic exists and the message type is `sensor_msgs/msg/Image`.
  - Adjust the topic names in the Python file to match.

- Clean shutdown:
  - Call `demo.stop()` before re-running. If MATLAB/Python gets into an initialized state, restart MATLAB as a last resort.

## Notes

- The executor loop uses `SingleThreadedExecutor.spin_once()` in a Python thread so stop() responds quickly.
- You can publish manual twists from MATLAB with `publishTwist(lx,ly,lz, ax,ay,az)`.
- See `FloatingCameraExample.m` for a simple motion demonstration.