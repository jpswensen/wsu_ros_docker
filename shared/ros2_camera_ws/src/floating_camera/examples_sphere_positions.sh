#!/bin/bash
# Example launch commands for floating_camera with sphere_positions parameter

# 1. Launch with default explicit positions (12 corner points from image)
echo "=== Launch with default explicit positions (random_spheres=False by default) ==="
echo "ros2 launch floating_camera camera_world.launch.py"
echo ""

# 2. Launch with random grid layout
echo "=== Launch with random grid layout ==="
echo 'ros2 launch floating_camera camera_world.launch.py random_spheres:=True'
echo ""

# 3. Launch with custom explicit positions
echo "=== Launch with custom explicit positions ==="
echo 'ros2 launch floating_camera camera_world.launch.py sphere_positions:="[[1,0,5],[2,0,5],[3,0,5],[4,0,5]]"'
echo ""

# 4. Launch with random grid + custom number of spheres
echo "=== Launch with random grid + custom number of spheres ==="
echo 'ros2 launch floating_camera camera_world.launch.py random_spheres:=True num_spheres:=25'
echo ""

# 5. Combined with larger spheres
echo "=== Launch with default positions + larger spheres ==="
echo 'ros2 launch floating_camera camera_world.launch.py sphere_radius:=0.1'
echo ""

# 6. Custom camera position (moved back to view scene better)
echo "=== Launch with camera moved back to z=5 to view scene ==="
echo 'ros2 launch floating_camera camera_world.launch.py camera_initial_pose:="[0.0, 0.0, 5.0, 0.0, 0.0, 0.0]"'
echo ""

# 7. Rotate camera (pitch down 45 degrees)
echo "=== Launch with camera pitched down 45° (0.785 rad) ==="
echo 'ros2 launch floating_camera camera_world.launch.py camera_initial_pose:="[0.0, 0.0, 5.0, 0.0, 0.785, 0.0]"'
echo ""

# 8. Rotate camera to look left (yaw 90 degrees)
echo "=== Launch with camera rotated left 90° (1.57 rad) ==="
echo 'ros2 launch floating_camera camera_world.launch.py camera_initial_pose:="[0.0, 0.0, 0.0, 0.0, 0.0, 1.57]"'
echo ""

# 9. Custom Gazebo GUI camera position (viewport behind the scene)
echo "=== Launch with custom Gazebo GUI camera position ==="
echo 'ros2 launch floating_camera camera_world.launch.py gui_camera_x:=-8.0 gui_camera_z:=3.0'
echo ""

# 10. GUI camera elevated and pitched down
echo "=== Launch with GUI camera elevated and pitched down ==="
echo 'ros2 launch floating_camera camera_world.launch.py gui_camera_z:=8.0 gui_camera_pitch:=-0.6'
echo ""

# 11. GUI camera from the side
echo "=== Launch with GUI camera to the side ==="
echo 'ros2 launch floating_camera camera_world.launch.py gui_camera_x:=0.0 gui_camera_y:=-8.0 gui_camera_yaw:=1.57'
echo ""
