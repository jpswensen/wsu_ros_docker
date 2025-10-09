from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('floating_camera')
    world_path = os.path.join(pkg_share, 'floating_camera', 'world_with_plugin.world')
    intrinsics_default = os.path.join(pkg_share, 'config', 'camera_intrinsics.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('num_spheres', default_value='16'),
        DeclareLaunchArgument('sphere_radius', default_value='0.05'),
        DeclareLaunchArgument('sphere_distance', default_value='4.0'),
        DeclareLaunchArgument('camera_update_rate', default_value='30.0'),
        DeclareLaunchArgument('intrinsics_file', default_value=intrinsics_default),
        DeclareLaunchArgument('camera_initial_pose', default_value='[0.0, 0.0, 2.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('camera_initial_velocity', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'),
        # Launch Gazebo as a system with correct plugins
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_path
            ],
            output='screen'
        ),

        # Spawn the camera after Gazebo is ready
        Node(
            package='floating_camera',
            executable='spawn_camera.py',
            output='screen',
            parameters=[{
                'num_spheres': LaunchConfiguration('num_spheres'),
                'sphere_radius': LaunchConfiguration('sphere_radius'),
                'sphere_distance': LaunchConfiguration('sphere_distance'),
                'camera_update_rate': LaunchConfiguration('camera_update_rate'),
                'intrinsics_file': LaunchConfiguration('intrinsics_file'),
                'camera_initial_pose': LaunchConfiguration('camera_initial_pose'),
                'camera_initial_velocity': LaunchConfiguration('camera_initial_velocity'),
            }]
        ),
    ])
