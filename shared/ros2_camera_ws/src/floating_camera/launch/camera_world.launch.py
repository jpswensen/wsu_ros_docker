from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('floating_camera')
    world_path = os.path.join(pkg_share, 'floating_camera', 'world_with_plugin.world')
    intrinsics_default = os.path.join(pkg_share, 'config', 'camera_intrinsics.yaml')

    # Default sphere positions from the image corners (12 corner points from 3 squares)
    default_positions = '[[-3.89273,0.24558,10],[-2.74558,-1.39273,10],[-2.25442,1.39273,10],[-1.10727,-0.24558,10],[-1.28171,-0.59767,12],[0.59767,-1.28171,12],[1.28171,0.59767,12],[-0.59767,1.28171,12],[2.63398,0.36603,16],[3.63397,-1.36603,16],[5.36602,-0.36603,16],[4.36603,1.36603,16]]'

    return LaunchDescription([
        DeclareLaunchArgument('num_spheres', default_value='16'),
        DeclareLaunchArgument('sphere_radius', default_value='0.05'),
        DeclareLaunchArgument('sphere_distance', default_value='4.0'),
        DeclareLaunchArgument(
            'random_spheres',
            default_value='False',
            description='Use random grid layout (True) or explicit positions (False)'
        ),
        DeclareLaunchArgument(
            'sphere_positions',
            default_value=default_positions,
            description='YAML string of sphere positions when random_spheres=False'
        ),
        DeclareLaunchArgument('camera_update_rate', default_value='30.0'),
        DeclareLaunchArgument('intrinsics_file', default_value=intrinsics_default),
        DeclareLaunchArgument(
            'camera_initial_pose',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]',
            description='Camera pose [x,y,z,angle,ax,ay,az] in axis-angle format (optical frame)'
        ),
        DeclareLaunchArgument('camera_initial_velocity', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'),
        
        # Gazebo GUI camera (viewport) settings
        DeclareLaunchArgument(
            'gui_camera_x',
            default_value='0.0',
            description='Gazebo GUI camera X position (world frame)'
        ),
        DeclareLaunchArgument(
            'gui_camera_y',
            default_value='0.0',
            description='Gazebo GUI camera Y position (world frame)'
        ),
        DeclareLaunchArgument(
            'gui_camera_z',
            default_value='-7.0',
            description='Gazebo GUI camera Z position (world frame)'
        ),
        DeclareLaunchArgument(
            'gui_camera_roll',
            default_value='0.0',
            description='Gazebo GUI camera roll (radians)'
        ),
        DeclareLaunchArgument(
            'gui_camera_pitch',
            default_value='-1.5707963268',
            description='Gazebo GUI camera pitch (radians)'
        ),
        DeclareLaunchArgument(
            'gui_camera_yaw',
            default_value='1.5707963268',
            description='Gazebo GUI camera yaw (radians)'
        ),
        DeclareLaunchArgument(
            'gui_camera_delay',
            default_value='3.0',
            description='Delay in seconds before setting GUI camera view'
        ),
        
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
                'random_spheres': LaunchConfiguration('random_spheres'),
                'sphere_positions': ParameterValue(LaunchConfiguration('sphere_positions'), value_type=str),
                'camera_update_rate': LaunchConfiguration('camera_update_rate'),
                'intrinsics_file': LaunchConfiguration('intrinsics_file'),
                'camera_initial_pose': LaunchConfiguration('camera_initial_pose'),
                'camera_initial_velocity': LaunchConfiguration('camera_initial_velocity'),
            }]
        ),
        
        # Set Gazebo GUI camera view automatically
        Node(
            package='floating_camera',
            executable='gazebo_view_setter.py',
            output='screen',
            parameters=[{
                'gui_camera_x': LaunchConfiguration('gui_camera_x'),
                'gui_camera_y': LaunchConfiguration('gui_camera_y'),
                'gui_camera_z': LaunchConfiguration('gui_camera_z'),
                'gui_camera_roll': LaunchConfiguration('gui_camera_roll'),
                'gui_camera_pitch': LaunchConfiguration('gui_camera_pitch'),
                'gui_camera_yaw': LaunchConfiguration('gui_camera_yaw'),
                'delay': LaunchConfiguration('gui_camera_delay'),
            }]
        ),
    ])
