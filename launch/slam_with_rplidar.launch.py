from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    my_package_share_dir = get_package_share_directory('denstista_ros2_drive')
    slam_config_path = os.path.join(my_package_share_dir, 'config', 'slam_config.yaml')
    script_path = os.path.join(my_package_share_dir, 'scripts','create_map_dir.sh')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[script_path],
            shell=True,
            output='screen'
        ),
        Node(
            package='denstista_ros2_drive',
            executable='odom_publisher',
            name='odom_publisher_left',
            output='screen'
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),


        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_path, {'use_sim_time': False, 'use_odometry': True}],
            remappings=[('scan', '/scan')]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.2', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
        )
    ])
