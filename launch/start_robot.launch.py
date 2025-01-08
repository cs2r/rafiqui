from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_node = Node(
        package='rafiqui',
        executable='ros_driver',
        name='robot_node',
        output='screen'
    )

    tcp_endpoint_launch_path = os.path.join(
        get_package_share_directory('ros_tcp_endpoint'),
        'launch',
        'endpoint.py'
    )
    tcp_endpoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tcp_endpoint_launch_path)
    )
    
    return LaunchDescription([
        robot_node,
        tcp_endpoint_launch
    ])
    
    
