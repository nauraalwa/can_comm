from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

"""
launch nodes:
- urg_node2 urg_node2.launch.py: lidar scanning for points
- can_comm lidar_test: detect walls and poles
see saitos code for integration of can
- ros2_socketcan: both receive and send
- can_comm can_translator: bridge between ROS2 and STM32 thru CAN
"""

def generate_launch_description():
    urg_node2_launch_path = os.path.join(
        get_package_share_directory('urg_node2'), # Check if package name is exactly urg_node2
        'launch',
        'urg_node2.launch.py'
    )

    socket_can_receiver_launch_path = os.path.join(
        get_package_share_directory('ros2_socketcan'),
        'launch',
        'socket_can_receiver.launch.py'
    )

    socket_can_sender_launch_path = os.path.join(
        get_package_share_directory('ros2_socketcan'),
        'launch',
        'socket_can_sender.launch.py'
    )

    can_translator_node = Node(
        package= 'can_comm',
        name= 'can_translator',
        executable= 'can_translator'
    )

    lidar_test_node = Node(
        package= 'can_comm',
        name= 'lidar_test',
        executable= 'lidar_test'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_node2_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(socket_can_receiver_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(socket_can_sender_launch_path),
        ),
        can_translator_node,
        lidar_test_node
    ])