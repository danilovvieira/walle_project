from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('walle_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'walle.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='walle_description',
            executable='joint_state_publisher.py',
            name='joint_state_publisher',
            output='screen'
        )
    ])
