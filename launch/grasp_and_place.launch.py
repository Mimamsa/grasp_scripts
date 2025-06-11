from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    node = Node(package='grasp_scripts',
                 executable='grasp_and_place',
                 name='grasp_and_place',
                 #parameters=[{'': ''}],
                 output='screen',)

    return LaunchDescription([
        node
    ])
